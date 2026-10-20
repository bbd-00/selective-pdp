# branch_and_price.py
"""
Branch and Price for SPDPTW with two courier types.

Master Problem (Restricted Master Problem, RMP):
  max  Σ_col  profit(col) · λ[col]
  s.t.
    cov[i] :  Σ_{col: i ∈ col.requests}  λ[col]  ≤ 1   ∀ i ∈ P
    use[k] :  Σ_{col: col.k == k}         λ[col]  = 1   ∀ k ∈ K   (every courier gets exactly one route)
    λ[col] ∈ [0, 1]   (LP relaxation; binary in MIP phase)

  Every courier — committed or occasional — is always assigned exactly one route.
  The empty route (no requests served) is always included as an initial column for
  every courier so the RMP is feasible from the start (Remark 1).

  Column profit is the exact route profit as returned by GraphSearch
  (revenue minus all vehicle costs, including committed courier fixed block cost).

Pricing Problem (per courier k):
  Find the feasible route for courier k maximizing:
      route_profit(route) − Σ_{i ∈ route} π[i]
  where π[i] is the "pricing adjustment" for request i
  (the dual of cov[i] in the current RMP LP solution).

  GraphSearch solves this directly: setting u[i] = π[i] causes it to subtract
  π[i] from each request's revenue.  The return value objVal already equals
  route_profit − Σ π[i], so the reduced cost is simply:
      rc = objVal − μ[k]
  where μ[k] is the dual of use[k].  A column is added when rc > ε.

Branching (conflict branching with forcing):
  S = {k : 0 < f_{ki*} < 1} for the request i* with the largest |S| (|S| ≥ 1 whenever
  some courier-request assignment f_{ki} is fractional); ties go to the largest dual π_i,
  then the smallest index.
  |S|+1 children that partition the solution space:
    for each k* in S: force f_{k*i*} = 1 (FORCE row in the master) and forbid i* for
                      every other courier;
    one more child  : forbid i* for every courier in S.
  With |S| = 1 this is the two-way branch "k* must / must not serve i*".
  Forcing lives in the master only; its dual σ is added to courier k*'s pricing duals,
  so the pricing solvers need no change. Forbidding sets z_UB = 0 in pricing.

Node state: forbidden_pairs and forced_pairs (frozensets of (i,k) tuples).
"""

import os
import sys
import math
import time
import heapq

import gurobipy as gp
from gurobipy import GRB

# ─────────────────────────────────────────────────────────────────────────────
# Solver parameters
# ─────────────────────────────────────────────────────────────────────────────

DEFAULT_TIME_LIMIT      = 3600   # global wall-clock budget (seconds)
DEFAULT_MIP_TIME        = 20.0   # seconds given to RMP MIP at each B&B node
DEFAULT_USE_MIP_HEURISTIC = True # False: skip the per-node MIP phase entirely; incumbents then
                                 # come only from nodes whose LP solution is integral
DEFAULT_ADD_ALL_COLUMNS = True  # False: add only first positive-rc column, then re-solve LP
                                 # True:  add every positive-rc column per pricing round
DEFAULT_PRICING_ALGO    = "graphsearch"

# ─────────────────────────────────────────────────────────────────────────────
# Pricing algorithm identifiers
# ─────────────────────────────────────────────────────────────────────────────

PRICING_GRAPHSEARCH = "graphsearch"  # DFS with bound pruning (graph_search.py)
PRICING_LABELING    = "labeling"     # ESPPRC labeling with dominance (labeling.py)

# Repo root (instance_loader), then this folder first (graph_search, labeling, pricing_bounds)
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from instance_loader import load_instance_from_txt
from preprocessing import preprocess_instance
from graph_search import GraphSearchSolver
from labeling import LabelingSolver

_PRICING_SOLVERS = {
    PRICING_GRAPHSEARCH: GraphSearchSolver,
    PRICING_LABELING:    LabelingSolver,
}


# ─────────────────────────────────────────────────────────────────────────────
# Column data structure
# ─────────────────────────────────────────────────────────────────────────────

class Column:
    """
    One feasible single-vehicle route.

    base_profit   : true route profit (revenue − all costs). Used as the RMP
                    objective coefficient. This is the original profit for the route for that courier as given by the OG model.
    pricing_objval: base_profit − Σ π[i] at the time this column was generated.
                    Used only to compute reduced cost during pricing. Basically, base_profit minus the sum of pis.
    """
    __slots__ = ("k", "requests", "route", "base_profit", "pricing_objval")

    def __init__(self, k, requests, route, base_profit, pricing_objval):
        self.k = k
        self.requests = frozenset(requests)
        self.route = list(route)
        self.base_profit = base_profit
        self.pricing_objval = pricing_objval

    def __repr__(self):
        return (f"Column(k={self.k}, route={self.route}, "
                f"reqs={sorted(self.requests)}, profit={self.base_profit:.3f})")


# ─────────────────────────────────────────────────────────────────────────────
# Precompute all instance-level data that stays fixed across nodes
# ─────────────────────────────────────────────────────────────────────────────

def _precompute(instance, prep):
    """Instance-level data fixed across nodes. `instance` is the preprocessed instance
    (prep["instance"]); elimination results come from prep (preprocessing.preprocess_instance)."""
    n = len(instance["requests"])
    P = list(range(1, n + 1))
    D = [i + n for i in P]
    KC = list(range(len(instance["committed"])))
    KO = list(range(len(instance["committed"]),
                    len(instance["committed"]) + len(instance["occasional"])))
    K = KC + KO

    coords = {}
    for req in instance["requests"]:
        coords[req["id"]]     = req["pickup"]
        coords[req["id"] + n] = req["dest"]

    ok = ([d["location"] for d in instance["committed"]] +
          [d["location"] for d in instance["occasional"]])

    A2_full = [(i, j) for i in P + D for j in P + D if i != j]

    ei, li, si = {}, {}, {}
    for req in instance["requests"]:
        pid = req["id"]
        ei[pid],     li[pid]     = req["e_pickup"], req["l_pickup"]
        ei[pid + n], li[pid + n] = req["e_dest"],   req["l_dest"]
        si[pid],     si[pid + n] = req["delay_p"],   req["delay_d"]

    dij = {(i, j): math.dist(coords[i], coords[j]) for (i, j) in A2_full}
    dijk = {(0, j, k): math.dist(ok[k], coords[j]) for j in P for k in K}

    # Courier-request (z_UB) and arc (x_UB, x_ind) elimination, shared with the exact MIP
    z_UB, x_UB, x_ind = prep["z_UB"], prep["x_UB"], prep["x_ind"]
    all_couriers = instance["committed"] + instance["occasional"]

    return dict(n=n, P=P, D=D, KC=KC, KO=KO, K=K,
                coords=coords, z_UB=z_UB, x_UB=x_UB, x_ind=x_ind,
                ei=ei, li=li, si=si, dij=dij, dijk=dijk,
                all_couriers=all_couriers)


# ─────────────────────────────────────────────────────────────────────────────
# Pricing oracle  (one call per courier per CG iteration)
# ─────────────────────────────────────────────────────────────────────────────

def _price_column(instance, precomp, k, pi, forbidden_pairs,
                  pricing_algo=DEFAULT_PRICING_ALGO, pricing_stats=None):
    """
    Run the selected pricing solver (GraphSearch or labeling) for courier k.

    pi             : dict {request_id: pricing_adjustment}  (π from the RMP LP duals)
    forbidden_pairs: frozenset of (i, k) tuples — courier k cannot serve request i
    pricing_stats  : optional dict; the solver's bound-pruning counters and its
                     eligible / dual-excluded request counts are added to it

    Returns a Column. Caller compares reduced cost against threshold.
    """
    z_UB_node = dict(precomp["z_UB"])
    for (i, k_forbidden) in forbidden_pairs:
        if k_forbidden == k:
            z_UB_node[i, k] = 0

    solver = _PRICING_SOLVERS[pricing_algo](
        instance_data=instance,
        vehicle_k=k,
        z_UB=z_UB_node,
        x_UB=precomp["x_UB"],
        x_ind=precomp["x_ind"],
        multipliers=pi,   # GraphSearch subtracts pi[i] from each request's revenue to form the adjusted revenue
        verbose=False,
    )
    result = solver.solve()

    if pricing_stats is not None:
        pricing_stats["bound_checks"] += result["n_bound_checks"]
        pricing_stats["bound_pruned"] += result["n_bound_pruned"]
        pricing_stats["eligible"]     += result["n_eligible"]
        pricing_stats["excluded"]     += result["n_excluded"]

    served = list(result["served_pickups"])
    pricing_objval = result["objVal"]          # = base_profit − Σ π[i]
    base_profit    = pricing_objval + sum(pi.get(i, 0.0) for i in served)       # re-adds sum of pis back to get the base_profit

    return Column(
        k=k,
        requests=served,
        route=result["route"],
        base_profit=base_profit,
        pricing_objval=pricing_objval,
    )


def _make_empty_columns(instance, precomp):
    """
    Build one empty-route column per courier.

    For committed couriers: the empty route has negative profit equal to the
    fixed block cost — this is always incurred and must be represented in the RMP.
    For occasional couriers: profit is zero.
    """
    b = int(instance["b"])
    wC = instance["c_com_h"] / b
    cols = []
    for k in precomp["K"]:
        drv = precomp["all_couriers"][k]
        if k in precomp["KC"]:
            # block cost on the original shift, even if preprocessing tightened the window
            profit = -wC * (drv.get("l_orig", drv["l"]) - drv.get("e_orig", drv["e"]))
        else:
            profit = 0.0
        cols.append(Column(k=k, requests=[], route=[],
                           base_profit=profit, pricing_objval=profit))
    return cols


# ─────────────────────────────────────────────────────────────────────────────
# Restricted Master Problem
# ─────────────────────────────────────────────────────────────────────────────

class RMP:
    """
    Gurobi LP representing the restricted set-packing master problem.

    Constraints are built once at construction with empty LHS; column
    coefficients are wired in incrementally via add_column().

    self.lambdas : list of Gurobi Var objects (λ_{kρ}), one per column added.
                   After solve_lp(), v.X gives the primal LP value for that column.
                   Mirrors self.columns index-for-index.
    self.columns : list of Column objects, parallel to self.lambdas.
    self.cov[i]  : Gurobi constraint for COV_i (coverage of request i).
    self.use[k]  : Gurobi constraint for USE_k (courier k assigned exactly one route).
    self.force[(i,k)] : branching constraint FORCE_ik: Σ_{ρ of k, i ∈ ρ} λ_{kρ} = 1
                        (courier k must serve request i), one per forced pair.
    """

    def __init__(self, P, K, KC, KO, forced_pairs=frozenset()):
        self.P  = P
        self.K  = K
        self.KC = KC   # kept for dual sign reference
        self.KO = KO
        self.forced_pairs = forced_pairs
        self.columns = []   # Column objects, parallel to self.lambdas
        self.lambdas = []   # Gurobi Var objects (λ_{kρ}), parallel to self.columns

        self.m = gp.Model("RMP")
        self.m.Params.OutputFlag = 0

        # COV_i: Σ_{k,ρ} a_{iρ} λ_{kρ} ≤ 1
        # LHS starts empty; coefficients are added one term at a time in add_column().
        self.cov = {}
        for i in P:
            self.cov[i] = self.m.addLConstr(
                gp.LinExpr(), GRB.LESS_EQUAL, 1.0, name=f"cov_{i}"
            )

        # USE_k: Σ_{ρ} λ_{kρ} = 1  for every courier k.
        # LHS starts empty; coefficients are added one term at a time in add_column().
        self.use = {}
        for k in K:
            self.use[k] = self.m.addLConstr(
                gp.LinExpr(), GRB.EQUAL, 1.0, name=f"use_{k}"
            )

        # FORCE_ik: courier k's routes that serve request i must carry total weight 1.
        # Together with USE_k this leaves weight only on k's routes containing i.
        self.force = {}
        for (i, k) in forced_pairs:
            self.force[i, k] = self.m.addLConstr(
                gp.LinExpr(), GRB.EQUAL, 1.0, name=f"force_{i}_{k}"
            )

        self.m.ModelSense = GRB.MAXIMIZE

    def add_column(self, col):
        """
        Add a new column λ_{kρ} to the RMP as a continuous variable in [0,1].

        objective coefficient : col.base_profit  (p_{kρ} — true route profit)
        wired into cov[i]     : for each request i ∈ col.requests  (a_{iρ} = 1)
        wired into use[col.k] : coefficient 1  (this route belongs to courier k)
        """
        var = self.m.addVar(
            lb=0.0, ub=1.0,
            obj=col.base_profit,
            name=f"lambda_{len(self.lambdas)}"
        )
        # Add this λ_{kρ} to each coverage constraint for requests it serves
        for i in col.requests:
            self.m.chgCoeff(self.cov[i], var, 1.0)
        # Add this λ_{kρ} to the usage constraint for its courier
        self.m.chgCoeff(self.use[col.k], var, 1.0)
        # Add it to every forcing constraint of its courier whose request it serves
        for i in col.requests:
            if (i, col.k) in self.force:
                self.m.chgCoeff(self.force[i, col.k], var, 1.0)
        self.columns.append(col)
        self.lambdas.append(var)

    def solve_lp(self):
        """
        Solve the LP relaxation of the current RMP.

        Returns (obj, pi, mu, sigma, lambda_vals) or None if infeasible/unbounded.
          pi[i]        = dual of cov[i]  — pricing adjustment for request i
          mu[k]        = dual of use[k]  — threshold offset for reduced cost check
          sigma[(i,k)] = dual of force[(i,k)] — extra adjustment for request i, courier k only
          lambda_vals  = list of primal λ values, parallel to self.columns
        """
        self.m.update()
        self.m.optimize()

        if self.m.Status == GRB.INFEASIBLE:
            return None
        if self.m.Status not in (GRB.OPTIMAL, GRB.SUBOPTIMAL):
            return None

        pi          = {i: self.cov[i].Pi for i in self.P}
        mu          = {k: self.use[k].Pi for k in self.K}
        sigma       = {ik: c.Pi for ik, c in self.force.items()}
        lambda_vals = [v.X for v in self.lambdas]

        return self.m.ObjVal, pi, mu, sigma, lambda_vals

    def solve_mip(self, time_limit=30.0):
        """
        Fix all λ variables to binary and solve the resulting MIP.

        Called once per B&B node after CG converges as a heuristic integer
        recovery step using the column pool generated so far.  Variables are
        restored to continuous afterward so LP solves at child nodes remain valid.

        Returns (obj, lambda_vals) or None if no feasible integer solution found.
        """
        # Temporarily enforce integrality on all columns
        for v in self.lambdas:
            v.VType = GRB.BINARY
        self.m.Params.TimeLimit = time_limit
        self.m.update()
        self.m.optimize()

        result = None
        if self.m.SolCount > 0:
            result = (self.m.ObjVal, [v.X for v in self.lambdas])

        # Restore to continuous so further LP solves remain valid
        for v in self.lambdas:
            v.VType = GRB.CONTINUOUS
        self.m.update()
        return result


# ─────────────────────────────────────────────────────────────────────────────
# Column generation at one B&B node
# ─────────────────────────────────────────────────────────────────────────────

def _courier_duals(pi, sigma, k):
    """Request duals as seen by courier k: π_i, plus σ_ik for requests forced onto k."""
    if not any(kk == k for (_, kk) in sigma):
        return pi
    pi_k = dict(pi)
    for (i, kk), s in sigma.items():
        if kk == k:
            pi_k[i] = pi_k.get(i, 0.0) + s
    return pi_k


def _column_generation(instance, precomp, forbidden_pairs,
                       init_columns, verbose, add_all_columns=DEFAULT_ADD_ALL_COLUMNS,
                       pricing_algo=DEFAULT_PRICING_ALGO, pricing_stats=None,
                       forced_pairs=frozenset()):
    """
    Run column generation at a single B&B node.

    init_columns    : list[Column] — columns inherited from the parent node
    forbidden_pairs : frozenset of (i,k) tuples — courier k cannot serve request i
    forced_pairs    : frozenset of (i,k) tuples — courier k must serve request i, enforced in
                      the master by FORCE_ik; its dual σ_ik is added to courier k's pricing duals
    add_all_columns : if True, add every positive-rc column found in each pricing round;
                      if False, add only the first positive-rc column and immediately
                      re-solve the LP (slower per iteration but tighter dual updates).
    pricing_algo    : PRICING_GRAPHSEARCH or PRICING_LABELING
    pricing_stats   : optional dict {"time", "calls", "bound_checks", "bound_pruned"}
                      accumulated across nodes by the caller

    Returns (lp_bound, all_columns, pi, mu, lambda_vals, rmp, cg_iters, n_new_cols, cg_time)
    or None if LP is infeasible.
    """
    cg_start = time.time()

    P  = precomp["P"]
    K  = precomp["K"]
    KC = precomp["KC"]
    KO = precomp["KO"]

    # Filter inherited columns: drop any where courier c.k is forbidden from
    # serving any request it covers.
    columns = [c for c in init_columns
               if not any((i, c.k) in forbidden_pairs for i in c.requests)]

    # # NOTE: THIS SHOULD BE REDUNDANT, SO I REMOVED IT
    # # Ensure every courier has at least one column (empty route); use[k] = 1 -
    # # for all k means the LP is infeasible without at least one column per courier.
    # present_k = {c.k for c in columns}
    # empty_cols = _make_empty_columns(instance, precomp)
    # for e in empty_cols:
    #     if e.k not in present_k:
    #         columns.append(e)

    rmp = RMP(P, K, KC, KO, forced_pairs)
    for col in columns:
        rmp.add_column(col)

    iteration  = 0
    n_new_cols = 0
    while True:
        iteration += 1

        lp_result = rmp.solve_lp()
        if lp_result is None:
            return None   # LP infeasible — prune this node

        lp_obj, pi, mu, sigma, lambda_vals = lp_result

        new_col_added = False
        for k in K:
            _t = time.perf_counter()
            col = _price_column(instance, precomp, k, _courier_duals(pi, sigma, k),
                                forbidden_pairs, pricing_algo, pricing_stats)
            if pricing_stats is not None:
                pricing_stats["time"]  += time.perf_counter() - _t
                pricing_stats["calls"] += 1

            # Reduced cost = pricing_objval − μ[k]
            rc = col.pricing_objval - mu[k]
            if rc > 1e-6:
                rmp.add_column(col)
                new_col_added = True
                n_new_cols   += 1
                if not add_all_columns:
                    break   # re-solve LP with this single new column

        if not new_col_added:
            if verbose:
                print(f"    [CG] converged after {iteration} iterations, "
                      f"LP bound = {lp_obj:.4f}")
            break

    # Final LP solve after adding all columns
    lp_result = rmp.solve_lp()
    if lp_result is None:
        return None

    lp_obj, pi, mu, _sigma, lambda_vals = lp_result
    cg_time = time.time() - cg_start

    return lp_obj, rmp.columns, pi, mu, lambda_vals, rmp, iteration, n_new_cols, cg_time


# ─────────────────────────────────────────────────────────────────────────────
# Branching helpers
# ─────────────────────────────────────────────────────────────────────────────


def _compute_fki(lambda_vals, columns, K, P):
    """f_{ki} = Σ_{ρ: i ∈ R(ρ), col.k == k} λ[col]"""
    fki = {(k, i): 0.0 for k in K for i in P}
    for j, col in enumerate(columns):
        for i in col.requests:
            fki[col.k, i] += lambda_vals[j]
    return fki


def _find_conflict(lambda_vals, columns, K, P, forbidden_pairs, pi):
    """
    Find request i* with the most couriers having fractional f_{ki} ∈ (0,1),
    i.e. the largest conflict set S = {k : 0 < f_{ki*} < 1}. Ties are broken by the
    largest dual π_i (the request the fleet values most), then by the smallest index.
    Returns (i*, S) or None if all f_{ki} are integer.
    """
    fki     = _compute_fki(lambda_vals, columns, K, P)
    best_i  = None
    best_S  = []
    for i in P:
        S = [k for k in K
             if (i, k) not in forbidden_pairs
             and 1e-6 < fki[k, i] < 1 - 1e-6]
        if not S:
            continue
        # Strict comparison keeps the smallest index among exact ties.
        if best_i is None or (len(S), pi[i]) > (len(best_S), pi[best_i]):
            best_S = S
            best_i = i
    if best_i is None:
        return None
    return best_i, best_S


def _is_integer(lambda_vals, tol=1e-5):
    return all(v < tol or v > 1 - tol for v in lambda_vals)


def _extract_solution(lambda_vals, columns):
    """Return routes dict {k: [node, ...]} for all couriers with λ ≈ 1. Runs after solve_mip and _is_integer to extract integer solutions."""
    routes = {}
    for j, col in enumerate(columns):
        if lambda_vals[j] > 0.5:
            routes[col.k] = col.route
    return routes


# ─────────────────────────────────────────────────────────────────────────────
# Branching dispatch
# ─────────────────────────────────────────────────────────────────────────────

def _push_node(heap, lp_obj, next_nid, forbidden_pairs, forced_pairs, columns):
    heapq.heappush(heap, (-lp_obj, next_nid(), frozenset(forbidden_pairs),
                          frozenset(forced_pairs), list(columns)))


def _push_children(heap, lambda_vals, columns, K, P, forbidden_pairs,
                   forced_pairs, pi, lp_obj, next_nid, verbose, label=""):
    """
    Conflict branching with forcing, on the request i* with the largest conflict set S.
    Creates |S| + 1 children that partition the solution space:
      one child per k* in S : k* must serve i* (FORCE constraint in the master) and every
                              other courier is forbidden from i*;
      one extra child       : no courier in S may serve i* (others outside S still may).
    Every child cuts off the current solution, and each forced child stays feasible with the
    inherited columns (some column of k* with i* had positive weight at the parent).
    Returns False if every f_{ki} is integral (nothing to branch on).
    """
    result = _find_conflict(lambda_vals, columns, K, P, forbidden_pairs, pi)
    if result is None:
        return False
    i_star, S = result
    if verbose:
        fki = _compute_fki(lambda_vals, columns, K, P)
        print(f"  [{label}] Conflict branch on i={i_star}, "
              f"S={S}, f_ki={[round(fki[k, i_star], 3) for k in S]}")
    for k_star in S:
        _push_node(heap, lp_obj, next_nid,
                   forbidden_pairs | {(i_star, k) for k in K if k != k_star},
                   forced_pairs | {(i_star, k_star)}, columns)
    _push_node(heap, lp_obj, next_nid,
               forbidden_pairs | {(i_star, k) for k in S}, forced_pairs, columns)
    return True


# ─────────────────────────────────────────────────────────────────────────────
# Main Branch and Price solver
# ─────────────────────────────────────────────────────────────────────────────

def solve_instance_bnp(instance_path,
                       time_limit=DEFAULT_TIME_LIMIT,
                       mip_time_per_node=DEFAULT_MIP_TIME,
                       use_mip_heuristic=DEFAULT_USE_MIP_HEURISTIC,
                       add_all_columns=DEFAULT_ADD_ALL_COLUMNS,
                       pricing_algo=DEFAULT_PRICING_ALGO,
                       gap_tol=0.0,
                       verbose=True,
                       tighten=False):
    """
    Branch and Price solver for SPDPTW with two courier types.

    Parameters
    ----------
    instance_path    : path to instance .txt file
    time_limit       : wall-clock budget in seconds
    mip_time_per_node: seconds given to the RMP MIP at each node for integer recovery
    use_mip_heuristic: if False, skip the MIP phase at every node (B&P stays exact;
                       incumbents then come only from integral LP solutions)
    pricing_algo     : pricing solver, PRICING_GRAPHSEARCH or PRICING_LABELING
    gap_tol          : stop early when gap ≤ gap_tol  (0.0 = proven optimality)
    tighten          : tighten time windows in preprocessing (preprocessing.preprocess_instance,
                       shared with solve_exact_mip); its time is prep_time, not in total_time

    Returns
    -------
    dict with keys:
      best_primal   — best integer objective found
      lp_root_bound — LP bound at the root (fixed reference; not used for gap)
      best_ub       — tightest valid UB at termination: max LP bound over open nodes
                      (equals lp_root_bound when tree is fully enumerated)
      best_gap (%)  — (best_ub − best_primal) / |best_primal| × 100  at termination
      n_nodes       — total B&B nodes solved
      n_cols_root   — columns in the RMP after root CG
      total_time (s)
      status_msg
      routes        — dict {courier_k: [node_sequence]} for best solution
      prep_time, prep_stats — preprocessing time and statistics
    """
    if pricing_algo not in _PRICING_SOLVERS:
        raise ValueError(f"Unknown pricing_algo {pricing_algo!r}; "
                         f"expected one of {sorted(_PRICING_SOLVERS)}")

    if verbose:
        print(f"\n{'='*65}")
        print(f"Branch and Price: {os.path.basename(instance_path)}  (pricing: {pricing_algo})")
        print(f"{'='*65}")

    # Shared preprocessing (elimination, plus time-window tightening when tighten=True); its time
    # is reported separately as prep_time and is not part of total_time
    prep     = preprocess_instance(load_instance_from_txt(instance_path), tighten=tighten)
    instance = prep["instance"]
    t_start  = time.time()
    precomp  = _precompute(instance, prep)
    P, K, KC, KO = precomp["P"], precomp["K"], precomp["KC"], precomp["KO"]
    n = precomp["n"]

    if verbose:
        print(f"Requests={n}, CDR={len(KC)}, ODR={len(KO)}, "
              f"Total couriers={len(K)}")

    best_primal   = -math.inf
    time_to_best  = None   # seconds from start until the final best incumbent was found
    best_routes   = None
    n_nodes       = 0
    lp_root_bound = None
    # Statistics counters
    n_pruned          = 0   # nodes pruned by bound before running CG
    n_lp_infeasible   = 0   # nodes pruned because CG returned infeasible LP
    n_mip_improvements= 0   # times solve_mip found a new incumbent
    n_cols_total      = 0   # new columns generated across all nodes (excl. root inherited)
    n_cg_iters_root   = 0   # CG iterations at root
    n_cg_iters_total  = 0   # CG iterations summed across all nodes
    total_cg_time     = 0.0 # wall time spent in column generation
    total_mip_time    = 0.0 # wall time spent in solve_mip calls
    # Pricing solver time/calls across all nodes, the solvers' bound-pruning counters, and
    # eligible requests per call vs. those excluded by the dual-based (gain <= 0) elimination
    pricing_stats     = {"time": 0.0, "calls": 0, "bound_checks": 0, "bound_pruned": 0,
                         "eligible": 0, "excluded": 0}
    root_gap          = None
    _node_id_ctr = [0]

    # This assigns a unique integer ID to each node as it is created for logging
    def next_nid():
        _node_id_ctr[0] += 1
        return _node_id_ctr[0]

    # Priority queue entries: (-lp_bound, node_id, forbidden_pairs, forced_pairs, init_columns)
    # forbidden_pairs : frozenset of (i,k) tuples — courier k may not serve request i
    # forced_pairs    : frozenset of (i,k) tuples — courier k must serve request i
    heap = []

    # ── Root node ──────────────────────────────────────────────────────────
    deadline = t_start + time_limit
    if verbose:
        print("\n[Root] Running initial column generation ...")

    init_cols = _make_empty_columns(instance, precomp)
    root_result = _column_generation(
        instance, precomp,
        forbidden_pairs=frozenset(),
        init_columns=init_cols,
        verbose=verbose,
        add_all_columns=add_all_columns,
        pricing_algo=pricing_algo,
        pricing_stats=pricing_stats,
    )

    if root_result is None:
        elapsed = time.time() - t_start
        if verbose:
            print("[Root] LP infeasible — problem has no feasible solution.")
        return dict(best_primal=None, lp_root_bound=None,
                    best_gap=None, n_nodes=0, n_cols_root=0,
                    total_time=elapsed, status_msg="INFEASIBLE", routes=None,
                    prep_time=prep["stats"]["prep_time"], prep_stats=prep["stats"])

    lp_obj, cols_root, pi, mu, lambda_vals, rmp_root, cg_iters, n_new, cg_t = root_result
    lp_root_bound    = lp_obj
    n_nodes         += 1
    n_cg_iters_root  = cg_iters
    n_cg_iters_total += cg_iters
    n_cols_total    += n_new
    total_cg_time   += cg_t

    if verbose:
        print(f"[Root] LP bound = {lp_root_bound:.4f}, "
              f"columns in RMP = {len(cols_root)}, CG iters = {n_cg_iters_root}")

    # Tries the MIP phase to attempt to get an integer solution from root RMP columns
    mip_result = None
    if use_mip_heuristic:
        _t = time.time()
        mip_result = rmp_root.solve_mip(time_limit=mip_time_per_node)
        total_mip_time += time.time() - _t
    if mip_result:
        mip_obj, mip_lam = mip_result
        if mip_obj > best_primal:
            if mip_obj > best_primal + 1e-6:
                time_to_best = time.time() - t_start
            best_primal = mip_obj
            best_routes = _extract_solution(mip_lam, cols_root)
            n_mip_improvements += 1
            if verbose:
                print(f"[Root MIP] Integer solution found: {best_primal:.4f}")

    # Checks if LP solution of MP at the root node is integral (in which case we are lucky)
    if _is_integer(lambda_vals):
        lp_int_val = sum(lambda_vals[j] * cols_root[j].base_profit
                         for j in range(len(lambda_vals)))
        if lp_int_val > best_primal:
            if lp_int_val > best_primal + 1e-6:
                time_to_best = time.time() - t_start
            best_primal = lp_int_val
            best_routes = _extract_solution(lambda_vals, cols_root)
            if verbose:
                print(f"[Root] LP solution is already integer: {best_primal:.4f}")
    else:
        # Branch at root only if LP solution is fractional
        _push_children(heap, lambda_vals, cols_root, K, P, frozenset(), frozenset(), pi,
                       lp_root_bound, next_nid, verbose, label="Root")

    # Root gap: gap after root CG + MIP but before any branching
    if lp_root_bound is not None and best_primal > -math.inf and abs(best_primal) > 1e-9:
        root_gap = (lp_root_bound - best_primal) / abs(best_primal) * 100

    # ── B&B loop ───────────────────────────────────────────────────────────
    while heap:
        if time.time() > deadline:
            break

        # Dynamic gap check: best open-node UB is the heap root (min-heap on -lp_bound)
        if best_primal > -math.inf and abs(best_primal) > 1e-9:
            current_ub  = -heap[0][0]
            current_gap = abs(current_ub - best_primal) / abs(best_primal) * 100
            if current_gap <= gap_tol:
                if verbose:
                    print(f"  [B&P] Gap tolerance reached: {current_gap:.4f}% ≤ {gap_tol}%")
                break

        # Pop the node with the highest LP bound (best-first). Bounds are stored
        # negated so the min-heap returns the largest bound first.
        neg_bound, nid, forbidden_pairs, forced_pairs, parent_cols = heapq.heappop(heap)
        node_ub = -neg_bound

        # Prune by bound
        if node_ub <= best_primal + 1e-6:
            if verbose:
                print(f"  [Node {nid}] Pruned by bound "
                      f"(UB={node_ub:.4f} ≤ best={best_primal:.4f})")
            n_pruned += 1
            continue

        if verbose:
            print(f"\n  [Node {nid}] "
                  f"forbidden_pairs={sorted(forbidden_pairs)}  "
                  f"forced_pairs={sorted(forced_pairs)}  UB={node_ub:.4f}")

        node_result = _column_generation(
            instance, precomp,
            forbidden_pairs=forbidden_pairs,
            forced_pairs=forced_pairs,
            init_columns=parent_cols,
            verbose=verbose,
            add_all_columns=add_all_columns,
            pricing_algo=pricing_algo,
            pricing_stats=pricing_stats,
        )

        n_nodes += 1

        if node_result is None:
            if verbose:
                print(f"  [Node {nid}] LP infeasible — pruned")
            n_lp_infeasible += 1
            continue

        lp_obj, cols_node, pi, mu, lambda_vals, rmp_node, node_cg_iters, n_new, cg_t = node_result
        n_cols_total     += n_new
        n_cg_iters_total += node_cg_iters
        total_cg_time    += cg_t

        if verbose:
            print(f"  [Node {nid}] LP bound = {lp_obj:.4f}, "
                  f"cols = {len(cols_node)}")

        # Prune by tightened bound
        if lp_obj <= best_primal + 1e-6:
            if verbose:
                print(f"  [Node {nid}] Pruned (LP={lp_obj:.4f})")
            n_pruned += 1
            continue

        # Try MIP solve phase on this node's columns
        mip_result = None
        if use_mip_heuristic:
            _t = time.time()
            mip_result = rmp_node.solve_mip(time_limit=mip_time_per_node)
            total_mip_time += time.time() - _t
        if mip_result:
            mip_obj, mip_lam = mip_result
            if mip_obj > best_primal:
                if mip_obj > best_primal + 1e-6:
                    time_to_best = time.time() - t_start
                best_primal = mip_obj
                best_routes = _extract_solution(mip_lam, cols_node)
                n_mip_improvements += 1
                if verbose:
                    print(f"  [Node {nid}] New best integer: {best_primal:.4f}")

        # Check if node's solution is integer (lambdas are all integral)
        if _is_integer(lambda_vals):
            lp_int_val = sum(lambda_vals[j] * cols_node[j].base_profit
                             for j in range(len(lambda_vals)))
            if lp_int_val > best_primal:
                if lp_int_val > best_primal + 1e-6:
                    time_to_best = time.time() - t_start
                best_primal = lp_int_val
                best_routes = _extract_solution(lambda_vals, cols_node)
                if verbose:
                    print(f"  [Node {nid}] LP integer solution: {best_primal:.4f}")
        else:
            _push_children(heap, lambda_vals, cols_node, K, P, forbidden_pairs, forced_pairs, pi,
                           lp_obj, next_nid, verbose, label=f"Node {nid}")

    # ── Final reporting ────────────────────────────────────────────────────
    elapsed   = time.time() - t_start
    remaining = len(heap)

    # Tightest UB at termination: best open node if heap non-empty, else tree is
    # fully exhausted and the solution is proven optimal (best_ub == best_primal).
    best_ub = (-heap[0][0]) if heap else best_primal

    if best_primal == -math.inf:
        best_primal = None

    if best_ub is not None and best_primal is not None and abs(best_primal) > 1e-9:
        gap = abs(best_ub - best_primal) / abs(best_primal) * 100
    else:
        gap = None

    if remaining == 0 and best_primal is not None:
        status = "OPTIMAL"
    elif best_primal is not None:
        status = "TIME_LIMIT"
    else:
        status = "NO_SOLUTION"

    gap_improvement = (root_gap - gap) if (root_gap is not None and gap is not None) else None

    if verbose:
        print(f"\n{'='*65}")
        print(f"B&P done:  status={status}  nodes={n_nodes}  "
              f"time={elapsed:.1f}s")
        print(f"  LP root bound : {lp_root_bound:.4f}" if lp_root_bound else "  LP root bound: N/A")
        print(f"  Best UB       : {best_ub:.4f}"        if best_ub       else "  Best UB: N/A")
        print(f"  Best integer  : {best_primal:.4f}"    if best_primal is not None else "  Best integer: None")
        print(f"  Root gap      : {root_gap:.2f}%"      if root_gap is not None else "  Root gap: N/A")
        print(f"  Final gap     : {gap:.2f}%"           if gap is not None else "  Final gap: N/A")
        print(f"  Gap improved  : {gap_improvement:.2f}%" if gap_improvement is not None else "  Gap improved: N/A")
        print(f"  Pruned nodes  : {n_pruned}  LP-infeasible: {n_lp_infeasible}")
        print(f"  MIP improvements: {n_mip_improvements}  New cols (total): {n_cols_total}")
        print(f"  CG iters (root): {n_cg_iters_root}  CG time: {total_cg_time:.1f}s  MIP time: {total_mip_time:.1f}s")
        print(f"  Pricing ({pricing_algo}): {pricing_stats['time']:.2f}s over "
              f"{pricing_stats['calls']} calls")
        print(f"  Bound pruning : {pricing_stats['bound_pruned']} of "
              f"{pricing_stats['bound_checks']} partial routes checked")
        print(f"  Dual exclusion: {pricing_stats['excluded']} of "
              f"{pricing_stats['eligible']} eligible courier-request pairs excluded (summed over calls)")
        print(f"{'='*65}")

    return dict(
        best_primal=best_primal,
        lp_root_bound=lp_root_bound,
        best_ub=best_ub,
        best_gap=gap,
        root_gap=root_gap,
        gap_improvement=gap_improvement,
        n_nodes=n_nodes,
        n_pruned=n_pruned,
        n_lp_infeasible=n_lp_infeasible,
        n_mip_improvements=n_mip_improvements,
        n_cols_root=len(cols_root),
        n_cols_total=n_cols_total,
        n_cg_iters_root=n_cg_iters_root,
        n_cg_iters_total=n_cg_iters_total,
        cg_time=total_cg_time,
        pricing_time=pricing_stats["time"],
        n_pricing_calls=pricing_stats["calls"],
        n_bound_checks=pricing_stats["bound_checks"],
        n_bound_pruned=pricing_stats["bound_pruned"],
        n_requests_eligible=pricing_stats["eligible"],
        n_requests_excluded=pricing_stats["excluded"],
        mip_time=total_mip_time,
        total_time=elapsed,
        time_to_best=time_to_best,
        status_msg=status,
        routes=best_routes,
        prep_time=prep["stats"]["prep_time"],
        prep_stats=prep["stats"],
    )


# ─────────────────────────────────────────────────────────────────────────────
# Quick standalone test
# ─────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    import sys

    if len(sys.argv) > 1:
        path = sys.argv[1]
    else:
        # Default small instance for quick testing
        script_dir = os.path.dirname(os.path.abspath(__file__))
        path = os.path.join(
            script_dir, "..",
            "InstanceswithDemand",
            "Instance_REQ10_CDR2_ODR4_FS0_SS0.txt",
        )
        path = os.path.normpath(path)

    if not os.path.exists(path):
        print(f"Instance not found: {path}")
        print("Usage: python branch_and_price.py <instance_path>")
        sys.exit(1)

    res = solve_instance_bnp(
        path,
        time_limit=DEFAULT_TIME_LIMIT,
        mip_time_per_node=DEFAULT_MIP_TIME,
        verbose=True,
    )

    print("\nResult summary:")
    for k, v in res.items():
        if k != "routes":
            print(f"  {k}: {v}")
    if res.get("routes"):
        print("  routes:")
        for courier, route in sorted(res["routes"].items()):
            print(f"    courier {courier}: {route}")
