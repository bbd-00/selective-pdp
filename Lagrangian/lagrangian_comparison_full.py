# lagrangian_comparison_full.py
#
# Flexible comparison runner.  Define which solver methods to compare in the
# METHODS list below — the sheets, headers, and comparison columns are all
# auto-generated from that list.  Exact MIP is always run as the baseline.
#
# Output: lagrangian_comparison_full.xlsx
#   Sheet "Exact"        — per-instance exact MIP results
#   Sheet "<MethodName>" — one detail sheet per configured method
#   Sheet "Comparison"   — all methods side-by-side per instance

import os
import sys
import math
import time

import gurobipy as gp
from gurobipy import GRB
from openpyxl import Workbook
from openpyxl.utils import get_column_letter
from openpyxl.styles import Font, PatternFill

# Make all solver modules importable regardless of working directory
_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
sys.path.insert(0, os.path.join(os.path.dirname(_HERE), "Branch and Price"))

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # repo root: instance_loader
from instance_loader      import load_instance_from_txt
from lagrangian_bnb_fresh import solve_instance_bnb
from lagrangian_reopt     import solve_instance_reopt
from branch_and_price     import solve_instance_bnp


# ── Tunable globals ───────────────────────────────────────────────────────────
EXACT_TIME_LIMIT = 3600   # seconds for the exact MIP
MAX_REQ          = 60     # largest REQ size to include (10, 20, …, MAX_REQ)
SEED             = 0      # SS index used when building instance filenames
# ─────────────────────────────────────────────────────────────────────────────


# ── Method registry ───────────────────────────────────────────────────────────
# Add, remove, or reorder entries freely.  The comparison file adapts.
#
# Required keys per entry:
#   name        — label used for the sheet name and all column prefixes
#   func        — callable(instance_path, **kwargs) → dict
#   kwargs      — passed verbatim to func on every call
#   key_primal  — result-dict key for the best primal objective value
#   key_bound   — result-dict key for the best upper/Lagrangian bound (or None)
#   key_time    — result-dict key for total wall-clock time
#   key_status  — result-dict key for the status / message string
#   extra_cols  — list of (column_label, result_dict_key) for method-specific stats

METHODS = [
    {
        "name":       "Reopt",
        "func":       solve_instance_reopt,
        "kwargs":     {"max_iters": 100, "verbose": True},
        "key_primal": "best_primal",
        "key_bound":  "best_L",
        "key_time":   "total_time",
        "key_status": "status_msg",
        "extra_cols": [
            ("n_dup_resolved", "n_duplicates_resolved"),
            ("n_inserted",     "n_inserted"),
        ],
    },
    {
        "name":       "BnB_Fresh",
        "func":       solve_instance_bnb,
        "kwargs":     {
            "max_iters_root": 100,
            "max_iters_node": 30,
            "bnb_time_limit": 3600,
            "verbose":        True,
        },
        "key_primal": "best_primal",
        "key_bound":  "best_L",
        "key_time":   "total_time",
        "key_status": "status_msg",
        "extra_cols": [
            ("nodes",    "n_nodes"),
            ("pruned",   "n_pruned"),
            ("feasible", "n_feasible"),
        ],
    },
    {
        "name":       "BnP",
        "func":       solve_instance_bnp,
        "kwargs":     {
            "time_limit":        600,
            "max_cg_iters":      300,
            "mip_time_per_node": 20.0,
            "verbose":           True,
        },
        "key_primal": "best_primal",
        "key_bound":  "lp_root_bound",
        "key_time":   "total_time",
        "key_status": "status_msg",
        "extra_cols": [
            ("nodes",     "n_nodes"),
            ("cols_root", "n_cols_root"),
        ],
    },
]
# ─────────────────────────────────────────────────────────────────────────────


# ── Exact MIP solver ──────────────────────────────────────────────────────────
def solve_exact_mip(instance_path, time_limit=3600, verbose=False):
    t0       = time.time()
    instance = load_instance_from_txt(instance_path)

    b        = int(instance["b"])
    num_comm = len(instance["committed"])
    num_occ  = len(instance["occasional"])
    num_req  = len(instance["requests"])

    P  = list(range(1, num_req + 1))
    D  = [i + num_req for i in P]
    KC = list(range(0, num_comm))
    KO = list(range(num_comm, num_comm + num_occ))
    n  = len(P)
    V  = [0] + P + D + [2 * n + 1]
    K  = KC + KO

    A1 = [(0, j) for j in P]
    A2 = [(i, j) for i in P + D for j in P + D if i != j]
    A3 = [(i, 2 * n + 1) for i in D] + [(0, 2 * n + 1)]

    coords = {}
    for req in instance["requests"]:
        pid = req["id"]
        coords[pid]           = req["pickup"]
        coords[pid + num_req] = req["dest"]

    ok    = ([d["location"] for d in instance["committed"]] +
             [d["location"] for d in instance["occasional"]])
    dij   = {**{(i, j): math.dist(coords[i], coords[j]) for (i, j) in A2},
             **{(i, j): 0 for (i, j) in A3}}
    dijk  = {(i, j, k): math.dist(ok[k], coords[j]) for (i, j) in A1 for k in K}
    tij   = dij.copy()
    tijk  = dijk.copy()

    si = {0: 0, 2 * num_req + 1: 0}
    ei, li, dem = {}, {}, {}
    for req in instance["requests"]:
        pid = req["id"]
        ei[pid], li[pid]               = req["e_pickup"],  req["l_pickup"]
        ei[pid + num_req], li[pid + num_req] = req["e_dest"],    req["l_dest"]
        si[pid]           = req["delay_p"]
        si[pid + num_req] = req["delay_d"]
        dem[pid], dem[pid + num_req] = req["l_dem"], -req["l_dem"]
    dem[0] = 0
    dem[2 * n + 1] = 0

    Ek = ({d["id"]: d["e"] for d in instance["committed"]} |
          {d["id"]: d["e"] for d in instance["occasional"]})
    Lk = ({d["id"]: d["l"] for d in instance["committed"]} |
          {d["id"]: d["l"] for d in instance["occasional"]})
    Qk = ({d["id"]: d["cap"] for d in instance["committed"]} |
          {d["id"]: d["cap"] for d in instance["occasional"]})
    Lk_org = Lk.copy()

    p      = {req["id"]: req["rev"] for req in instance["requests"]}   # per-request revenue
    wC     = instance["c_com_h"] / b
    cO     = instance["c_occ_o"]
    deltaC = instance["c_com_d"]
    deltaO = instance["c_occ_d"]

    z_UB = {
        (pickup, driver): (
            1 if (li[pickup] - si[pickup] >= max(dijk[0, pickup, driver] + Ek[driver], ei[pickup] - si[pickup])
                  and Lk[driver] >= ei[pickup + num_req])
            else 0
        )
        for driver in K for pickup in P
    }

    x_UB = {
        (i, j): 0 if (ei[i] + dij[i, j] + si[j] > li[j]) or (i in D and j == i - num_req) else 1
        for (i, j) in A2
    }
    UBxIPC1 = {
        (n2, d2): 0 if (d2 in D and d2 - num_req != n2
                        and x_UB.get((d2 - num_req, n2), 0) == 0) else 1
        for (n2, d2) in A2
    }
    UBxIPC2 = {
        (p2, n2): 0 if (p2 in P and p2 + num_req != n2
                        and x_UB.get((n2, p2 + num_req), 0) == 0) else 1
        for (p2, n2) in A2
    }
    x_ind = {(i, j): x_UB[i, j] * UBxIPC1[i, j] * UBxIPC2[i, j] for (i, j) in A2}
    A2    = [(i, j) for (i, j) in A2 if x_ind[i, j] == 1]
    A     = A1 + A2 + A3
    PK    = [(pp, kk) for pp in P for kk in K if z_UB[pp, kk] == 1]

    m = gp.Model("exact_mip")
    m.Params.OutputFlag = 1 if verbose else 0
    m.Params.TimeLimit  = time_limit

    x    = m.addVars(A, K, vtype=GRB.BINARY,      name="x")
    Lvar = m.addVars(A2, vtype=GRB.CONTINUOUS,  name="L")
    y    = m.addVars(P + D, K, vtype=GRB.BINARY,   name="y")
    T    = m.addVars(P + D + [0, 2 * n + 1], K, vtype=GRB.CONTINUOUS, lb=0, name="T")

    obj = gp.quicksum(p[i] * gp.quicksum(y[i, k] for k in K) for i in P)
    for k in KC:
        obj -= wC * (Lk_org[k] - Ek[k]) + deltaC * (
            gp.quicksum(dijk[0, j, k] * x[0, j, k] for (_, j) in A1) +
            gp.quicksum(dij[i, j]     * x[i, j, k] for (i, j) in A2)
        )
    for k in KO:
        obj -= cO * gp.quicksum(y[i, k] for i in P) + deltaO * gp.quicksum(
            dij[i, j] * x[i, j, k] for (i, j) in A2
        )
    m.setObjective(obj, GRB.MAXIMIZE)

    for i in P:
        m.addConstr(gp.quicksum(x[i, j, k] for k in K for j in V if (i, j) in A) <= 1)
    for i in P + D:
        for k in K:
            m.addConstr(
                gp.quicksum(x[j, i, k] for j in V if (j, i) in A) -
                gp.quicksum(x[i, j, k] for j in V if (i, j) in A) == 0
            )
    for i in P:
        for k in K:
            m.addConstr(
                gp.quicksum(x[i, j, k] for j in V if (i, j) in A) -
                gp.quicksum(x[j, i + num_req, k] for j in V if (j, i + num_req) in A) == 0
            )
    for k in K:
        m.addConstr(gp.quicksum(x[0, j, k] for j in V if (0, j) in A) == 1)
        m.addConstr(gp.quicksum(x[j, 2 * n + 1, k] for j in V if (j, 2 * n + 1) in A) == 1)
    for i in P + D:
        for k in K:
            m.addConstr(y[i, k] == gp.quicksum(x[i, l, k] for l in V if (i, l) in A))
    for i in P:
        for k in K:
            if z_UB[i, k] == 0:
                m.addConstr(y[i, k] == 0)

    svMTZ = {(i, j): min(-si[i] - dij[j, i], ei[j] - ei[i]) for (i, j) in A2}
    for (i, j) in A2:
        for k in K:
            if (j, i) in A2:
                m.addConstr(
                    T[i, k] - T[j, k] + (tij[i, j] + si[j]) * x[i, j, k]
                    + svMTZ[i, j] * x[j, i, k]
                    <= li[i] * y[i, k] - ei[j] * y[j, k]
                    - (li[i] - ei[j]) * (x[i, j, k] + x[j, i, k])
                )
            else:
                m.addConstr(
                    T[i, k] - T[j, k] + (tij[i, j] + si[j]) * x[i, j, k]
                    <= li[i] * y[i, k] - ei[j] * y[j, k]
                    - (li[i] - ei[j]) * x[i, j, k]
                )
    for (i, j) in A3:
        if i != 0:
            for k in K:
                m.addConstr(
                    T[i, k] - T[j, k] + (tij[i, j] + si[j]) * x[i, j, k]
                    <= li[i] * y[i, k]
                    - Ek[k] * gp.quicksum(x[0, ii, k] for ii in P if (ii, k) in PK)
                    - (li[i] - Ek[k]) * x[i, j, k]
                )
    for (i, j) in A1:
        for k in K:
            m.addConstr(
                T[0, k] - T[j, k] + (tijk[0, j, k] + si[j]) * x[0, j, k]
                <= Lk[k] * gp.quicksum(x[0, ii, k] for ii in P if (ii, k) in PK)
                - ei[j] * y[j, k] - (Lk[k] - ei[j]) * x[0, j, k]
            )
    for i in P + D:
        for k in K:
            m.addConstr(ei[i] * y[i, k] <= T[i, k])
            m.addConstr(T[i, k] <= li[i] * y[i, k])
    for i in P:
        for k in K:
            m.addConstr(T[i, k] + (tij[i, i + num_req] + si[i + num_req]) * y[i, k]
                        <= T[i + num_req, k])
    for k in K:
        m.addConstr(Ek[k] * gp.quicksum(x[0, i, k] for i in P if (i, k) in PK) <= T[0, k])
        m.addConstr(T[2 * n + 1, k]
                    <= Lk[k] * gp.quicksum(x[0, i, k] for i in P if (i, k) in PK))
    # Load flow with one load variable per arc, shared by all couriers (each arc is used by at
    # most one courier, so the sums over k pick the capacity of whichever courier uses it).
    for i in P + D:
        m.addConstr(
            gp.quicksum(Lvar[i, j] for j in P + D if (i, j) in A2) -
            gp.quicksum(Lvar[j, i] for j in P + D if (j, i) in A2)
            == dem[i] * gp.quicksum(x[i, j, k] for k in K for j in V if (i, j) in A)
        )
    for (i, j) in A2:
        m.addConstr(Lvar[i, j] >= gp.quicksum(max(dem[i], 0) * x[i, j, k] for k in K))
        m.addConstr(Lvar[i, j] <= gp.quicksum((Qk[k] + min(dem[i], 0)) * x[i, j, k] for k in K))

    m.optimize()
    elapsed = time.time() - t0

    if m.SolCount == 0:
        return {"obj": None, "mip_gap": None, "status": m.Status, "time": elapsed}
    return {
        "obj":     float(m.objVal),
        "mip_gap": round(float(m.MIPGap) * 100.0, 4),
        "status":  m.Status,
        "time":    round(elapsed, 2),
    }


# ── Excel helpers ─────────────────────────────────────────────────────────────
def _autosize(ws):
    for col in range(1, ws.max_column + 1):
        max_len    = 0
        col_letter = get_column_letter(col)
        for row in range(1, ws.max_row + 1):
            v = ws.cell(row=row, column=col).value
            if v is not None:
                max_len = max(max_len, len(str(v)))
        ws.column_dimensions[col_letter].width = min(max_len + 2, 60)


def _gap_vs_exact(primal, exact_obj):
    if primal is None or exact_obj is None or abs(exact_obj) < 1e-9:
        return None
    return round(100.0 * (exact_obj - primal) / abs(exact_obj), 4)


def _safe(v, decimals=4):
    if v is None:
        return None
    try:
        return round(float(v), decimals)
    except (TypeError, ValueError):
        return v


def _err_result():
    return {}


# ── Header builders ───────────────────────────────────────────────────────────
EXACT_HEADER = [
    "instance_file", "nbr_req", "nbr_cdr", "nbr_odr",
    "Exact_obj", "Exact_mip_gap (%)", "Exact_status", "Exact_time (s)",
]

# Standard per-method detail columns (same sheet layout for every method).
# The method's extra_cols are appended after these.
_DETAIL_BASE = [
    "instance_file", "nbr_req", "nbr_cdr", "nbr_odr",
    "best_primal", "gap_vs_exact (%)", "bound", "total_time (s)", "status",
]

def _detail_header(method):
    return _DETAIL_BASE + [label for label, _ in method["extra_cols"]]

def _comparison_header(methods):
    cols = ["instance_file", "nbr_req", "nbr_cdr", "nbr_odr"]
    # ── objectives / primals ──
    cols.append("Exact_obj")
    cols += [f"{m['name']}_primal" for m in methods]
    # ── gap vs exact ──
    cols.append("Exact_mip_gap (%)")
    cols += [f"{m['name']}_gap_vs_exact (%)" for m in methods]
    # ── upper / Lagrangian bounds ──
    cols += [f"{m['name']}_bound" for m in methods]
    # ── wall-clock times ──
    cols.append("Exact_time (s)")
    cols += [f"{m['name']}_time (s)" for m in methods]
    # ── statuses ──
    cols.append("Exact_status")
    cols += [f"{m['name']}_status" for m in methods]
    # ── method-specific extras (grouped by method) ──
    for m in methods:
        cols += [f"{m['name']}_{label}" for label, _ in m["extra_cols"]]
    return cols


# ── Row builders ──────────────────────────────────────────────────────────────
def _detail_row(method, res, nbr_req, nbr_cdr, nbr_odr, instance_file, exact_obj):
    row = [
        instance_file, nbr_req, nbr_cdr, nbr_odr,
        _safe(res.get(method["key_primal"])),
        _gap_vs_exact(res.get(method["key_primal"]), exact_obj),
        _safe(res.get(method["key_bound"])),
        _safe(res.get(method["key_time"]), 2),
        res.get(method["key_status"]),
    ]
    for _, rkey in method["extra_cols"]:
        row.append(_safe(res.get(rkey)))
    return row


def _comparison_row(methods, results, res_exact, nbr_req, nbr_cdr, nbr_odr, instance_file):
    exact_obj = res_exact.get("obj")
    row = [instance_file, nbr_req, nbr_cdr, nbr_odr]
    # ── objectives / primals ──
    row.append(_safe(exact_obj))
    for m in methods:
        row.append(_safe(results[m["name"]].get(m["key_primal"])))
    # ── gap vs exact ──
    row.append(_safe(res_exact.get("mip_gap")))
    for m in methods:
        row.append(_gap_vs_exact(results[m["name"]].get(m["key_primal"]), exact_obj))
    # ── upper / Lagrangian bounds ──
    for m in methods:
        row.append(_safe(results[m["name"]].get(m["key_bound"])))
    # ── wall-clock times ──
    row.append(_safe(res_exact.get("time"), 2))
    for m in methods:
        row.append(_safe(results[m["name"]].get(m["key_time"]), 2))
    # ── statuses ──
    row.append(res_exact.get("status"))
    for m in methods:
        row.append(results[m["name"]].get(m["key_status"]))
    # ── method-specific extras (grouped by method) ──
    for m in methods:
        for _, rkey in m["extra_cols"]:
            row.append(_safe(results[m["name"]].get(rkey)))
    return row


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    script_dir = _HERE
    out_xlsx   = os.path.join(script_dir, "lagrangian_comparison_full_results.xlsx")

    wb          = Workbook()
    hdr_fill    = PatternFill(start_color="366092", end_color="366092", fill_type="solid")
    hdr_font    = Font(bold=True, color="FFFFFF")

    def _style(ws):
        for cell in ws[1]:
            cell.fill = hdr_fill
            cell.font = hdr_font

    # Build workbook sheets
    ws_exact = wb.active
    ws_exact.title = "Exact"
    ws_exact.append(EXACT_HEADER)
    _style(ws_exact)

    method_sheets = {}
    for m in METHODS:
        ws = wb.create_sheet(m["name"])
        ws.append(_detail_header(m))
        _style(ws)
        method_sheets[m["name"]] = ws

    ws_comp = wb.create_sheet("Comparison")
    ws_comp.append(_comparison_header(METHODS))
    _style(ws_comp)

    # Build instance list
    instances = []
    nbr_req = 10
    while nbr_req <= MAX_REQ:
        for nbr_cdr in range(0, 12, 2):
            nbr_odr = 10 - nbr_cdr
            instance_file = (
                f"InstanceswithDemand/Instance_REQ{nbr_req}_CDR{nbr_cdr}"
                f"_ODR{nbr_odr}_FS0_SS{SEED}.txt"
            )
            instances.append((nbr_req, nbr_cdr, nbr_odr, instance_file))
        nbr_req += 10

    total = len(instances)
    method_names = ", ".join(m["name"] for m in METHODS)
    print(f"Full comparison: Exact + [{method_names}]")
    print(f"Instances: {total}  (REQ 10–{MAX_REQ}, CDR ∈ {{0,2,4,6,8,10}})")
    print(f"Output: {out_xlsx}")
    wb.save(out_xlsx)

    all_sheets = [ws_exact] + list(method_sheets.values()) + [ws_comp]

    for idx, (nbr_req, nbr_cdr, nbr_odr, instance_file) in enumerate(instances, start=1):
        print(f"\n{'='*70}")
        print(f"[{idx}/{total}]  REQ={nbr_req}  CDR={nbr_cdr}  ODR={nbr_odr}")
        print(f"  {instance_file}")
        print(f"{'='*70}")

        if not os.path.exists(instance_file):
            print("  [SKIP] file not found")
            ws_exact.append([instance_file, nbr_req, nbr_cdr, nbr_odr] + [None] * 4)
            skip = [instance_file, nbr_req, nbr_cdr, nbr_odr] + [None] * (len(_DETAIL_BASE) - 4) + ["FILE NOT FOUND"]
            for m in METHODS:
                method_sheets[m["name"]].append(
                    skip + [None] * len(m["extra_cols"])
                )
            ws_comp.append(
                [instance_file, nbr_req, nbr_cdr, nbr_odr] +
                [None] * (len(_comparison_header(METHODS)) - 4)
            )
            for ws in all_sheets:
                _autosize(ws)
            wb.save(out_xlsx)
            continue

        # ── Exact MIP ────────────────────────────────────────────────────────
        print("\n--- Exact MIP ---")
        try:
            res_exact = solve_exact_mip(instance_file, time_limit=EXACT_TIME_LIMIT, verbose=False)
        except Exception as exc:
            print(f"  [ERROR] {exc}")
            res_exact = {"obj": None, "mip_gap": None, "status": f"ERROR: {exc}", "time": 0.0}
        ws_exact.append([
            instance_file, nbr_req, nbr_cdr, nbr_odr,
            _safe(res_exact.get("obj")),
            _safe(res_exact.get("mip_gap")),
            res_exact.get("status"),
            _safe(res_exact.get("time"), 2),
        ])
        print(f"  obj={res_exact.get('obj')}  gap={res_exact.get('mip_gap')}%  "
              f"time={res_exact.get('time', 0.0):.1f}s  status={res_exact.get('status')}")

        # ── Configured methods ────────────────────────────────────────────────
        results = {}
        for m in METHODS:
            print(f"\n--- {m['name']} ---")
            try:
                res = m["func"](instance_file, **m["kwargs"])
            except Exception as exc:
                print(f"  [ERROR] {exc}")
                res = {m["key_status"]: f"ERROR: {exc}"}
            results[m["name"]] = res
            method_sheets[m["name"]].append(
                _detail_row(m, res, nbr_req, nbr_cdr, nbr_odr,
                            instance_file, res_exact.get("obj"))
            )
            print(f"  primal={res.get(m['key_primal'])}  "
                  f"bound={res.get(m['key_bound'])}  "
                  f"time={res.get(m['key_time'], 0.0):.1f}s  "
                  f"status={res.get(m['key_status'])}")

        # ── Comparison row ────────────────────────────────────────────────────
        ws_comp.append(
            _comparison_row(METHODS, results, res_exact, nbr_req, nbr_cdr, nbr_odr, instance_file)
        )

        for ws in all_sheets:
            _autosize(ws)
        wb.save(out_xlsx)
        print(f"\n  [SAVED] {out_xlsx}")

    print(f"\n{'='*70}")
    print(f"Done. Results in: {out_xlsx}")
    print(f"{'='*70}")


if __name__ == "__main__":
    main()
