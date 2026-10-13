# bnp_pricing_comparison.py
#
# Compares Branch-and-Price pricing solvers (and optionally the exact MIP) on the
# InstanceswithDemand sweep: REQ = 10, 20, ..., MAX_REQ and CDR ∈ {0, 2, ..., 10},
# ODR = 10 − CDR.  Methods are defined in the METHODS registry below — sheets,
# headers, and comparison columns are generated from it.
#
# Output: bnp_pricing_comparison_{cgall|cgfirst}_{modes}.xlsx, where {modes} lists the
# MIP-heuristic settings run, e.g. "mip", "nomip" or "mip_nomip".
#   Sheet "Exact"        — per-instance exact MIP results (only if RUN_EXACT)
#   Sheet "<MethodName>" — one detail sheet per configured method
#   Sheet "Comparison"   — all methods side-by-side per instance, plus pairwise columns:
#                          every method vs the first (reference) method, and every method
#                          vs its own "baseline" (e.g. BnP_GS_nomip vs BnP_GS_mip)

import os
import sys

from openpyxl import Workbook
from openpyxl.utils import get_column_letter
from openpyxl.styles import Font, PatternFill

_HERE      = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.dirname(_HERE)
sys.path.insert(0, _REPO_ROOT)   # solve_exact_mip, instance_loader
sys.path.insert(0, _HERE)        # branch_and_price and its pricing solvers

from branch_and_price import solve_instance_bnp
from solve_exact_mip import solve_exact_mip


# ── Tunable parameters ────────────────────────────────────────────────────────
MAX_REQ           = 100              # largest REQ size to include (10, 20, ..., MAX_REQ)
SEED              = 0               # SS index used when building instance filenames
RUN_EXACT         = False            # False: skip the exact MIP (two-way pricing comparison)
EXACT_TIME_LIMIT  = 3600            # seconds for the exact MIP
TIME_LIMIT        = 3600            # seconds per B&P run
MIP_TIME_PER_NODE = 20.0            # seconds for the RMP MIP at each node
MIP_HEURISTIC_MODES = [True, False]  # per-node MIP heuristic settings to run, each as its own
                                     # methods: [True] / [False] / [True, False] (side by side)
ADD_ALL_COLUMNS   = True            # False: first positive-rc column per round; True: all
# ─────────────────────────────────────────────────────────────────────────────

_BNP_KWARGS = {
    "time_limit":        TIME_LIMIT,
    "mip_time_per_node": MIP_TIME_PER_NODE,
    "add_all_columns":   ADD_ALL_COLUMNS,
    "verbose":           False,
}

_BNP_EXTRA_COLS = [
    ("best_ub",            "best_ub"),
    ("root_gap (%)",       "root_gap"),
    ("final_gap (%)",      "best_gap"),
    ("gap_improvement (%)", "gap_improvement"),
    ("nodes",              "n_nodes"),
    ("pruned",             "n_pruned"),
    ("lp_infeasible",      "n_lp_infeasible"),
    ("mip_improvements",   "n_mip_improvements"),
    ("cols_root",          "n_cols_root"),
    ("cols_total",         "n_cols_total"),
    ("cg_iters_root",      "n_cg_iters_root"),
    ("cg_iters_total",     "n_cg_iters_total"),
    ("cg_time (s)",        "cg_time"),
    ("pricing_time (s)",   "pricing_time"),
    ("pricing_calls",      "n_pricing_calls"),
    ("bound_checks",       "n_bound_checks"),
    ("bound_pruned",       "n_bound_pruned"),
    ("requests_eligible",  "n_requests_eligible"),
    ("requests_excluded",  "n_requests_excluded"),
    ("mip_time (s)",       "mip_time"),
]


# ── Method registry ───────────────────────────────────────────────────────────
# The first entry is the reference for the "vs reference" comparison columns.
#
# Required keys per entry:
#   name             — label used for the sheet name and all column prefixes
#   func             — callable(instance_path, **kwargs) → dict
#   kwargs           — passed verbatim to func on every call
#   key_primal       — result-dict key for the best primal objective value
#   key_bound        — result-dict key for the bound compared across methods (or None)
#   key_time         — result-dict key for total wall-clock time
#   key_time_to_best — result-dict key for the time at which the final best solution was found
#   key_status       — result-dict key for the status / message string
#   key_pricing_time — result-dict key for pricing time (or None)
#   extra_cols       — list of (column_label, result_dict_key) for method-specific stats
# Optional:
#   baseline         — name of another method to compare this one against directly

_PRICING_SOLVERS = [("GS", "graphsearch"), ("Label", "labeling")]


def _mode_tag(use_mip):
    return "mip" if use_mip else "nomip"


def _bnp_method(solver_tag, pricing_algo, use_mip):
    method = {
        "name":             f"BnP_{solver_tag}_{_mode_tag(use_mip)}",
        "func":             solve_instance_bnp,
        "kwargs":           {**_BNP_KWARGS, "pricing_algo": pricing_algo,
                             "use_mip_heuristic": use_mip},
        "key_primal":       "best_primal",
        "key_bound":        "lp_root_bound",
        "key_time":         "total_time",
        "key_time_to_best": "time_to_best",
        "key_status":       "status_msg",
        "key_pricing_time": "pricing_time",
        "extra_cols":       _BNP_EXTRA_COLS,
    }
    # Without the heuristic, compare against the same pricing solver with it.
    if not use_mip and True in MIP_HEURISTIC_MODES:
        method["baseline"] = f"BnP_{solver_tag}_{_mode_tag(True)}"
    return method


METHODS = [_bnp_method(tag, algo, use_mip)
           for tag, algo in _PRICING_SOLVERS
           for use_mip in MIP_HEURISTIC_MODES]
# ─────────────────────────────────────────────────────────────────────────────


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


def _safe(v, decimals=4):
    if v is None:
        return None
    try:
        return round(float(v), decimals)
    except (TypeError, ValueError):
        return v


def _gap_vs_exact(primal, exact_obj):
    if primal is None or exact_obj is None or abs(exact_obj) < 1e-9:
        return None
    return round(100.0 * (exact_obj - primal) / abs(exact_obj), 4)


def _diff(a, b):
    if a is None or b is None:
        return None
    return round(a - b, 6)


def _ratio(ref, other):
    if ref is None or other is None or other < 1e-9:
        return None
    return round(ref / other, 3)


# ── Header builders ───────────────────────────────────────────────────────────
_INSTANCE_COLS = ["instance_file", "nbr_req", "nbr_cdr", "nbr_odr"]

EXACT_HEADER = _INSTANCE_COLS + [
    "Exact_obj", "Exact_lp_relax_ub", "Exact_mip_gap (%)", "Exact_time (s)",
    "Exact_time_to_best (s)", "Exact_status",
]

_DETAIL_BASE = _INSTANCE_COLS + [
    "best_primal", "gap_vs_exact (%)", "bound", "total_time (s)", "time_to_best (s)", "status",
]


def _detail_header(method):
    return _DETAIL_BASE + [label for label, _ in method["extra_cols"]]


def _comparison_pairs(methods):
    """(method, baseline) pairs for the pairwise columns: every method vs the first one,
    then every method vs its own declared baseline, without duplicates."""
    pairs = [(m, methods[0]) for m in methods[1:]]
    by_name = {m["name"]: m for m in methods}
    for m in methods:
        base = by_name.get(m.get("baseline"))
        if base is not None and (m, base) not in pairs:
            pairs.append((m, base))
    return pairs


def _comparison_header(methods, run_exact):
    cols = list(_INSTANCE_COLS)
    # ── objectives / primals ──
    if run_exact:
        cols.append("Exact_obj")
    cols += [f"{m['name']}_primal" for m in methods]
    # ── gap vs exact ──
    if run_exact:
        cols.append("Exact_mip_gap (%)")
        cols += [f"{m['name']}_gap_vs_exact (%)" for m in methods]
    # ── bounds ──
    if run_exact:
        cols.append("Exact_lp_relax_ub")
    cols += [f"{m['name']}_bound" for m in methods]
    # ── wall-clock times ──
    if run_exact:
        cols.append("Exact_time (s)")
    cols += [f"{m['name']}_time (s)" for m in methods]
    # ── time until the final best solution was found ──
    if run_exact:
        cols.append("Exact_time_to_best (s)")
    cols += [f"{m['name']}_time_to_best (s)" for m in methods]
    # ── statuses ──
    if run_exact:
        cols.append("Exact_status")
    cols += [f"{m['name']}_status" for m in methods]
    # ── pairwise: method vs reference, method vs its own baseline ──
    for m, base in _comparison_pairs(methods):
        a, b = m["name"], base["name"]
        cols += [f"{a}_bound_diff_vs_{b}",
                 f"{a}_primal_diff_vs_{b}",
                 f"{a}_time_speedup_vs_{b}",
                 f"{a}_time_to_best_speedup_vs_{b}"]
        if m.get("key_pricing_time") and base.get("key_pricing_time"):
            cols.append(f"{a}_pricing_speedup_vs_{b}")
    # ── method-specific extras (grouped by method) ──
    for m in methods:
        cols += [f"{m['name']}_{label}" for label, _ in m["extra_cols"]]
    return cols


# ── Row builders ──────────────────────────────────────────────────────────────
def _detail_row(method, res, instance_cols, exact_obj):
    row = list(instance_cols) + [
        _safe(res.get(method["key_primal"])),
        _gap_vs_exact(res.get(method["key_primal"]), exact_obj),
        _safe(res.get(method["key_bound"])) if method["key_bound"] else None,
        _safe(res.get(method["key_time"]), 2),
        _safe(res.get(method.get("key_time_to_best")), 2) if method.get("key_time_to_best") else None,
        res.get(method["key_status"]),
    ]
    for _, rkey in method["extra_cols"]:
        row.append(_safe(res.get(rkey)))
    return row


def _comparison_row(methods, results, res_exact, instance_cols, run_exact):
    def get(m, key_field):
        key = m.get(key_field)
        return results[m["name"]].get(key) if key else None

    exact_obj = res_exact.get("obj") if run_exact else None
    row = list(instance_cols)
    # ── objectives / primals ──
    if run_exact:
        row.append(_safe(exact_obj))
    row += [_safe(get(m, "key_primal")) for m in methods]
    # ── gap vs exact ──
    if run_exact:
        row.append(_safe(res_exact.get("mip_gap")))
        row += [_gap_vs_exact(get(m, "key_primal"), exact_obj) for m in methods]
    # ── bounds ──
    if run_exact:
        row.append(_safe(res_exact.get("lp_relax_ub")))
    row += [_safe(get(m, "key_bound")) for m in methods]
    # ── wall-clock times ──
    if run_exact:
        row.append(_safe(res_exact.get("time"), 2))
    row += [_safe(get(m, "key_time"), 2) for m in methods]
    # ── time until the final best solution was found ──
    if run_exact:
        row.append(_safe(res_exact.get("time_to_best"), 2))
    row += [_safe(get(m, "key_time_to_best"), 2) for m in methods]
    # ── statuses ──
    if run_exact:
        row.append(res_exact.get("status"))
    row += [get(m, "key_status") for m in methods]
    # ── pairwise: method vs reference, method vs its own baseline ──
    for m, base in _comparison_pairs(methods):
        row += [_diff(get(m, "key_bound"),  get(base, "key_bound")),
                _diff(get(m, "key_primal"), get(base, "key_primal")),
                _ratio(get(base, "key_time"), get(m, "key_time")),
                _ratio(get(base, "key_time_to_best"), get(m, "key_time_to_best"))]
        if m.get("key_pricing_time") and base.get("key_pricing_time"):
            row.append(_ratio(get(base, "key_pricing_time"), get(m, "key_pricing_time")))
    # ── method-specific extras (grouped by method) ──
    for m in methods:
        for _, rkey in m["extra_cols"]:
            row.append(_safe(results[m["name"]].get(rkey)))
    return row


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    cg_tag   = "cgall" if ADD_ALL_COLUMNS else "cgfirst"
    mip_tag  = "_".join(_mode_tag(use_mip) for use_mip in MIP_HEURISTIC_MODES)
    out_xlsx = os.path.join(_HERE, f"bnp_pricing_comparison_{cg_tag}_{mip_tag}.xlsx")

    wb       = Workbook()
    hdr_fill = PatternFill(start_color="366092", end_color="366092", fill_type="solid")
    hdr_font = Font(bold=True, color="FFFFFF")

    def _style(ws):
        for cell in ws[1]:
            cell.fill = hdr_fill
            cell.font = hdr_font

    ws_exact = None
    first_ws = wb.active
    if RUN_EXACT:
        ws_exact = first_ws
        ws_exact.title = "Exact"
        ws_exact.append(EXACT_HEADER)
        _style(ws_exact)

    method_sheets = {}
    for idx, m in enumerate(METHODS):
        if idx == 0 and not RUN_EXACT:
            ws = first_ws
            ws.title = m["name"]
        else:
            ws = wb.create_sheet(m["name"])
        ws.append(_detail_header(m))
        _style(ws)
        method_sheets[m["name"]] = ws

    comp_header = _comparison_header(METHODS, RUN_EXACT)
    ws_comp = wb.create_sheet("Comparison")
    ws_comp.append(comp_header)
    _style(ws_comp)

    all_sheets = ([ws_exact] if ws_exact else []) + list(method_sheets.values()) + [ws_comp]

    instances = []
    for nbr_req in range(10, MAX_REQ + 1, 10):
        for nbr_cdr in range(0, 12, 2):
            nbr_odr = 10 - nbr_cdr
            instance_file = (f"InstanceswithDemand/Instance_REQ{nbr_req}_CDR{nbr_cdr}"
                             f"_ODR{nbr_odr}_FS0_SS{SEED}.txt")
            instances.append((nbr_req, nbr_cdr, nbr_odr, instance_file))

    total = len(instances)
    method_names = ", ".join(m["name"] for m in METHODS)
    print(f"B&P comparison: {'Exact + ' if RUN_EXACT else ''}[{method_names}]")
    print(f"  Instances : {total}  (REQ 10 – {MAX_REQ}, CDR ∈ {{0,2,4,6,8,10}})")
    print(f"  Output    : {out_xlsx}")
    wb.save(out_xlsx)

    for idx, (nbr_req, nbr_cdr, nbr_odr, instance_file) in enumerate(instances, start=1):
        instance_cols = [instance_file, nbr_req, nbr_cdr, nbr_odr]
        instance_path = os.path.join(_REPO_ROOT, instance_file)
        print(f"\n{'='*70}")
        print(f"[{idx}/{total}]  REQ={nbr_req}  CDR={nbr_cdr}  ODR={nbr_odr}")
        print(f"  {instance_file}")
        print(f"{'='*70}")

        if not os.path.exists(instance_path):
            print("  [SKIP] file not found")
            if ws_exact:
                ws_exact.append(instance_cols + [None] * (len(EXACT_HEADER) - 5) + ["FILE NOT FOUND"])
            for m in METHODS:
                hdr = _detail_header(m)
                row = instance_cols + [None] * (len(hdr) - 4)
                row[hdr.index("status")] = "FILE NOT FOUND"
                method_sheets[m["name"]].append(row)
            ws_comp.append(instance_cols + [None] * (len(comp_header) - 4))
            for ws in all_sheets:
                _autosize(ws)
            wb.save(out_xlsx)
            continue

        # ── Exact MIP ────────────────────────────────────────────────────────
        res_exact = {}
        if RUN_EXACT:
            print("\n--- Exact MIP ---")
            try:
                res_exact = solve_exact_mip(instance_path, time_limit=EXACT_TIME_LIMIT, verbose=False)
            except Exception as exc:
                print(f"  [ERROR] {exc}")
                res_exact = {"status": f"ERROR: {exc}"}
            ws_exact.append(instance_cols + [
                _safe(res_exact.get("obj")),
                _safe(res_exact.get("lp_relax_ub")),
                _safe(res_exact.get("mip_gap")),
                _safe(res_exact.get("time"), 2),
                _safe(res_exact.get("time_to_best"), 2),
                res_exact.get("status"),
            ])
            print(f"  obj={res_exact.get('obj')}  gap={res_exact.get('mip_gap')}%  "
                  f"time={res_exact.get('time') or 0.0:.1f}s  status={res_exact.get('status')}")

        # ── Configured methods ────────────────────────────────────────────────
        results = {}
        for m in METHODS:
            print(f"\n--- {m['name']} ---")
            try:
                res = m["func"](instance_path, **m["kwargs"])
            except Exception as exc:
                print(f"  [ERROR] {exc}")
                res = {m["key_status"]: f"ERROR: {exc}"}
            results[m["name"]] = res
            method_sheets[m["name"]].append(
                _detail_row(m, res, instance_cols, res_exact.get("obj"))
            )
            pricing = (f"  pricing={res.get(m['key_pricing_time']) or 0.0:.2f}s"
                       if m.get("key_pricing_time") else "")
            print(f"  primal={res.get(m['key_primal'])}  "
                  f"bound={res.get(m['key_bound']) if m['key_bound'] else None}  "
                  f"time={res.get(m['key_time']) or 0.0:.1f}s{pricing}  "
                  f"status={res.get(m['key_status'])}")

        # ── Comparison row ────────────────────────────────────────────────────
        ws_comp.append(_comparison_row(METHODS, results, res_exact, instance_cols, RUN_EXACT))

        for ws in all_sheets:
            _autosize(ws)
        wb.save(out_xlsx)
        print(f"\n  [SAVED] {out_xlsx}")

    print(f"\n{'='*70}")
    print(f"Done. Results in: {out_xlsx}")
    print(f"{'='*70}")


if __name__ == "__main__":
    main()
