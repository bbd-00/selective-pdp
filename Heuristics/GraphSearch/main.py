# main.py
from instance_generator import generate_random_instance
from dfs_solver import DFSHeuristic
from gurobi_solver import solve_with_gurobi
import time
import traceback

def show_solution(inst, routes, obj):
    print(f"\nBest objective value: {obj:.2f}", flush=True)
    for k, r in enumerate(routes, 1):
        if r:
            seq = " -> ".join(map(str, r))
            print(f"Vehicle {k}: [{inst.depot_start}] -> {seq} -> [{inst.depot_end}]", flush=True)
        else:
            print(f"Vehicle {k}: [{inst.depot_start}] -> [{inst.depot_end}] (idle)", flush=True)

if __name__ == "__main__":
    try:
        import random
        N_OPTIONS = [5, 10, 15]
        K_OPTIONS = [1, 2, 3]
        REV_OPTIONS = ["low", "high"]
        TW_OPTIONS = ["regular", "large"]

        for trial in range(1, 21):
            n_requests = random.choice(N_OPTIONS)
            n_vehicles = random.choice(K_OPTIONS)
            revenue_level = random.choice(REV_OPTIONS)
            tw_size = random.choice(TW_OPTIONS)
            seed = random.randrange(1000000)

            print(f"\n=== Trial {trial} === "
                  f"(n={n_requests}, K={n_vehicles}, rev={revenue_level}, tw={tw_size}, seed={seed})")

            inst = generate_random_instance(
                n_requests=n_requests,
                n_vehicles=n_vehicles,
                revenue_level=revenue_level,
                tw_size=tw_size,
                seed=seed,
            )

            # --- DFS heuristic ---
            h0 = time.time()
            routes_h, obj_h = DFSHeuristic(inst).solve()
            h_time = time.time() - h0

            # --- Gurobi ---
            routes_g, obj_g, g_time = solve_with_gurobi(inst, verbose=False)

            # --- Compare ---
            diff = obj_h - obj_g
            if abs(diff) <= 1e-4:
                status = "MATCH"
            else:
                status = "MISMATCH"
            print(f"Heuristic: {obj_h:.2f} ({h_time:.2f}s), "
                  f"Gurobi: {obj_g:.2f} ({g_time:.2f}s) -> {status}, Δ={diff:.6f}")

        print("\nAll trials done.")

    except Exception:
        print("An error occurred:\n", flush=True)
        traceback.print_exc()



    # try:
    #     # --- build instance ---
    #     inst = generate_random_instance(
    #         n_requests=50,
    #         n_vehicles=1,
    #         revenue_level="high",
    #         tw_size="large",
    #         seed=420,
    #     )
    #     print("Instance generated.", flush=True)

    #     # --- DFS heuristic ---
    #     print("\n ********** GRAPHSEARCH HEURISTIC ***********")
    #     solver = DFSHeuristic(inst)
    #     print("Solver created. Solving…", flush=True)

    #     h0 = time.time()
    #     routes, obj = solver.solve()
    #     h_time = time.time() - h0

    #     print(f"Heuristic objective: {obj:.2f}", flush=True)
    #     print(f"Heuristic runtime: {h_time:.2f}s", flush=True)
    #     show_solution(inst, routes, obj)

    #     # --- Gurobi MILP comparison ---
    #     print("\n ********** GUROBI ***********")
    #     try:
    #         print("\nRunning Gurobi MILP...", flush=True)
    #         gr_routes, gr_obj, gr_time = solve_with_gurobi(inst, verbose=True)
    #         print(f'Gurobi objective: {gr_obj:.2f}', flush=True)
    #         print(f'Gurobi runtime: {gr_time:.2f}s', flush=True)
    #         show_solution(inst, gr_routes, gr_obj)
    #     except Exception as e:
    #         print(f"Gurobi not run / failed: {e}", flush=True)

    #     print("Done.", flush=True)

    # except Exception:
    #     print("An error occurred:\n", flush=True)
    #     traceback.print_exc()