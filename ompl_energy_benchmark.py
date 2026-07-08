import pyRAPL
import pandas as pd
import trimesh
import os
import numpy as np
from ompl import base as ob
from ompl import geometric as og

# 1. Initialize pyRAPL
pyRAPL.setup()

# Proximity-Based Mesh Collision Checker
class MeshValidityChecker(ob.StateValidityChecker):
    def __init__(self, si, dae_filename, safety_margin=0.5):
        super(MeshValidityChecker, self).__init__(si)
        loaded = trimesh.load(dae_filename)
        if isinstance(loaded, trimesh.Scene):
            self.mesh = loaded.to_mesh()
        else:
            self.mesh = loaded
        self.safety_margin = safety_margin
       
    def isValid(self, state):
        point = [state[0], state[1], state[2]]
        _, distance, _ = self.mesh.nearest.on_surface([point])
        if distance[0] < self.safety_margin:
            return False
        return True

def run_energy_profile(planner, pdef, time_limit=10.0):
    """Wraps the OMPL solve function in a hardware energy measurement block."""
    meter = pyRAPL.Measurement('UAV_Planner')
    meter.begin()
    solved = planner.solve(time_limit)
    meter.end()
   
    energy_joules = meter.result.pkg[0] / 1e6 if (meter.result and meter.result.pkg) else 0.0
    return solved, energy_joules, meter.result.duration

def find_valid_close_point(space, checker, target_coords):
    """If a coordinate is trapped in a wall, search nearby until finding open air."""
    state = space.allocState()
    state[0], state[1], state[2] = target_coords[0], target_coords[1], target_coords[2]
   
    if checker.isValid(state):
        return state
       
    for nudge in range(1, 50):
        for dx in [-5.0, 0.0, 5.0]:
            for dy in [-5.0, 0.0, 5.0]:
                for dz in [-2.0, 0.0, 2.0]:
                    state[0] = target_coords[0] + (dx * nudge)
                    state[1] = target_coords[1] + (dy * nudge)
                    state[2] = target_coords[2] + (dz * nudge)
                    if checker.isValid(state):
                        return state
    return state

def main():
    # Tailored configurations testing a full spectrum of checking qualities per map
    environment_setups = {
        "cubicles_env.dae": {
            "start": [-300.0, 10.0, 10.0],
            "goal": [100.0, 10.0, 10.0],  
            "bounds_low": [-520.0, -120.0, 5.0],
            "bounds_high": [330.0, 130.0, 15.0],
            "resolutions": {"Coarse": 0.05, "Standard": 0.01, "Fine": 0.001}
        },
        "Apartment_env.dae": {
            "start": [0.0, -45.0, 0.0],
            "goal": [150.0, -45.0, 10.0],
            "bounds_low": [-80.0, -100.0, -10.0],
            "bounds_high": [300.0, 10.0, 180.0],
            "resolutions": {"Coarse": 0.05, "Standard": 0.01, "Fine": 0.002}
        },
        "Twistycool_env.dae": {
            "start": [50.0, 150.0, 20.0],
            "goal": [400.0, 420.0, 20.0],
            "bounds_low": [10.0, 70.0, 15.0],  
            "bounds_high": [460.0, 510.0, 25.0],
            "resolutions": {"Coarse": 0.02, "Standard": 0.005, "Fine": 0.0005}
        }
    }
   
    # Using 2 trials per layout variation to keep execution time within reasonable limits
    trials = 2    
    results = []

    print("Starting Resolution Parametric Benchmark Loop...")

    for env_file, config in environment_setups.items():
        if not os.path.exists(env_file):
            print(f"Skipping {env_file} (File not found)")
            continue
           
        print(f"\n--- LOADING ENVIRONMENT: {env_file} ---")
       
        # Nested Loop: Iterate through each resolution quality tier
        for q_name, res_val in config["resolutions"].items():
            print(f"   -> Activating Resolution Quality: {q_name} ({res_val})")
           
            space = ob.RealVectorStateSpace(3)
            bounds = ob.RealVectorBounds(3)
           
            for i in range(3):
                bounds.setLow(i, config["bounds_low"][i])
                bounds.setHigh(i, config["bounds_high"][i])
            space.setBounds(bounds)
           
            si = ob.SpaceInformation(space)
            validity_checker = MeshValidityChecker(si, env_file)
            si.setStateValidityChecker(validity_checker)
           
            # Dynamically push the resolution quality parameter into OMPL
            si.setStateValidityCheckingResolution(res_val)
            si.setup()

            start = find_valid_close_point(space, validity_checker, config["start"])
            goal = find_valid_close_point(space, validity_checker, config["goal"])

            for planner_class, p_name in [(og.PRM, "PRM"), (og.RRTstar, "RRTstar"), (og.RRTConnect, "RRTConnect")]:
                for trial in range(trials):
                    pdef = ob.ProblemDefinition(si)
                    pdef.setStartAndGoalStates(start, goal)
                   
                    planner = planner_class(si)
                    planner.setProblemDefinition(pdef)
                    planner.setup()
                   
                    print(f"      [{env_file}] {p_name} | Quality: {q_name} | Trial: {trial+1}/{trials}")
                   
                    solved, energy, duration = run_energy_profile(planner, pdef)
                   
                    # Log data with the explicit parameter quality designations
                    results.append({
                        "Environment": env_file,
                        "Resolution_Quality": q_name,
                        "Resolution_Value": res_val,
                        "Planner": p_name,
                        "Trial": trial + 1,
                        "Solved": solved.asString(),
                        "Energy_Joules": energy,
                        "Time_Seconds": duration / 1e6
                    })
                    planner.clear()

    # Save to your CSV file
    df = pd.DataFrame(results)
    df.to_csv("multi_env_energy_trends.csv", index=False)
    print("\nBenchmark Complete! Parametric dataset saved to multi_env_energy_trends.csv")

if __name__ == "__main__":
    main()
