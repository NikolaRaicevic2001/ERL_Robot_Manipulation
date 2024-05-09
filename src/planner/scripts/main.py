import argparse
from problems import visualizer_2D as viz2D
from problems import narrow_path_2D
from problems import dodging_cones_2D

from planners import *

def main(problem_type, planner):
    if problem_type == 'narrow_path_2D':
        environment = narrow_path_2D.Narrow_Path()
        print(planner)
        # planner = MazePlanner(environment)
    elif problem_type == 'dodging_cones_2D':
        environment = dodging_cones_2D.Dodging_Cones()
        print(planner)
        # planner = ObstaclePlanner(environment)
    else:
        print("Unknown problem type specified")
        return

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run the planning environment")
    parser.add_argument('problem_type', type=str, help='Type of problem to solve ("narrow_path_2D", "dodging_cones_2D")')
    parser.add_argument('planner_type', type=str, help='Type of planner to use ("rrt*", "rrt")')
    args = parser.parse_args()

    main(args.problem_type, args.planner_type)
