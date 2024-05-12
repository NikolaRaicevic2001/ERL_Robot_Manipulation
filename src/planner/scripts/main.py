import argparse
from problems import visualizer_2D as viz2D
from problems import narrow_path_2D, dodging_cones_2D

from planners import RRT

def main(problem_type, planner_type):
    if problem_type == 'narrow_path_2D':
        environment = narrow_path_2D.Narrow_Path()
    elif problem_type == 'dodging_cones_2D':
        environment = dodging_cones_2D.Dodging_Cones()
    else:
        print("Unknown problem type specified")
        return
    
    window_size,obstacles,start,goal = environment.get_environment()

    if planner_type == 'rrt_star':
        bounds = (1,1,1,1)
        planner = RRT.RRT_Star(start, goal,obstacles, bounds,goal_bias=0.1, step_size=0.1, max_iterations=1000)
        nodes = planner.get_nodes()
    elif planner_type == 'rrt':
        planner = RRT.RRT(start, goal,obstacles,goal_bias=0.05, step_size=20, max_iterations=1000)
        nodes = planner.get_nodes()
    else:
        print("Unknown planner type specified")
        return

    env = viz2D.Environment(obstacles, start, goal,nodes)
    app = viz2D.PlannerApp(window_size, window_size, env)
    app.run()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run the planning environment")
    parser.add_argument('problem_type', type=str, help='Type of problem to solve ("narrow_path_2D", "dodging_cones_2D")')
    parser.add_argument('planner_type', type=str, help='Type of planner to use ("rrt_star", "rrt")')
    args = parser.parse_args()

    main(args.problem_type, args.planner_type)

