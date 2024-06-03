import argparse
from problems import visualizer_2D as viz2D
from problems import narrow_path_2D, dodging_cones_2D
from problems import dodging_cones_2D_SDF, ellipse_2D_SDF
from problems import dodging_cones_2D_Occupied_SDF

from planners import RRT

def main(problem_type, planner_type):
    if problem_type == 'narrow_path_2D':
        environment = narrow_path_2D.Narrow_Path()
    elif problem_type == 'dodging_cones_2D':
        environment = dodging_cones_2D.Dodging_Cones()
    elif problem_type == 'dodging_cones_2D_SDF':
        environment = dodging_cones_2D_SDF.Dodging_Cones_SDF()
    elif problem_type == 'ellipse_2D_SDF':
        environment = ellipse_2D_SDF.Dodging_Ellipse_SDF()
    elif problem_type == 'dodging_cones_occupied_SDF':
        environment = dodging_cones_2D_Occupied_SDF.Dodging_Cones_Occupied_SDF()
    else:
        print("Unknown problem type specified")
        return
    
    window_size,obstacles,start,goal = environment.get_environment()

    if planner_type == 'rrt_star':
        bounds = (0,window_size,0,window_size)
        planner = RRT.RRT_Star(start, goal,obstacles, bounds,goal_bias=0.2, step_size=15, max_iterations=3000)
        nodes = planner.get_nodes()
    elif planner_type == 'rrt':
        planner = RRT.RRT(start, goal,obstacles,goal_bias=0.05, step_size=20, max_iterations=3000)
        nodes = planner.get_nodes()
    else:
        print("Unknown planner type specified")
        return

    grid_size = 5
    env = viz2D.Environment(obstacles, start, goal,nodes, window_size, window_size, grid_size)
    app = viz2D.PlannerApp(window_size, window_size, env, grid_size)
    app.run()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run the planning environment")
    parser.add_argument('problem_type', type=str, help='Type of problem to solve ("narrow_path_2D", "dodging_cones_2D", "dodging_cones_2D_SDF","dodging_cones_occupied_SDF")')
    parser.add_argument('planner_type', type=str, help='Type of planner to use ("rrt_star", "rrt")')
    args = parser.parse_args()

    main(args.problem_type, args.planner_type)

