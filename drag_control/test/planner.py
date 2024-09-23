import numpy as np
import matplotlib.pyplot as plt
import collision

from dataclasses import dataclass
from corgipath.collision import BoundingVolumeHierarchy
from corgipath.planning import HybridAstar
from corgipath.search_space import DefaultHybridGrid, DefaultHybridNode
from corgipath.matplot import static_draw as draw
from corgipath.matplot import live_draw as live
from corgipath.matplot.utils import pick_color, auto_scale
from drag_control_server import Drag_Control
from scipy.interpolate import CubicSpline
from matplotlib.patches import Ellipse
from typing import Tuple, Dict, List

class StableTopContactPushServer:
    def __init__(self, world_config, drag_config):
        self.drag_control = Drag_Control(drag_config)
        self.hybrid_planner = HybridAstarDragPlanner(world_config)
        self.local_planner = DWALocalPlanner()

    def plot_trajectory(self, trajectory):
        
        fig, ax = plt.subplots()
        ax.set_xlim(self.hybrid_planner.world_bounds[0], self.hybrid_planner.world_bounds[1])  
        ax.set_ylim(self.hybrid_planner.world_bounds[2], self.hybrid_planner.world_bounds[3])  

        line, = ax.plot([], [], 'b-', lw=1.0)  
        ax.grid(True)
        
        ellipse = Ellipse((0, 0), width=self.drag_control.minor_axis, height=self.drag_control.major_axis, angle=0, color='r', fill=False)
        ax.add_patch(ellipse)
        
        x_data = []
        y_data = []

        for frame in range(len(trajectory_data)):
            current_x, current_y, current_angle = trajectory_data[frame]
            x_data.append(current_x)
            y_data.append(current_y)
            line.set_data(x_data, y_data)

            ellipse.set_center((current_x, current_y))
            ellipse.angle = np.rad2deg(current_angle)  

            plt.draw()
            plt.pause(0.05)  

        plt.show()

class HybridAstarDragPlanner:
    def __init__(self, config=None):

        default_config = {
            'world_bounds': (-1.0, 1.0, -1.0, 1.0),
            'start': (0.0, 0.0, np.radians(90)),
            'goal': (0.2, 0.4, np.radians(0)),
            'grid_size': 0.02,
            'theta_gap': np.radians(5),
            'max_heading_change': np.radians(10)
        }
        if config:
            default_config.update(config)

        self.world_bounds   = default_config['world_bounds']
        self.start          = default_config['start']
        self.goal           = default_config['goal']
        self.grid_size      = default_config['grid_size']
        self.theta_gap      = default_config['theta_gap']
        self.max_heading_change      = default_config['max_heading_change']

        self.planner = HybridAstar()
        self.planner.collision_system = self.sample_bvh()
        self.planner.collision_system.build()
        self.planner.search_space = self.sample_grid(grid_size=self.grid_size, theta_gap=self.theta_gap)
        self.planner.search_space.reset()

    def sample_bvh(self):
        bvh = BoundingVolumeHierarchy(bounds=self.world_bounds)
        bvh.agent_collision = collision.Poly.from_box(collision.Vector(0.0, 0.0), 0.1, 0.2)
        return bvh
    
    def sample_grid(self, grid_size, theta_gap):
        grid = DefaultHybridGrid(dxy=grid_size, dtheta=theta_gap, node_type=DefaultHybridNode)

        # You can use the default successor template. Or you can customize it.
        grid.use_default_successor_template(max_heading_change=self.max_heading_change, allow_backward=False)
        return grid
            
    def live_draw_options(self, ax):
        styles = {
            "focus_current_node": {
                "color": "r",
                "fill": False,
                "coordinates_type": "directional circle",
                "coordinates_size": 0.05,
            },
            "open_list": {
                "color": "gold",
                "fill": False,
                "coordinates_type": "directional circle",
                "coordinates_size": 0.05,
            },
            "path_reconstruction": {
                "color": "b",
                "fill": True,
                "coordinates_type": "directional circle",
                "coordinates_size": 0.05,
            },
        }
        live_draw_options = {
            "focus_current_node": live.LiveDrawOption(
                draw_func=lambda xyt: draw.draw_coordinates(ax, xyt, style=styles["focus_current_node"]),
                # pause_after=0.05,
                # wait_key=True,
            ),
            "open_list": live.LiveDrawOption(
                draw_func=lambda xyt: draw.draw_coordinates(ax, xyt, style=styles["open_list"]),
            ),
            "path_reconstruction": live.LiveDrawOption(
                draw_func=lambda xyt: draw.draw_coordinates(ax, xyt, style=styles["path_reconstruction"]),
                pause_before=0.1,
            ),
        }
        return live_draw_options
    
    def plan(self):
        fig, ax = plt.subplots()

        # Draw background objects (environment-related information)
        draw.draw_grid(ax, grid=self.planner.search_space, drawing_bounds=self.world_bounds, style={"color": "0.8", "linewidth": 0.5})

        # Draw background objects (agent-related objects)
        agent_shape = self.planner.collision_system.agent_collision
        draw.draw_shape(ax, agent_shape, at=self.start, style={"color": pick_color(0.7, "turbo"), "fill": True}) 
        draw.draw_shape(ax, agent_shape, at=self.goal,  style={"color": pick_color(0.7, "rainbow"), "fill": True}) 
        
        auto_scale(ax)

        self.planner.set_live_draw_options(self.live_draw_options(ax))
        waypoints = self.planner.solve(self.start, self.goal)
        print(waypoints)
        
        interpolated_path =self.interpolate_path(waypoints, waypoints_num=30)

        color = list(pick_color(0.3, "rainbow"))
        color[3] = 0.8  # Set alpha
        draw.draw_waypoints(
            ax,
            waypoints,
            agent_shape,
            show_shape=True,
            shape_style={"color": color, "fill": False},
            show_coordinates=False,
        )

        # # Wait for closing the plot
        plt.pause(2)

        return interpolated_path

    @staticmethod
    def interpolate_path(path, waypoints_num=30):
        path = np.array(path, dtype=np.float32)

        # Extract x, y, and theta columns from the motion path
        x = path[:, 0]
        y = path[:, 1]
        theta = path[:, 2]

        # Create an array of indices corresponding to the original motion path points
        N = len(path)
        indices = np.arange(N)

        # Create a new array of indices for the interpolated points
        new_indices = np.linspace(0, N-1, waypoints_num)

        # Perform cubic spline interpolation on x, y, and theta separately
        cs_x = CubicSpline(indices, x)
        cs_y = CubicSpline(indices, y)
        cs_theta = CubicSpline(indices, theta)

        # Evaluate the cubic splines at the new indices
        interpolated_x = cs_x(new_indices)
        interpolated_y = cs_y(new_indices)
        interpolated_theta = cs_theta(new_indices)

        # Combine the interpolated x, y, and theta into a new motion path
        interpolated_path = np.column_stack((interpolated_x, interpolated_y, interpolated_theta))

        return interpolated_path

class DWALocalPlanner:
    def __init__(self, config=None):

        default_config = {
            'delta_t': 0.1,
            'terminal_distance': 0.005
        }

        if config:
            default_config.update(config)
        
        self.delta_t = default_config['delta_t']
        self.terminal_distance = default_config['terminal_distance']

    def select_velocity(self, waypoint, object_pose, velocity_candidate):
        
        waypoint = np.array(waypoint)
        if np.abs(waypoint[2] - object_pose[2]) < np.deg2rad(1):
            filtered_velocity_candidate = velocity_candidate[velocity_candidate[:, 2] == 0]
            for velocity in filtered_velocity_candidate:
                waypoint        
        else:
            filtered_velocity_candidate = velocity_candidate

    def _terminal_condition(self, waypoint, object_pose):
        waypoint = np.array(waypoint, dtype=np.float32)
        object_pose = np.array(object_pose, dtype=np.float32)

        if np.linalg.norm(waypoint - object_pose) < self.terminal_distance:
            return True
        else:
            return False
    

if __name__ == '__main__':
    
    ##############  SIMULATION EXPERIMENT PROCESS  ##############
    # SET GRID WORLD FOR SIMULATION 
    world_bounds = (-1.0, 1.0, -0.5, 1.0)
    start = (0.0, 0.0, np.radians(90))
    goal = (0.2, 0.4, np.radians(30))

    world_config = {'world_bounds': world_bounds, 'start': start, 'goal': goal, \
                    'theta_gap': np.radians(5), 'max_heading_change': np.radians(20)}

    # SET FRICTION COEFFICIENT AND OTHER VALUES FOR SETTING
    drag_config = {'delta_t': 0.1}

    # CALCULATE FEASIBLE VELOCITY USING DRAG CONTROL ALGORITHM
    drag_server = StableTopContactPushServer(world_config, drag_config)
    trajectory_data = drag_server.hybrid_planner.plan()

    drag_server.plot_trajectory(trajectory_data)

    # MAKE TRAJECTORY OF DISHES USING HYBRID A STAR ALGORITHM
        # hybrid astar algorithm will be made by other github repository 
    
    # STORE TRAJECTORY OF DISHES AND VISUALIZE IT

    #############################################################


