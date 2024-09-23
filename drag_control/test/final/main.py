import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import collision

from dataclasses import dataclass
from corgipath.collision import BoundingVolumeHierarchy
from corgipath.planning import HybridAstar
from corgipath.search_space import DefaultHybridGrid, DefaultHybridNode, HybridSuccessor
from corgipath.matplot import static_draw as draw
from corgipath.matplot import live_draw as live
from corgipath.matplot.utils import pick_color, auto_scale
from drag_control_server import Drag_Control
from hybrid_astar_server import HybridAstarDragPlanner

from typing import Tuple, Dict

class StableTopContactPushServer(object):

    def __init__(self, world_config, drag_config):
        # this will be code for ros communication and other setting
        pass

class HybridAstarDragPlanner(object):
    def __init__(self, drag_config=None, world_config=None):
        """Initialize setting for HybridAstarDragPlanner

        Args:
            drag_config  (dict): parameter for drag config
            world_config (dict): parameter for world config
        """
        self.drag_config = drag_config
        self.drag_server = Drag_Control(drag_config)

        default_world_config = {
            'world_bounds': (-1.0, 1.0, -1.0, 1.0),
            'start': (0.0, 0.0, np.radians(90)),
            'goal': (0.2, 0.4, np.radians(0)),
            'grid_size': 0.04,
            'max_heading_change': np.radians(15)
        }

        if world_config:
            default_world_config.update(world_config)

        self.world_bounds            = default_world_config['world_bounds']
        self.start                   = default_world_config['start']
        self.goal                    = default_world_config['goal']
        self.grid_size               = default_world_config['grid_size']
        self.max_heading_change      = default_world_config['max_heading_change']

        self._planner = HybridAstar()

    def plan(self):
        """ Plan stable drag path from a given drag config

        Returns:
            hand_path (List[Tuple[float, float, float]]): xy hand-path with exerted force F
            object_path (List[Tuple[float, float, float]]):
        """
    
        print('start pose: {:.2f} {:.2f} {:.2f}'.format(self.start[0], self.start[1], np.rad2deg(self.start[2])))
              
        self._planner.collision_system = self._get_collision_system()
        self._planner.collision_system.build()

        succesor_template = self._get_successor_template()
        search_space = self._get_search_space(succesor_template)

        self._planner.search_space = search_space
        self._planner.search_space.reset()


        # show animation plot 
        fig, ax = plt.subplots()

        # Draw background objects (environment-related information)
        draw.draw_grid(ax, grid=self._planner.search_space, drawing_bounds=self.world_bounds, style={"color": "0.8", "linewidth": 0.5})

        # Draw background objects (agent-related objects)
        agent_shape = self._planner.collision_system.agent_collision
        draw.draw_shape(ax, agent_shape, at=self.start, style={"color": pick_color(0.7, "turbo"), "fill": True}) 
        draw.draw_shape(ax, agent_shape, at=self.goal,  style={"color": pick_color(0.7, "rainbow"), "fill": True}) 
        
        auto_scale(ax)

        # self._planner.set_live_draw_options(self.live_draw_options(ax))

        waypoints = self._planner.solve(start, goal)
        print(waypoints)
        print(len(waypoints))

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
        plt.pause(20)

    def _get_collision_system(self):
        """Get the collision system

        Returns:
            bvh (BoundingVolumeHierarchy): Bounding volume hierarchy
        """
        bvh = BoundingVolumeHierarchy(bounds=self.world_bounds)
        bvh.agent_collision = collision.Poly.from_box(collision.Vector(0.0, 0.0), self.drag_server.major_axis, self.drag_server.minor_axis)
        return bvh
    
    def _get_successor_template(self):
        """Set the successor template to the default template.

        Returns:
            template (List[Tuple[float, float, float], float]): succesor template from current node
        """
        # In Hybrid A*, minimum distance to forward >= diagonal length.
        diagonal = np.sqrt(2.0) * 0.5 * self.drag_config['delta_t']

        # Pair of displacement and cost
        forward = (diagonal, diagonal)
        edges = (forward,)

        drag = self.drag_server.drag_velocity_candidate()

        # Make template
        template = []
        for displacement, cost in edges:
            _dist = displacement / np.sqrt(2.0)
            _cost = cost / np.sqrt(2.0)
            for q_o_dot in drag[2]:
                template.append(HybridSuccessor.from_velocity_with_time(velocity=q_o_dot*10, delta_t=self.drag_config['delta_t'], cost=_cost))

        return template

    def _get_search_space(self, successor_template):
        search_space = DefaultHybridGrid(
            dxy=0.5*self.drag_config['delta_t'],
            dtheta=np.deg2rad(5),
            node_type=DefaultHybridNode
        )
        search_space.successor_template = successor_template
        return search_space

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
            )
        }
        return live_draw_options
    
if __name__ == '__main__':
    ##############  SIMULATION EXPERIMENT PROCESS  ##############
    # SET GRID WORLD FOR SIMULATION 
    world_bounds = (-1.0, 1.0, -0.5, 1.0)
    start = (0.0, 0.0, np.radians(90))
    goal = (-0.6, -0.2, np.radians(0))

    world_config = {
                'world_bounds': world_bounds,
                'start': start, 
                'goal': goal, 
                'theta_gap': np.radians(5), 
                'max_heading_change': np.radians(20)
            }

    # SET FRICTION COEFFICIENT AND OTHER VALUES FOR SETTING
    drag_config = {
            'start_t': 0.0,
            'delta_t': 0.1,
            'finish_t': 30.0
        }

    # CALCULATE FEASIBLE VELOCITY USING DRAG CONTROL ALGORITHM
    server = HybridAstarDragPlanner(drag_config, world_config)
    server.plan()

    # MAKE TRAJECTORY OF DISHES USING HYBRID A STAR ALGORITHM
        # hybrid astar algorithm will be made by other github repository 
    
    # STORE TRAJECTORY OF DISHES AND VISUALIZE IT


    #############################################################

    
    # ##############  SIMULATION EXPERIMENT PROCESS  ##############
    # # SET GRID WORLD FOR SIMULATION 
    # world_bounds = (-2.0, 2.0, -2.0, 2.0)
    # start = (-0.5, -1.5, np.radians(90))
    # goal = (1.0, 0.0, np.radians(0))

    # world_config = {'world_bounds': world_bounds, 'start': start, 'goal': goal}

    # # SET FRICTION COEFFICIENT AND OTHER VALUES FOR SETTING
    # drag_config = {'delta_t': 0.1}

    # # CALCULATE FEASIBLE VELOCITY USING DRAG CONTROL ALGORITHM
    # drag_server = StableTopContactPushServer(world_config, drag_config)

    # # MAKE TRAJECTORY OF DISHES USING HYBRID A STAR ALGORITHM
    #     # hybrid astar algorithm will be made by other github repository 
    
    # # STORE TRAJECTORY OF DISHES AND VISUALIZE IT


    # #############################################################
    

    # ##############  REAL EXPERIMENT PROCESS  ##############
    # # SET WORLD BOUNDS USING OPTI-TRACK

    # # SET FRICTION COEFFICIENT USING PREVIOUS EXPERIMENT
    
    # # DETECT TARGET DISHES AND GET SHAPE OF DISHES
    #     # Refer to previous rviz visualization.. make shape of dishes using stl file

    # # SELECT DISH CONTACT CANDIDATE BASED ON SHAPE OF DISHES

    # # CALCULATE FEASIBLE VELOCITY USING DRAG CONTROL ALGORITHM

    # # MAKE TRAJECTORY OF DISHES USING HYBRID A STAR ALGORITHM
    #     # hybrid astar algorithm will be made by other github repository
        
    # # SEND TRAJECTORY OF END-EFFECTOR WITH FORCE TO FRANKA RESEARCH MANIPULATOR        
    #     # Control exerted force using equilibrium pose with pid control

    # # MOVE TO DISH CONTACT POINT
    #     # 

    # # EXERT TOP CONTACT FORCE

    # # END-EFFECTOR MOVE ALONG FOLLOWING TRAJECTORY

    # #############################################################