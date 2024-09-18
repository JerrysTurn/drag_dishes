import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import collision

from dataclasses import dataclass
from corgipath.collision import BoundingVolumeHierarchy
from corgipath.planning import HybridAstar
from corgipath.search_space import DefaultHybridGrid, DefaultHybridNode
from drag_control_server import Drag_Control

class StableTopContactPushServer:
    def __init__(self, world_config, drag_config):
        self.hybrid_planner = HybridAstarDragPlanner(world_config)
        self.drag_control = Drag_Control(drag_config)
        pass
    
@dataclass
class DragHybridNode(DefaultHybridGrid):
    pass

class HybridAstarDragPlanner():
    def __init__(self, config=None):

        default_config = {
            'world_bounds': (-2.0, 2.0, -2.0, 2.0),
            'start': (-0.5, -1.5, np.radians(90)),
            'goal': (1.0, 0.0, np.radians(0)),
            'grid_size': 0.4
        }
        if config:
            default_config.update(config)

        self.world_bounds   = default_config['world_bounds']
        self.start          = default_config['start']
        self.goal           = default_config['goal']
        self.grid_size      = default_config['grid_size']

        self.planner = HybridAstar()
        self.planner.collision_system = self.sample_bvh()
        self.planner.collision_system.build()
        # self.planner.search_space = 

    def sample_bvh(self):
        bvh = BoundingVolumeHierarchy(bounds=self.world_bounds)
        bvh.agent_collision = collision.Poly.from_box(collision.Vector(0.0, 0.0), 0.1, 0.2)

        return bvh
    
    def sample_grid(grid_size):
        grid = DefaultHybridGrid(dxy=grid_size, node_type=DefaultHybridNode)
    


if __name__ == '__main__':
    
    ##############  SIMULATION EXPERIMENT PROCESS  ##############
    # SET GRID WORLD FOR SIMULATION 
    world_bounds = (-2.0, 2.0, -2.0, 2.0)
    start = (-0.5, -1.5, np.radians(90))
    goal = (1.0, 0.0, np.radians(0))

    world_config = {'world_bounds': world_bounds, 'start': start, 'goal': goal}

    # SET FRICTION COEFFICIENT AND OTHER VALUES FOR SETTING
    drag_config = {'delta_t': 0.1}

    # CALCULATE FEASIBLE VELOCITY USING DRAG CONTROL ALGORITHM
    drag_server = StableTopContactPushServer(world_config, drag_config)

    # MAKE TRAJECTORY OF DISHES USING HYBRID A STAR ALGORITHM
        # hybrid astar algorithm will be made by other github repository 
    
    # STORE TRAJECTORY OF DISHES AND VISUALIZE IT


    #############################################################
    

    ##############  REAL EXPERIMENT PROCESS  ##############
    # SET WORLD BOUNDS USING OPTI-TRACK

    # SET FRICTION COEFFICIENT USING PREVIOUS EXPERIMENT
    
    # DETECT TARGET DISHES AND GET SHAPE OF DISHES
        # Refer to previous rviz visualization.. make shape of dishes using stl file

    # SELECT DISH CONTACT CANDIDATE BASED ON SHAPE OF DISHES

    # CALCULATE FEASIBLE VELOCITY USING DRAG CONTROL ALGORITHM

    # MAKE TRAJECTORY OF DISHES USING HYBRID A STAR ALGORITHM
        # hybrid astar algorithm will be made by other github repository
        
    # SEND TRAJECTORY OF END-EFFECTOR WITH FORCE TO FRANKA RESEARCH MANIPULATOR        
        # Control exerted force using equilibrium pose with pid control

    # MOVE TO DISH CONTACT POINT
        # 

    # EXERT TOP CONTACT FORCE

    # END-EFFECTOR MOVE ALONG FOLLOWING TRAJECTORY

    #############################################################
