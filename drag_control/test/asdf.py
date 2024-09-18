import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import collision

from dataclasses import dataclass
from corgipath.collision import BoundingVolumeHierarchy
from corgipath.planning import HybridAstar
from corgipath.search_space import DefaultHybridGrid, DefaultHybridNode

planner = HybridAstar()
map_corners = [0, 10, 0, 10]
collision_system = BoundingVolumeHierarchy(bounds=map_corners)
planner.collision_system = collision_system
planner.collision_system.build()
planner.search_space

import numpy as np
import matplotlib.pyplot as plt
import collision
from corgipath.collision import BoundingVolumeHierarchy
from corgipath.search_space import DefaultHybridGrid, DefaultHybridNode
from corgipath.planning import HybridAstar
from corgipath.matplot import static_draw as draw
from corgipath.matplot import live_draw as live
from corgipath.matplot.utils import pick_color, auto_scale


def sample_grid(grid_size, theta_gap):
    grid = DefaultHybridGrid(dxy=grid_size, dtheta=theta_gap, node_type=DefaultHybridNode)

    # You can use the default successor template. Or you can customize it.
    grid.use_default_successor_template(max_heading_change=np.radians(30), allow_backward=False)

    return grid


def live_draw_options(ax: plt.Axes):
    styles = {
        "focus_current_node": {
            "color": "r",
            "fill": False,
            "coordinates_type": "directional circle",
            "coordinates_size": 0.3,
        },
        "defocus_current_node": {
            "color": "lightgray",
            "fill": False,
            "linewidth": 2,
            "coordinates_type": "directional circle",
            "coordinates_size": 0.3,
        },
        "open_list": {
            "color": "gold",
            "fill": False,
            "coordinates_type": "directional circle",
            "coordinates_size": 0.3,
        },
        "path_reconstruction": {
            "color": "b",
            "fill": True,
            "coordinates_type": "directional circle",
            "coordinates_size": 0.5,
        },
    }
    live_draw_options = {
        "focus_current_node": live.LiveDrawOption(
            draw_func=lambda xyt: draw.draw_coordinates(ax, xyt, style=styles["focus_current_node"]),
            # pause_after=0.05,
            # wait_key=True,
        ),
        "defocus_current_node": live.LiveDrawOption(
            draw_func=lambda xyt: draw.draw_coordinates(ax, xyt, style=styles["defocus_current_node"]),
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


def main(unit_test=False):
    world_bounds = (-5.0, 5.0, -5.0, 5.0)

    # Mission
    start = (-2.5, -3.5, np.radians(90))
    goal = (3.0, 0.0, np.radians(0))

    # Trivial but tricky mission because of grid_size
    # start = (0.0, 0.0, np.radians(0))
    # goal = (0.7, 0.0, np.radians(0))

    # Configure planner
    # -----------------
    # Configuration for planning is not required every time.
    # A one-time setup is sufficient, but reconfiguration can be done at any point.
    # For more information, see another example: ``examples/hybrid_astar_basic.py``

    planner = HybridAstar()
    planner.collision_system = sample_bvh(world_bounds)
    planner.collision_system.build()
    planner.search_space = sample_grid(grid_size=0.5, theta_gap=np.radians(10))
    planner.search_space.reset()
    
    # Draw background objects
    # -----------------------
    fig, ax = plt.subplots()

    # Draw background objects (environment-related information)
    draw.draw_grid(ax, grid=planner.search_space, drawing_bounds=world_bounds, style={"color": "0.8", "linewidth": 0.5})
    draw.draw_collsion_objects(ax, planner.collision_system.obstacles, {"color": "0.5", "fill": True})

    # Draw background objects (agent-related objects)
    agent_shape = planner.collision_system.agent_collision
    draw.draw_shape(ax, agent_shape, at=start, style={"color": pick_color(0.7, "turbo"), "fill": True})
    draw.draw_shape(ax, agent_shape, at=goal, style={"color": pick_color(0.7, "rainbow"), "fill": True})
    draw.draw_coordinates(ax, start, style={"color": "0", "head_width": 0.1, "head_length": 0.15, "coordinates_size": 0.3})
    draw.draw_coordinates(ax, goal, style={"color": "0", "head_width": 0.1, "head_length": 0.15, "coordinates_size": 0.3})

    # Adjust figure
    auto_scale(ax)

    # Compute path with live animation
    # --------------------------------
    # This is the only part that is different from the static version.

    planner.set_live_draw_options(live_draw_options(ax))

    waypoints = planner.solve(start, goal)
    print(waypoints)
    print(len(waypoints))

    # Result
    # ------

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

    # Wait for closing the plot
    plt.pause(0)

    # -------------------------
    # Ignore the following code when running the script as a standalone.
    # It is solely for unit testing purposes.
    if unit_test:
        from unittest import common_results

        return common_results(planner, waypoints)


if __name__ == "__main__":
    main()