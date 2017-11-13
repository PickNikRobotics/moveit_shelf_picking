# MoveIt! Shelf Picking Benchmark

Benchmarking suite for dual arm manipulation

## Usage

Start Rviz:

    roslaunch moveit_shelf_picking baxter_visualize.launch

Run benchmark:

    roslaunch moveit_shelf_picking run_benchmark.launch

If you are prompted to press next to continue, find the "MoveItDashboard" panel in Rviz and click Next.

## Configuration

There are lots of settings that can easily be tweaked in the following file:

    moveit_shelf_picking/config/config_baxter.yaml

In particular, pay attention to the ``visualize/`` configurations for more indepth view of what is going on.
