# AgROS Paths

## Description
Part of the AgROS Suite, this package generates boustrophedon motion for use in
fields.

## Dependencies
* ROS (see package.xml)
* MatPlotLib
* Shapely

## Usage
### UTM Toolset
To use the path generation node, you must know the UTM zone/band that you'll be
working in (it will be added to your config file). This package provides a simple
tool to check this information, using the following command:

`rosrun agros_paths find_zone.py _lat:=<LAT> _lon:=<LON>`

### Path Generation
1. Modify or copy the example config file (`config/example.yaml`) to set your
desire geometric and geographic properties
2. Launch the node using `roslaunch agros_paths path.launch` and the the generator
will publish a nav_msgs/Path and a geometry_msgs/RouteNetwork message to be used
in other nodes.

### Path Following
Launch the follower node using `roslaunch agros_paths follower.launch`. It will wait and
use the **first path message** it receives and will focus on it. You may set the base
frame id and the acceptance threshold (how close you must be to satisfy "goal reached")
in the launch file

## Visualizing
When launching, if the visualize parameter is set to true, then a plot will appear
to visualize the AB line, boundary, headlands, and generated path. The generated path
will always be colored fuchsia. (Note: Because the AB line gets inflated, the plot may be
initially zoomed out)

## Known Issues

_There are some known issues with the algorithm that solutions are being found for._

(Generator)
* If the AB line has an azimuth of 0&deg; (i.e. the points true north) it can't always choose the right direction to propagate the offsets
* Right now it assumes that the inner headlands border and the AB line do not intersect. This may cause some issues, but you can always translate it away for the time being

(Follower)
* If the robot manages to skip a goal without getting close enough to it, the robot must go
back to reach the goal. It is planned to add a "look ahead" feature to deal with skips such
that if you are significantly close to one of the proceeding goals, it will jump to that
one