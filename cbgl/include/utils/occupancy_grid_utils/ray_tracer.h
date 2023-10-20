#ifndef OCCUPANCY_GRID_UTILS_RAY_TRACER_H
#define OCCUPANCY_GRID_UTILS_RAY_TRACER_H

#include <string>
#include <boost/optional.hpp>

#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/LaserScan.h>

#include <utils/occupancy_grid_utils/ray_trace_iterator.h>

namespace occupancy_grid_utils
{

/*******************************************************************************
 * Basic ray tracing
 */
typedef std::pair<RayTraceIterator, RayTraceIterator> RayTraceIterRange;

inline double euclideanDistance (const geometry_msgs::Point& p1,
  const geometry_msgs::Point& p2)
{
  const double dx=p1.x-p2.x;
  const double dy=p1.y-p2.y;
  const double dz=p1.z-p2.z;
  return sqrt(dx*dx + dy*dy + dz*dz);
}



/*******************************************************************************
\brief Returns an iterator range over the cells on the
line segment between two points (inclusive).

\param project_target_onto_grid If true, \a p2 may be off the grid,
in which case the ray stops right before falling off the grid
\param project_source_onto_grid If true, \a p1 may be off the grid,
in which case the ray starts at the point where it enters the grid
\param max_range The maximum range to raycast a point out to
\throws PointOutOfBoundsException if \a p1 is off grid
and project_source_onto_grid is false
\throws PointOutOfBoundsException if \a p2 is off grid
and project_target_onto_grid is false
\retval range models
<a href="http://www.boost.org/doc/libs/1_42_0/libs/range/doc/range.html#forward_range">Forward range</a>
with reference type const Cell&, i.e., it's a pair of iterators that you can use
in for loops, std::for_each, etc.
**/
RayTraceIterRange rayTrace (const nav_msgs::MapMetaData& info,
  const geometry_msgs::Point& p1, const geometry_msgs::Point& p2,
  bool project_target_onto_grid=false, bool project_source_onto_grid=false,
  float max_range = -1);



/*******************************************************************************
\brief Simulate a planar laser range scan

\param grid Occupancy grid
\param sensor_pose Assumed to lie on the grid and point along the grid
\param scanner_info Only the angle_{min,max,increment} and range_max
fields of this are used.
\param unknown_cells_are_obstacles If true, rays that hit unknown space
are considered to have hit an obstacle rather than being max-range
\retval Simulated laser range scan from the given pose on the grid
**/
sensor_msgs::LaserScan::Ptr simulateRangeScan(
  const nav_msgs::OccupancyGrid& grid, const geometry_msgs::Pose& sensor_pose,
  const sensor_msgs::LaserScan& scanner_info,
  bool unknown_cells_are_obstacles=false);

} // namespace occupancy_grid_utils

#endif // include guard
