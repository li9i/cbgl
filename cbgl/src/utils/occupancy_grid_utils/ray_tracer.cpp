#include <algorithm>
#include <boost/bind.hpp>
#include <boost/ref.hpp>
#include <boost/foreach.hpp>
#include <boost/optional.hpp>
#include <ros/assert.h>
#include <tf/transform_datatypes.h>
#include <utils/occupancy_grid_utils/ray_tracer.h>
#include <utils/occupancy_grid_utils/exceptions.h>

namespace occupancy_grid_utils
{

namespace nm=nav_msgs;
namespace gm=geometry_msgs;
namespace sm=sensor_msgs;

using boost::bind;
using boost::ref;
using boost::optional;

/*******************************************************************************
 */
RayTraceIterRange rayTrace (const Cell& c1, const Cell& c2)
{
  return RayTraceIterRange(
    RayTraceIterator(c1,c2), RayTraceIterator(c2,c2,true));
}


/*******************************************************************************
 */
inline
bool cellWithinBounds (const nm::MapMetaData& info, const Cell& c)
{
  return withinBounds(info, c);
}


/*******************************************************************************
 */
optional<Cell> rayTraceOntoGrid (const nm::MapMetaData& info, const Cell& c1,
  const Cell& c2)
{
  RayTraceIterRange r = rayTrace(c2, c1);
  RayTraceIterator pos =
    std::find_if (r.first, r.second, bind(cellWithinBounds, boost::ref(info), _1));

  optional<Cell> c;

  if (pos!=r.second)
    c = *pos;

  return c;
}


/*******************************************************************************
 */
RayTraceIterRange rayTrace (const nm::MapMetaData& info, const gm::Point& p1,
  const gm::Point& p2, bool project_target_onto_grid,
  bool project_source_onto_grid, float max_range)
{

  gm::Point np2 = p2;
  if (max_range > 0) {
    double distance = euclideanDistance(p1,p2);
    if (distance > max_range) {
      np2.x = ((p2.x - p1.x) * max_range/distance) + p1.x;
      np2.y = ((p2.y - p1.y) * max_range/distance) + p1.y;
    }
  }

  Cell c1 = pointCell(info, p1);
  Cell c2 = pointCell(info, np2);
  ROS_DEBUG_STREAM_NAMED ("ray_trace", "Ray tracing between " << c1.x <<
                          ", " << c1.y << " and " << c2.x << ", " << c2.y);
  const RayTraceIterator done(c1, c1, true);
  const RayTraceIterRange empty_range(done, done);
  if (!withinBounds(info, c1))
  {
    if (project_source_onto_grid)
    {
      const optional<Cell> c = rayTraceOntoGrid(info, c2, c1);
      if (c)
        c1 = *c;
      else
        return empty_range;
    }
    else
      throw PointOutOfBoundsException(p1);
  }

  if (!withinBounds(info, np2))
  {
    if (project_target_onto_grid)
    {
      const optional<Cell> c = rayTraceOntoGrid(info, c1, c2);
      if (c)
        c2 = *c;
      else
        return empty_range;
    }
    else
    {
      throw PointOutOfBoundsException(np2);
    }
  }

  ROS_DEBUG_STREAM_NAMED ("ray_trace", "Projected ray trace endpoints to " <<
    c1.x << ", " << c1.y << " and " << c2.x << ", " << c2.y);

  return rayTrace(c1, c2);
}


/*******************************************************************************
 */
// Singleton class to generate points at max range of each ray
// of a scan from a specified sensor pose
// Caches the points and as long as the scan properties are
// the same, only a transformation to the new sensor pose is applied
class ScanEndPoints
{
private:
  ScanEndPoints()
    : angle_min_(0.0)
    , angle_max_(0.0)
    , range_max_(0.0)
    , angle_increment_(0.0)
  {}
  ScanEndPoints(const ScanEndPoints&);
  void operator=(const ScanEndPoints&);

public:
  typedef std::vector<gm::Point>::const_iterator PointIter;
  typedef std::pair<PointIter, PointIter> PointRange;

  static ScanEndPoints& getInstance()
  {
    static ScanEndPoints instance;
    return instance;
  }

  PointRange operator()(const sm::LaserScan& scanner_info,
    const gm::Pose& sensor_pose)
  {
    validate(scanner_info);
    transform(sensor_pose);
    return std::make_pair(points_.begin(), points_.end());
  }

private:
  void validate(const sm::LaserScan& info)
  {
    if (angle_min_ == info.angle_min && angle_max_ == info.angle_max &&
       range_max_ == info.range_max && angle_increment_ == info.angle_increment)
    {
      return;
    }
    else createPoints(info);
  }

  void transform(const gm::Pose& sensor_pose)
  {
    tf::Transform pose;
    tf::poseMsgToTF(sensor_pose, pose);

    for (size_t i = 0; i < tf_points_.size(); ++i)
    {
      tf::pointTFToMsg(pose * tf_points_[i], points_[i]);
    }
  }

  void createPoints(const sm::LaserScan& info)
  {
    angle_min_ = info.angle_min;
    angle_max_ = info.angle_max;
    range_max_ = info.range_max;
    angle_increment_ = info.angle_increment;

    const unsigned int size =
      round(1 + (angle_max_ - angle_min_)/angle_increment_);

    tf_points_.clear();
    tf_points_.reserve(size);
    points_.resize(size);

    double angle = angle_min_;

    for (size_t i = 0; i < size; ++i, angle += angle_increment_)
    {
      const double cos_angle = cos(angle);
      const double sin_angle = sin(angle);

      tf_points_.push_back(
        tf::Point(cos_angle * range_max_, sin_angle * range_max_, 0));
    }
  }

  std::vector<gm::Point> points_;
  std::vector<tf::Point> tf_points_;
  double angle_min_;
  double angle_max_;
  double range_max_;
  double angle_increment_;
};


/*******************************************************************************
 */
sm::LaserScan::Ptr simulateRangeScan (const nm::OccupancyGrid& grid,
  const gm::Pose& sensor_pose, const sm::LaserScan& scanner_info,
  const bool unknown_obstacles)
{
  sm::LaserScan::Ptr result(new sm::LaserScan(scanner_info));
  result->ranges.clear();
  const gm::Point& p0 = sensor_pose.position;
  const Cell c0 = pointCell(grid.info, p0);

  // For each ray ...
  BOOST_FOREACH (
    const gm::Point& scan_max, ScanEndPoints::getInstance()(scanner_info, sensor_pose))
  {
    // ... ray trace
    result->ranges.push_back(scanner_info.range_max); // Default if loop terminates
    BOOST_FOREACH (const Cell& c, rayTrace(grid.info, p0, scan_max, true, true))
    {
      const gm::Point p = cellCenter(grid.info, c);
      const double d = sqrt((p.x-p0.x)*(p.x-p0.x) + (p.y-p0.y)*(p.y-p0.y));
      char data = grid.data[cellIndex(grid.info, c)];
      if (d > scanner_info.range_max)
      {
        break;
      }
      else if (data == OCCUPIED && !(c == c0))
      {
        result->ranges.back() = d;
        break;
      }
      else if (data == UNKNOWN && !(c == c0))
      {
        result->ranges.back() = unknown_obstacles ? d : scanner_info.range_max;
        break;
      }
    }
  }

  return result;
}


} // namespace occupancy_grid_utils
