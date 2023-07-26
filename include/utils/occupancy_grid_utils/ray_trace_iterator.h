#ifndef OCCUPANCY_GRID_UTILS_RAY_TRACE_ITERATOR_H
#define OCCUPANCY_GRID_UTILS_RAY_TRACE_ITERATOR_H

#include <utils/occupancy_grid_utils/coordinate_conversions.h>
#include <ros/assert.h>
#include <boost/iterator/iterator_facade.hpp>

namespace occupancy_grid_utils
{

inline
int8_t sign(const coord_t x)
{
  return (x>0) ? 1 : -1;
}

class RayTraceIterator
  : public boost::iterator_facade<RayTraceIterator, Cell,
                                  boost::forward_traversal_tag, const Cell&>
{
public:

  RayTraceIterator();
  RayTraceIterator (const Cell& c1, const Cell& c2, bool done=false) :
    cell_(c1), goal_(c2), done_(done)
  {
    const coord_t dx = c2.x-c1.x;
    const coord_t dy = c2.y-c1.y;
    const coord_t abs_dx = abs(dx);
    const coord_t abs_dy = abs(dy);
    const int8_t offset_dx = sign(dx);
    const int8_t offset_dy = sign(dy);

    if (abs_dx>abs_dy) {
      x_inc_ = offset_dx;
      y_inc_ = 0;
      x_correction_ = 0;
      y_correction_ = offset_dy;
      error_ = abs_dx/2;
      error_inc_ = abs_dy;
      error_threshold_ = abs_dx;
    }
    else {
      x_inc_ = 0;
      y_inc_ = offset_dy;
      x_correction_ = offset_dx;
      y_correction_ = 0;
      error_ = abs_dy/2;
      error_inc_ = abs_dx;
      error_threshold_ = abs_dy;
    }

    ROS_DEBUG_NAMED ("ray_trace_iter", "Setting up ray trace iterator from %d, %d to %d, %d",
                     c1.x, c1.y, c2.x, c2.y);
    ROS_DEBUG_NAMED ("ray_trace_iter", " x_inc=%d, y_inc=%d, x_correction=%d, y_correction=%d",
                     x_inc_, y_inc_, x_correction_, y_correction_);
    ROS_DEBUG_NAMED ("ray_trace_iter", " error=%u, error_inc=%u, error_threshold=%u",
                     error_, error_inc_, error_threshold_);
  }

private:

  /****************************************
   * Iterator facade ops
   ****************************************/

  friend class boost::iterator_core_access;

  void increment ()
  {
    ROS_ASSERT_MSG (!done_, "Can't increment ray trace iterator that's already reached the end");
    if (cell_ == goal_) {
      done_=true;
    }
    else {
      cell_.x += x_inc_;
      cell_.y += y_inc_;
      error_ += error_inc_;
      if (error_ >= error_threshold_) {
        cell_.x += x_correction_;
        cell_.y += y_correction_;
        error_ -= error_threshold_;
      }
      ROS_DEBUG_NAMED ("ray_trace_iter", "cell is %d, %d", cell_.x, cell_.y);
    }
  }

  const Cell& dereference () const
  {
    ROS_ASSERT_MSG (!done_, "Can't dereference ray trace iterator that has reached the end");
    return cell_;
  }

  bool equal (const RayTraceIterator& other) const
  {
    // incomplete, but this should only be called when other relevant state is equal
    return (this->done_ && other.done_) || (!this->done_ && !other.done_ && this->cell_ == other.cell_);
  }


  /****************************************
   * Internal state
   ****************************************/

  coord_t x_inc_, y_inc_, x_correction_, y_correction_;
  coord_t error_, error_inc_, error_threshold_;
  Cell cell_, goal_;
  bool done_;
};





} // namespace occupancy_grid_utils

#endif // include guard
