#ifndef FSM_H
#define FSM_H

// --- INCLUDES ----------------------------------------------------------------
#include <memory>
#include <signal.h>
#include <cmath>
#include <numeric>
#include <iostream>
#include <fstream>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <functional>
#include <algorithm>
#include <vector>
#include <tuple>
#include <utility>
#include <math.h>
#include <chrono>
#include <assert.h>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/variate_generator.hpp>
#include <fftw3.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Filtered_kernel.h>
#include <CGAL/Cartesian.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/convex_hull_2.h>
#include <CGAL/minimum_enclosing_quadrilateral_2.h>
#include <CGAL/squared_distance_2.h>
#include <CGAL/Min_ellipse_2.h>
#include <CGAL/Min_ellipse_2_traits_2.h>
#include <eigen3/Eigen/Geometry>

// --- TYPEDEFs ----------------------------------------------------------------
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_2                       Point_2;
typedef Kernel::Vector_2                      Vector_2;
typedef CGAL::Polygon_2<Kernel>               Polygon_2;
typedef Polygon_2::Vertex_iterator            VertexIterator;
typedef CGAL::Min_ellipse_2_traits_2<Kernel>  Traits;
typedef CGAL::Min_ellipse_2<Traits>           Min_ellipse;


// --- FSM ---------------------------------------------------------------------
namespace FSM
{

struct input_params
{
  unsigned int num_iterations;
  double xy_bound;
  double t_bound;
  unsigned int max_counter;
  unsigned int min_magnification_size;
  unsigned int max_magnification_size;
  unsigned int max_recoveries;
};

struct output_params
{
  double exec_time;
  double rotation_times;
  double translation_times;
  double rotation_iterations;
  double translation_iterations;
  double intersections_times;
  unsigned int num_recoveries;
  std::vector< std::tuple<double,double,double> > trajectory;

  // Rotation criterion
  double rc;

  // Translation criterion
  double tc;

  output_params()
  {
    exec_time = 0;
    rotation_times = 0;
    translation_times = 0;
    rotation_iterations = 0;
    translation_iterations = 0;
    intersections_times = 0;
    num_recoveries = 0;
    rc = 0;
    tc = 0;
  };
};


// -----------------------------------------------------------------------------
class X
{
  public:

  /*****************************************************************************
  */
  static std::vector< std::pair<double,double> > find(
    const std::tuple<double,double,double>& pose,
    const std::vector< std::pair<double, double> >& lines,
    const unsigned int& num_rays)
  {
    return findExact2(pose, lines, num_rays);
  }

  /*****************************************************************************
  */
  static std::vector< std::pair<double,double> > findExact(
    const std::tuple<double,double,double>& pose,
    const std::vector< std::pair<double, double> >& lines,
    const unsigned int& num_rays)
  {
#ifdef TIMES
    std::chrono::high_resolution_clock::time_point a =
      std::chrono::high_resolution_clock::now();
#endif

    double px = std::get<0>(pose);
    double py = std::get<1>(pose);
    double pt = std::get<2>(pose);

    std::vector< std::pair<double,double> > intersections;
    double mul = 100000000.0;

    for (int i = 0; i < num_rays; i++)
    {
      double t_ray = i * 2*M_PI / num_rays + pt - M_PI;
      t_ray = fmod(t_ray + 5*M_PI, 2*M_PI) - M_PI;

      double x_far = px + mul*cos(t_ray);
      double y_far = py + mul*sin(t_ray);


      double tan_t_ray = tan(t_ray);
      bool tan_peligro = false;
      //if (fabs(fabs(t_ray) - M_PI/2) == 0.0)
      if (fabs(fabs(t_ray) - M_PI/2) < 0.0001)
        tan_peligro = true;


      std::vector< std::pair<double,double> > candidate_points;

      for (int l = 0; l < lines.size(); l++)
      {
        // The index of the first sensed point
        int idx_1 = l;

        // The index of the second sensed point (in counter-clockwise order)
        int idx_2 = idx_1 + 1;


        if (idx_2 >= lines.size())
          idx_2 = fmod(idx_2, lines.size());

        if (idx_1 >= lines.size())
          idx_1 = fmod(idx_1, lines.size());

        double det_1 =
          (lines[idx_1].first-px)*(lines[idx_2].second-py)-
          (lines[idx_2].first-px)*(lines[idx_1].second-py);

        double det_2 =
          (lines[idx_1].first-x_far)*(lines[idx_2].second-y_far)-
          (lines[idx_2].first-x_far)*(lines[idx_1].second-y_far);


        if (det_1 * det_2 <= 0.0)
        {
          double det_3 =
            (px-lines[idx_1].first)*(y_far-lines[idx_1].second)-
            (x_far-lines[idx_1].first)*(py-lines[idx_1].second);

          double det_4 =
            (px-lines[idx_2].first)*(y_far-lines[idx_2].second)-
            (x_far-lines[idx_2].first)*(py-lines[idx_2].second);

          if (det_3 * det_4 <= 0.0)
          {
            // They intersect!

            double x = 0.0;
            double y = 0.0;

            double ttp_x = lines[idx_2].first - lines[idx_1].first;
            double ttp_y = lines[idx_2].second - lines[idx_1].second;

            // The line segment is perpendicular to the x-axis
            if (ttp_x == 0.0)
            {
              // The ray is parallel to the x-axis
              if (x_far == px)
              {
                x = lines[idx_1].first;
                y = py;
              }
              else
              {
                x = lines[idx_1].first;
                y = y_far + (y_far - py)/(x_far - px) * (x - x_far);
              }
            }
            else
            {
              double tan_two_points = ttp_y / ttp_x;

              if (!tan_peligro)
              {
                x = (py - lines[idx_1].second + tan_two_points * lines[idx_1].first
                  -tan_t_ray * px) / (tan_two_points - tan_t_ray);

                y = py + tan_t_ray * (x - px);
              }
              else
              {
                x = px;
                y = lines[idx_1].second + tan_two_points * (x - lines[idx_1].first);
                //y = (lines[idx_2].second + lines[idx_1].second)/2;
              }
            }

            candidate_points.push_back(std::make_pair(x,y));
          }
        }
      }

      double min_r = 100000000.0;
      int idx = -1;
      for (int c = 0; c < candidate_points.size(); c++)
      {
        double dx = candidate_points[c].first - px;
        double dy = candidate_points[c].second - py;
        double r = dx*dx+dy*dy;

        if (r < min_r)
        {
          min_r = r;
          idx = c;
        }
      }

      assert(idx >= 0);

      intersections.push_back(
        std::make_pair(candidate_points[idx].first, candidate_points[idx].second));
    }

#ifdef TIMES
    std::chrono::high_resolution_clock::time_point b =
      std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> elapsed =
      std::chrono::duration_cast< std::chrono::duration<double> >(b-a);

    printf("%f [X::findExact]\n", elapsed.count());
#endif

    return intersections;
  }


  /*******************************************************************************
   */
  static std::vector< std::pair<double,double> > findExact2(
    const std::tuple<double,double,double>& pose,
    const std::vector< std::pair<double, double> >& lines,
    const unsigned int& num_rays)
  {
#ifdef TIMES
    std::chrono::high_resolution_clock::time_point a =
      std::chrono::high_resolution_clock::now();
#endif

    double px = std::get<0>(pose);
    double py = std::get<1>(pose);
    double pt = std::get<2>(pose);

    std::vector< std::pair<double,double> > intersections;
    double mul = 100000000.0;


    int start0 = 0;
    int end0 = lines.size();

    for (int i = 0; i < num_rays; i++)
    {
      double t_ray = i * 2*M_PI / num_rays + pt - M_PI;
      t_ray = fmod(t_ray + 5*M_PI, 2*M_PI) - M_PI;

      double x_far = px + mul*cos(t_ray);
      double y_far = py + mul*sin(t_ray);


      double tan_t_ray = tan(t_ray);
      bool tan_peligro = false;
      //if (fabs(fabs(t_ray) - M_PI/2) == 0.0)
      if (fabs(fabs(t_ray) - M_PI/2) < 0.0001)
        tan_peligro = true;


      std::pair<double,double> intersection_point;
      int segment_id;
      bool success = false;
      int inc = lines.size()/16;

      // Start off with the first ray. Scan the whole `lines` vector and find
      // the index of the segment the first ray hits. This index becomes the
      // starting index from which the second ray shall start searching. The last
      // segment the second ray shall end at is defined by `inc`. And do this
      // for all rays.
      // IF there is no intersection between [start, start+inc] then start from
      // start+inc and go up to start+2inc... and do this until you find a hit.
      while(!success)
      {
        success = findExactOneRay(px,py,tan_t_ray, x_far,y_far,lines,
          start0, end0, tan_peligro,
          &intersection_point, &segment_id);

        if (success)
          start0 = segment_id;
        else
          start0 += inc;

        end0 = start0 + inc;
      }

      intersections.push_back(intersection_point);
    }

#ifdef TIMES
    std::chrono::high_resolution_clock::time_point b =
      std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> elapsed =
      std::chrono::duration_cast< std::chrono::duration<double> >(b-a);

    printf("%f [X::findExact]\n", elapsed.count());
#endif

    return intersections;
  }


  /*****************************************************************************
   */
  static bool findExactOneRay(
    const double& px, const double& py, const double& tan_t_ray,
    const double& x_far, const double& y_far,
    const std::vector< std::pair<double, double> >& lines,
    const int& start_search_id, const int& end_search_id,
    const bool& tan_peligro,
    std::pair<double,double>* intersection_point,
    int* start_segment_id)
  {
    std::vector< std::pair<double,double> > candidate_points;
    std::vector<int> candidate_start_segment_ids;

    for (int l = start_search_id; l < end_search_id; l++)
    {
      // The index of the first sensed point
      int idx_1 = l;

      // The index of the second sensed point (in counter-clockwise order)
      int idx_2 = idx_1 + 1;

      if (idx_2 >= lines.size())
        idx_2 = fmod(idx_2, lines.size());

      if (idx_1 >= lines.size())
        idx_1 = fmod(idx_1, lines.size());

      double det_1 =
        (lines[idx_1].first-px)*(lines[idx_2].second-py)-
        (lines[idx_2].first-px)*(lines[idx_1].second-py);

      double det_2 =
        (lines[idx_1].first-x_far)*(lines[idx_2].second-y_far)-
        (lines[idx_2].first-x_far)*(lines[idx_1].second-y_far);

      if (det_1 * det_2 <= 0.0)
      {
        double det_3 =
          (px-lines[idx_1].first)*(y_far-lines[idx_1].second)-
          (x_far-lines[idx_1].first)*(py-lines[idx_1].second);

        double det_4 =
          (px-lines[idx_2].first)*(y_far-lines[idx_2].second)-
          (x_far-lines[idx_2].first)*(py-lines[idx_2].second);

        if (det_3 * det_4 <= 0.0)
        {
          // They intersect!

          double x = 0.0;
          double y = 0.0;

          double ttp_x = lines[idx_2].first - lines[idx_1].first;
          double ttp_y = lines[idx_2].second - lines[idx_1].second;

          // The line segment is perpendicular to the x-axis
          if (ttp_x == 0.0)
          {
            // The ray is parallel to the x-axis
            if (x_far == px)
            {
              x = lines[idx_1].first;
              y = py;
            }
            else
            {
              x = lines[idx_1].first;
              y = y_far + (y_far - py)/(x_far - px) * (x - x_far);
            }
          }
          else
          {
            double tan_two_points = ttp_y / ttp_x;

            if (!tan_peligro)
            {
              x = (py - lines[idx_1].second + tan_two_points * lines[idx_1].first
                -tan_t_ray * px) / (tan_two_points - tan_t_ray);

              y = py + tan_t_ray * (x - px);
            }
            else
            {
              x = px;
              y = lines[idx_1].second + tan_two_points * (x - lines[idx_1].first);
              //y = (lines[idx_2].second + lines[idx_1].second)/2;
            }
          }

          candidate_points.push_back(std::make_pair(x,y));
          candidate_start_segment_ids.push_back(idx_1);
        }
      }
    }

    double min_r = 100000000.0;
    int idx = -1;
    for (int c = 0; c < candidate_points.size(); c++)
    {
      double dx = candidate_points[c].first - px;
      double dy = candidate_points[c].second - py;
      double r = dx*dx+dy*dy;

      if (r < min_r)
      {
        min_r = r;
        idx = c;
      }
    }

    if (idx >= 0)
    {
      *intersection_point =
        std::make_pair(candidate_points[idx].first, candidate_points[idx].second);
      *start_segment_id = candidate_start_segment_ids[idx];

      return true;
    }
    else
      return false;
  }

};

// -----------------------------------------------------------------------------
class Utils
{
  public:


  /*****************************************************************************
  */
  static Eigen::Matrix3d
  computeTransform(const std::tuple<double,double,double>& d,
    const Eigen::Matrix3d& M)
  {
    double dx = std::get<0>(d);
    double dy = std::get<1>(d);
    double dt = std::get<2>(d);

    // Translation matrix
    Eigen::Matrix3d T;
    T = Eigen::Matrix3d::Identity();
    T(0,2) = dx;
    T(1,2) = dy;

    // Rotation matrix
    Eigen::Matrix3d R;
    R = Eigen::Matrix3d::Identity();
    R(0,0) = +cosf(dt);
    R(0,1) = -sin(dt);
    R(1,0) = +sin(dt);
    R(1,1) = +cosf(dt);

    // Compute the new transform matrix
    Eigen::Matrix3d M_;
    M_ = M * T * R;

    return M_;
  }

  /*****************************************************************************
  */
  static std::vector< std::pair<double, double> > conjugate(
    const std::vector< std::pair<double, double> >& vec)
  {
#ifdef TIMES
    std::chrono::high_resolution_clock::time_point start =
      std::chrono::high_resolution_clock::now();
#endif

    std::vector< std::pair<double,double> > ret_vector;
    for (int i = 0; i < vec.size(); i++)
      ret_vector.push_back(std::make_pair(vec[i].first, -vec[i].second));

#ifdef TIMES
    std::chrono::high_resolution_clock::time_point end =
      std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed =
      std::chrono::duration_cast< std::chrono::duration<double> >(end-start);

    printf("%f [conjugate]\n", elapsed.count());
#endif

    return ret_vector;
  }

  /*****************************************************************************
  */
  static void diffScansPerRay(
    const std::vector<double>& scan1, const std::vector<double>& scan2,
    const double& inclusion_bound, std::vector<double>* diff,
    std::vector<double>* diff_true)
  {
    assert (scan1.size() == scan2.size());

    diff->clear();
    diff_true->clear();

    double eps = 0.000001;
    if (inclusion_bound < 0.0001)
      eps = 1.0;

#ifdef DEBUG
    printf("inclusion_bound = %f\n", inclusion_bound + eps);
#endif

    double d = 0.0;
    for (unsigned int i = 0; i < scan1.size(); i++)
    {
      d = scan1[i] - scan2[i];

      if (fabs(d) <= inclusion_bound + eps)
        diff->push_back(d);
      else
        diff->push_back(0.0);

      diff_true->push_back(d);
    }
  }

  /*****************************************************************************
  */
  static void generatePose(
    const std::tuple<double,double,double>& real_pose,
    const double& dxy, const double& dt,
    std::tuple<double,double,double>* virtual_pose)
  {
    assert(dxy >= 0);
    assert(dt >= 0);

    std::random_device rand_dev;
    std::mt19937 generator_x(rand_dev());
    std::mt19937 generator_y(rand_dev());
    std::mt19937 generator_t(rand_dev());
    std::mt19937 generator_sign(rand_dev());

    std::uniform_real_distribution<double> distribution_x(-dxy, dxy);
    std::uniform_real_distribution<double> distribution_y(-dxy, dxy);
    std::uniform_real_distribution<double> distribution_t(-dt, dt);

    double rx = distribution_x(generator_x);
    double ry = distribution_y(generator_y);
    double rt = distribution_t(generator_t);

    std::get<0>(*virtual_pose) = std::get<0>(real_pose) + rx;
    std::get<1>(*virtual_pose) = std::get<1>(real_pose) + ry;
    std::get<2>(*virtual_pose) = std::get<2>(real_pose) + rt;

    wrapAngle(&std::get<2>(*virtual_pose));
  }

  /*****************************************************************************
  */
  static bool generatePose(
    const std::tuple<double,double,double>& base_pose,
    const std::vector< std::pair<double,double> >& map,
    const double& dxy, const double& dt, const double& dist_threshold,
    std::tuple<double,double,double>* real_pose)
  {
    assert(dxy >= 0.0);
    assert(dt >= 0.0);

    std::random_device rand_dev_x;
    std::random_device rand_dev_y;
    std::random_device rand_dev_t;
    std::mt19937 generator_x(rand_dev_x());
    std::mt19937 generator_y(rand_dev_y());
    std::mt19937 generator_t(rand_dev_t());

    std::uniform_real_distribution<double> distribution_x(-dxy, dxy);
    std::uniform_real_distribution<double> distribution_y(-dxy, dxy);
    std::uniform_real_distribution<double> distribution_t(-dt, dt);

    // A temp real pose
    std::tuple<double,double,double> real_pose_ass;

    // Fill in the orientation regardless
    double rt = distribution_t(generator_t);
    std::get<2>(real_pose_ass) = std::get<2>(base_pose) + rt;
    double t = std::get<2>(real_pose_ass);
    Utils::wrapAngle(&t);
    std::get<2>(real_pose_ass) = t;

    // We assume that the lidar sensor is distanced from the closest obstacle
    // by a certain amount (e.g. the radius of a circular base)
    bool pose_found = false;
    while (!pose_found)
    {
      pose_found = true;
      double rx = distribution_x(generator_x);
      double ry = distribution_y(generator_y);

      std::get<0>(real_pose_ass) = std::get<0>(base_pose) + rx;
      std::get<1>(real_pose_ass) = std::get<1>(base_pose) + ry;

      if (isPositionInMap(real_pose_ass, map))
      {
        for (unsigned int i = 0; i < map.size(); i++)
        {
          double dx = std::get<0>(real_pose_ass) - map[i].first;
          double dy = std::get<1>(real_pose_ass) - map[i].second;

          if (dx*dx + dy*dy < dist_threshold*dist_threshold)
          {
            pose_found = false;
            break;
          }
        }
      }
      else pose_found = false;
    }

    *real_pose = real_pose_ass;

    // Verify distance threshold
    std::vector< std::pair<double,double> > intersections =
      X::find(real_pose_ass, map, map.size());
    std::vector<double> real_scan;
    points2scan(intersections, real_pose_ass, &real_scan);

    unsigned int min_dist_idx =
      std::min_element(real_scan.begin(), real_scan.end()) - real_scan.begin();

    return real_scan[min_dist_idx] > dist_threshold;
  }

  /*****************************************************************************
  */
  static bool generatePoseWithinMap(
    const std::vector< std::pair<double,double> >& map,
    const double& dist_threshold,
    std::tuple<double,double,double>* pose)
  {
    // A temp real pose
    std::tuple<double,double,double> real_pose_ass;

    // Generate orientation
    std::random_device rand_dev_t;
    std::mt19937 generator_t(rand_dev_t());

    std::uniform_real_distribution<double> distribution_t(-M_PI, M_PI);

    // Fill in the orientation regardless
    std::get<2>(real_pose_ass) = distribution_t(generator_t);

    // Find the bounding box of the map
    double max_x = -1000.0;
    double min_x = +1000.0;
    double max_y = -1000.0;
    double min_y = +1000.0;

    for (unsigned int i = 0; i < map.size(); i++)
    {
      if (map[i].first > max_x)
        max_x = map[i].first;

      if (map[i].first < min_x)
        min_x = map[i].first;

      if (map[i].second > max_y)
        max_y = map[i].second;

      if (map[i].second < min_y)
        min_y = map[i].second;
    }

    std::random_device rand_dev_x;
    std::random_device rand_dev_y;
    std::mt19937 generator_x(rand_dev_x());
    std::mt19937 generator_y(rand_dev_y());

    std::uniform_real_distribution<double> distribution_x(min_x, max_x);
    std::uniform_real_distribution<double> distribution_y(min_y, max_y);

    // We assume that the lidar sensor is distanced from the closest obstacle
    // by a certain amount (e.g. the radius of a circular base)
    bool pose_found = false;
    while (!pose_found)
    {
      pose_found = true;
      double rx = distribution_x(generator_x);
      double ry = distribution_y(generator_y);

      std::get<0>(real_pose_ass) = rx;
      std::get<1>(real_pose_ass) = ry;

      if (isPositionInMap(real_pose_ass, map))
      {
        for (unsigned int i = 0; i < map.size(); i++)
        {
          double dx = std::get<0>(real_pose_ass) - map[i].first;
          double dy = std::get<1>(real_pose_ass) - map[i].second;

          if (dx*dx + dy*dy < dist_threshold*dist_threshold)
          {
            pose_found = false;
            break;
          }
        }
      }
      else pose_found = false;
    }

    *pose = real_pose_ass;

    // Verify distance threshold
    std::vector< std::pair<double,double> > intersections =
      X::find(real_pose_ass, map, map.size());
    std::vector<double> real_scan;
    points2scan(intersections, real_pose_ass, &real_scan);

    unsigned int min_dist_idx =
      std::min_element(real_scan.begin(), real_scan.end()) - real_scan.begin();

    return real_scan[min_dist_idx] > dist_threshold;
  }

  /*****************************************************************************
  */
  static std::vector<double> innerProduct(const std::vector<double>& vec1,
    const std::vector<double>& vec2)
  {
    assert(vec1.size() == vec2.size());

    std::vector<double> ret_vector;

    for (int i = 0; i < vec1.size(); i++)
    {
      ret_vector.push_back(vec1[i] * vec2[i]);
    }

    return ret_vector;
  }

  /*****************************************************************************
  */
  static std::vector< std::pair<double, double> > innerProductComplex(
    const std::vector< std::pair<double, double> >& vec1,
    const std::vector< std::pair<double, double> >& vec2)
  {
#ifdef TIMES
    std::chrono::high_resolution_clock::time_point start =
      std::chrono::high_resolution_clock::now();
#endif

    assert(vec1.size() == vec2.size());

    std::vector< std::pair<double, double> > ret_vector;

    for (int i = 0; i < vec1.size(); i++)
    {
      double re =
        vec1[i].first * vec2[i].first - vec1[i].second * vec2[i].second;
      double im =
        vec1[i].first * vec2[i].second + vec1[i].second * vec2[i].first;

      ret_vector.push_back(std::make_pair(re,im));
    }

#ifdef TIMES
    std::chrono::high_resolution_clock::time_point end =
      std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed =
      std::chrono::duration_cast< std::chrono::duration<double> >(end-start);

    printf("%f [innerProductComplex]\n", elapsed.count());
#endif

    return ret_vector;
  }

  /*****************************************************************************
  */
  static bool isPositionInMap(
    const std::tuple<double, double, double>& pose,
    const std::vector< std::pair<double,double> >& map)
  {
    Point_2 point(std::get<0>(pose), std::get<1>(pose));

    // Construct polygon from map
    Polygon_2 poly;
    for (int p = 0; p < map.size(); p++)
      poly.push_back(Point_2(map[p].first, map[p].second));

    poly.push_back(Point_2(map[map.size()-1].first, map[map.size()-1].second));

    bool inside = false;
    if(CGAL::bounded_side_2(poly.vertices_begin(),
        poly.vertices_end(),
        point, Kernel()) == CGAL::ON_BOUNDED_SIDE)
    {
      inside = true;
    }

    return inside;
  }

  /*****************************************************************************
  */
  static bool isPositionFartherThan(
    const std::tuple<double, double, double>& pose,
    const std::vector< std::pair<double,double> >& map,
    const double& dist)
  {
    for (unsigned int i = 0; i < map.size(); i++)
    {
      double dx = std::get<0>(pose) - map[i].first;
      double dy = std::get<1>(pose) - map[i].second;
      double d = sqrt(dx*dx + dy*dy);

      if (d < dist)
        return false;
    }

    return true;
  }

  /*****************************************************************************
  */
  static std::pair<double,double> multiplyWithRotationMatrix(
    const std::pair<double,double>& point, const double& angle)
  {
    double R11 = cos(angle);
    double R12 = -sin(angle);
    double R21 = -R12;
    double R22 = R11;

    double x = R11 * point.first + R12 * point.second;
    double y = R21 * point.first + R22 * point.second;

    return std::make_pair(x,y);

  }

  /*****************************************************************************
  */
  static std::vector< std::pair<double,double> > multiplyWithRotationMatrix(
    const std::vector< std::pair<double,double> >& points,
    const double& angle)
  {
    std::vector< std::pair<double,double> > return_vector;

    for (int i = 0; i < points.size(); i++)
      return_vector.push_back(multiplyWithRotationMatrix(points[i], angle));

    return return_vector;
  }

  /*****************************************************************************
  */
  static double norm(const std::pair<double,double>& vec)
  {
    return sqrt(vec.first*vec.first + vec.second*vec.second);
  }

  /*****************************************************************************
  */
  static std::vector<double> norm(
    const std::vector< std::pair<double,double> >& vec)
  {
    std::vector<double> ret_vector;

    for (int i = 0; i < vec.size(); i++)
      ret_vector.push_back(norm(vec[i]));

    return ret_vector;
  }

  /*****************************************************************************
  */
  static double norm2(const std::vector< std::pair<double,double> >& vec)
  {
    std::vector<double> ret_vector;

    for (int i = 0; i < vec.size(); i++)
      ret_vector.push_back(norm(vec[i]));

    return accumulate(ret_vector.begin(), ret_vector.end(), 0.0);
  }

  /*****************************************************************************
  */
  static std::pair<double,double> pairDiff(
    const std::pair<double,double>& pair1,
    const std::pair<double,double>& pair2)
  {
    std::pair<double,double> ret_pair;
    ret_pair.first = pair2.first - pair1.first;
    ret_pair.second = pair2.second - pair1.second;

    return ret_pair;
  }

  /*****************************************************************************
  */
  static void points2scan(
    const std::vector< std::pair<double,double> >& points,
    const std::tuple<double,double,double>& pose,
    std::vector<double>* scan)
  {
#ifdef TIMES
    std::chrono::high_resolution_clock::time_point start =
      std::chrono::high_resolution_clock::now();
#endif

    scan->clear();

    double px = std::get<0>(pose);
    double py = std::get<1>(pose);

    double dx = 0.0;
    double dy = 0.0;
    for (int i = 0; i < points.size(); i++)
    {
      dx = points[i].first - px;
      dy = points[i].second - py;
      scan->push_back(sqrt(dx*dx+dy*dy));
    }

#ifdef TIMES
    std::chrono::high_resolution_clock::time_point end =
      std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed =
      std::chrono::duration_cast< std::chrono::duration<double> >(end-start);

    printf("%f [points2scan]\n", elapsed.count());
#endif
  }

  /*****************************************************************************
  */
  static void scan2points(
    const std::vector<double>& scan,
    const std::tuple<double,double,double> pose,
    std::vector< std::pair<double,double> >* points,
    const double& angle_span = 2*M_PI)
  {
#ifdef TIMES
    std::chrono::high_resolution_clock::time_point start =
      std::chrono::high_resolution_clock::now();
#endif

    points->clear();

    double px = std::get<0>(pose);
    double py = std::get<1>(pose);
    double pt = std::get<2>(pose);

    // The angle of the first ray (in the local coordinate system)
    double sa = -angle_span/2;

    for (int i = 0; i < scan.size(); i++)
    {
      double x =
        px + scan[i] * cos(i * angle_span / scan.size() + pt + sa);
      double y =
        py + scan[i] * sin(i * angle_span / scan.size() + pt + sa);

      points->push_back(std::make_pair(x,y));
    }

#ifdef TIMES
    std::chrono::high_resolution_clock::time_point end =
      std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed =
      std::chrono::duration_cast< std::chrono::duration<double> >(end-start);

    printf("%f [scan2points]\n", elapsed.count());
#endif
  }

  /*****************************************************************************
  */
  static void scanFromPose(
    const std::tuple<double,double,double>& pose,
    const std::vector< std::pair<double,double> >& points,
    const unsigned int& num_rays,
    std::vector<double>* scan)
  {
    scan->clear();

    std::vector< std::pair<double,double> > intersections =
      X::find(pose, points, num_rays);

    points2scan(intersections, pose, scan);
  }

  /*****************************************************************************
  */
  static int sgn(const double& a)
  {
    return (a > 0.0) - (a < 0.0);
  }

  /*****************************************************************************
  */
  static std::vector<double>
  subsampleScan(const std::vector<double>& scan_in, const size_t& sz)
  {
    std::tuple<double,double,double> zero_pose;
    std::get<0>(zero_pose) = 0.0;
    std::get<1>(zero_pose) = 0.0;
    std::get<2>(zero_pose) = 0.0;

    // Turn scan to points
    std::vector< std::pair<double,double> > scan_points;
    scan2points(scan_in, zero_pose, &scan_points);

    // scan_out: the ranges to `scan_points` from `zero_pose`
    std::vector<double> scan_out;
    scanFromPose(zero_pose, scan_points, sz, &scan_out);

    return scan_out;
  }

  /***************************************************************************
  */
  static std::vector< std::pair<double,double> > vectorDiff(
    const std::vector< std::pair<double,double> >& vec)
  {
    std::vector< std::pair<double,double> > ret_vector;

    for (int i = 0; i < vec.size()-1; i++)
      ret_vector.push_back(pairDiff(vec[i], vec[i+1]));

    return ret_vector;
  }

  /***************************************************************************
  */
  static std::pair<double,double> vectorStatistics(
    const std::vector< double >& v)
  {
    double sum = std::accumulate(v.begin(), v.end(), 0.0);
    double mean = sum / v.size();

    std::vector<double> diff(v.size());
    std::transform(v.begin(), v.end(), diff.begin(),
      std::bind2nd(std::minus<double>(), mean));
    double sq_sum =
      std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
    double stdev = std::sqrt(sq_sum / v.size());

    return std::make_pair(mean, stdev);
  }

  /***************************************************************************
  */
  static void wrapAngle(double* angle)
  {
    *angle = fmod(*angle + 5*M_PI, 2*M_PI) - M_PI;
  }

};


// -----------------------------------------------------------------------------
class DatasetUtils
{
  public:

  /***************************************************************************
  */
  static std::vector< std::vector< std::pair<double,double> > >
    dataset2points(const char* dataset_filepath)
    {
      std::vector< std::vector<double> > ranges;
      std::vector< std::tuple<double,double,double> > poses;

      readDataset(dataset_filepath, &ranges, &poses);

      int num_scans = ranges.size();
      int num_rays = ranges[0].size();
      double angle_span = M_PI;

      std::vector< std::vector< std::pair<double,double> > > polygons;
      for (int s = 0; s < ranges.size(); s++)
      {
        double px = std::get<0>(poses[s]);
        double py = std::get<1>(poses[s]);
        double pt = std::get<2>(poses[s]);

        std::vector< std::pair<double,double> > polygon;

        for (int r = 0; r < ranges[s].size(); r++)
        {
          double x =
            px + ranges[s][r] * cos(r*angle_span/(num_rays-1) + pt -angle_span/2);
          double y =
            py + ranges[s][r] * sin(r*angle_span/(num_rays-1) + pt -angle_span/2);

          polygon.push_back(std::make_pair(x,y));
        }

        polygons.push_back(polygon);
      }

      return polygons;
    }

  /***************************************************************************
  */
  static void dataset2rangesAndPose(
    const char* dataset_filepath,
    std::vector<double>* ranges,
    std::tuple<double,double,double>* pose)
  {
    readDataset(dataset_filepath, ranges, pose);
  }

  /*****************************************************************************
  */
  static void readDataset(
    const char* filepath,
    std::vector<double>* ranges,
    std::tuple<double,double,double>* pose)
  {
    // First read the first two number: they show
    // (1) the number of scans and
    // (2) the number of rays per scan.
    FILE* fp = fopen(filepath, "r");
    if (fp == NULL)
      exit(EXIT_FAILURE);

    char* line = NULL;
    size_t len = 0;

    unsigned int line_number = 0;
    long int num_scans = 0;
    long int num_rays = 0;
    while ((getline(&line, &len, fp)) != -1 && line_number < 1)
    {
      line_number++;

      char * pEnd;
      num_scans = strtol (line, &pEnd, 10);
      num_rays = strtol (pEnd, &pEnd, 10);
    }

    fclose(fp);

    if (line)
      free(line);


    // Begin for all scans
    fp = fopen(filepath, "r");
    line = NULL;
    len = 0;

    // The line number read at each iteration
    line_number = 0;

    // loop
    while ((getline(&line, &len, fp)) != -1)
    {
      line_number++;

      // We don't have to care about the first line now
      if (line_number == 1)
        continue;

      // These lines host the poses from which the scans were taken
      if ((line_number-1) % (num_rays+1) == 0)
      {
        // The pose from which the scan_number-th scan was taken
        std::string pose_d(line); // convert from char to string
        std::string::size_type sz; // alias of size_t

        double px = std::stod(pose_d,&sz);
        pose_d = pose_d.substr(sz);
        double py = std::stod(pose_d,&sz);
        double pt = std::stod(pose_d.substr(sz));
        Utils::wrapAngle(&pt);
        *pose = std::make_tuple(px,py,pt);

        continue;
      }

      // At this point we are in a line holding a range measurement; fo sho
      double range_d;
      assert(sscanf(line, "%lf", &range_d) == 1);
      ranges->push_back(range_d);
    }

    fclose(fp);

    if (line)
      free(line);
  }

  /*****************************************************************************
  */
  static std::vector<double>
  interpolateRanges( const std::vector<double>& ranges)
  {
    // Identify contiguous regions of zero measurement
    std::vector< std::vector <int> > regions;
    int i = 0;
    int j;
    while(i < ranges.size())
    {
      if (ranges[i] == 0)
      {
        int region_begin = i;
        int region_end;
        bool broke = false;
        for (j = region_begin+1; j < ranges.size(); j++)
        {
          if (ranges[j] == 0)
            continue;
          if (ranges[j] > 0)
          {
            region_end = j-1;
            {
              broke = true;
              break;
            }
          }
        }

        if (j == ranges.size() && !broke)
          region_end = j-1;

        std::vector<int> region;
        region.push_back(region_begin);
        region.push_back(region_end);
        regions.push_back(region);

        i = j+1;
      }
      else
        i++;
    }


    // Inflate to consecutive indices
    for (unsigned int i = 0; i < regions.size(); i++)
    {
      int begin = regions[i][0];
      int end = regions[i][regions[i].size()-1];

      regions[i].clear();

      for (unsigned int j = begin; j <= end; j++)
        regions[i].push_back(j);
    }



    // Is the first index 0 and the last equal to the size-1?
    int num_regions = regions.size();
    int numel_last_region = regions[num_regions-1].size();

    if (regions[0][0] == 0 &&
      regions[num_regions-1][numel_last_region-1] == ranges.size()-1)
    {
      for (int i = 0; i < regions[0].size(); i++)
        regions[num_regions-1].push_back(regions[0][i]);

      regions.erase(regions.begin(), regions.begin()+1);
    }

    /*
       for (int i = 0; i < regions.size(); i++)
       {
       printf("region %d\n", i);
       for (int j = 0; j < regions[i].size(); j++)
       printf("%d,", regions[i][j]);
       printf("\n");
       }
       */

    std::vector<double> ranges_interp = ranges;

    for (unsigned int i = 0; i < regions.size(); i++)
    {
      int interp_begin = regions[i][0]-1;
      int interp_end = regions[i][regions[i].size()-1]+1;

      if (interp_begin < 0)
        interp_begin = ranges.size()-1;

      if (interp_end >= ranges.size())
        interp_end = 0;

      double range_a = ranges[interp_begin];
      double range_b = ranges[interp_end];
      double interp = (range_a + range_b)/2;

      for (unsigned int j = 0; j < regions[i].size(); j++)
        ranges_interp[regions[i][j]]= interp;
    }

    /*
       for (int j = 0; j < ranges.size(); j++)
       printf("%f\n", ranges_interp[j]);
       */

    return ranges_interp;
  }

  /*****************************************************************************
  */
  static void readDataset(
    const char* filepath,
    std::vector< std::vector<double> >* ranges,
    std::vector< std::tuple<double,double,double> >* poses)
  {
    // First read the first two number: they show
    // (1) the number of scans and
    // (2) the number of rays per scan.
    FILE* fp = fopen(filepath, "r");
    if (fp == NULL)
      exit(EXIT_FAILURE);

    char* line = NULL;
    size_t len = 0;

    unsigned int line_number = 0;
    long int num_scans = 0;
    long int num_rays = 0;
    while ((getline(&line, &len, fp)) != -1 && line_number < 1)
    {
      line_number++;

      char * pEnd;
      num_scans = strtol (line, &pEnd, 10);
      num_rays = strtol (pEnd, &pEnd, 10);
    }

    fclose(fp);

    if (line)
      free(line);


    // Begin for all scans
    fp = fopen(filepath, "r");
    line = NULL;
    len = 0;

    // The line number read at each iteration
    line_number = 0;

    // A vector holding scan ranges for one scan
    std::vector<double> ranges_one_scan;

    // loop
    while ((getline(&line, &len, fp)) != -1)
    {
      line_number++;

      // We don't have to care about the first line now
      if (line_number == 1)
        continue;

      // These lines host the poses from which the scans were taken
      if ((line_number-1) % (num_rays+1) == 0)
      {
        // Finished with this scan
        ranges->push_back(ranges_one_scan);

        // Clear the vector so we can begin all over
        ranges_one_scan.clear();

        // The pose from which the scan_number-th scan was taken
        std::string pose(line); // convert from char to string
        std::string::size_type sz; // alias of size_t

        double px = std::stod(pose,&sz);
        pose = pose.substr(sz);
        double py = std::stod(pose,&sz);
        double pt = std::stod(pose.substr(sz));
        Utils::wrapAngle(&pt);
        poses->push_back(std::make_tuple(px,py,pt));

        continue;
      }

      // At this point we are in a line holding a range measurement; fo sho
      double range;
      assert(sscanf(line, "%lf", &range) == 1);
      ranges_one_scan.push_back(range);
    }

    fclose(fp);

    if (line)
      free(line);
  }


  /****************************************************************************
  */
  static void printDataset(const char* dataset_filepath)
  {
    std::vector< std::vector<double> > ranges;
    std::vector< std::tuple<double,double,double> > poses;

    readDataset(dataset_filepath, &ranges, &poses);

    for (int s = 0; s < ranges.size(); s++)
    {
      printf("NEW SCAN\n");
      for (int r = 0; r < ranges[s].size(); r++)
      {
        printf("r[%d] = %f\n", r, ranges[s][r]);
      }

      printf("FROM POSE (%f,%f,%f)\n",
        std::get<0>(poses[s]), std::get<1>(poses[s]), std::get<2>(poses[s]));
    }
  }
};


// -----------------------------------------------------------------------------
class Dump
{
  public:

  /*****************************************************************************
  */
  static void scan(
    const std::vector<double>& real_scan,
    const std::tuple<double,double,double>& real_pose,
    const std::vector<double>& virtual_scan,
    const std::tuple<double,double,double>& virtual_pose,
    const std::string& dump_filepath)
  {
    std::vector< std::pair<double,double> > real_scan_points;
    Utils::scan2points(real_scan, real_pose, &real_scan_points);

    std::vector< std::pair<double,double> > virtual_scan_points;
    Utils::scan2points(virtual_scan, virtual_pose, &virtual_scan_points);

    std::ofstream file(dump_filepath.c_str(), std::ios::trunc);

    if (file.is_open())
    {
      file << "rx = [];" << std::endl;
      file << "ry = [];" << std::endl;

      for (int i = 0; i < real_scan.size(); i++)
      {
        file << "rx = [rx " << real_scan_points[i].first << "];" << std::endl;
        file << "ry = [ry " << real_scan_points[i].second << "];" << std::endl;
      }

      file << "vx = [];" << std::endl;
      file << "vy = [];" << std::endl;
      for (int i = 0; i < virtual_scan.size(); i++)
      {
        file << "vx = [vx " << virtual_scan_points[i].first << "];" << std::endl;
        file << "vy = [vy " << virtual_scan_points[i].second << "];" << std::endl;
      }

      file << "r00 = [" << std::get<0>(real_pose) <<
        ", " << std::get<1>(real_pose) << "];" << std::endl;
      file << "v00 = [" << std::get<0>(virtual_pose) <<
        ", " << std::get<1>(virtual_pose) << "];" << std::endl;

      file.close();
    }
    else
      printf("Could not log scans\n");
  }

  /*****************************************************************************
  */
  static void rangeScan(
    const std::vector<double>& real_scan,
    const std::vector<double>& virtual_scan,
    const std::string& dump_filepath)
  {
    std::ofstream file(dump_filepath.c_str(), std::ios::trunc);

    if (file.is_open())
    {
      file << "rr = [];" << std::endl;
      for (int i = 0; i < real_scan.size(); i++)
        file << "rr = [rr " << real_scan[i] << "];" << std::endl;

      file << "rt = [];" << std::endl;
      for (int i = 0; i < real_scan.size(); i++)
        file << "rt = [rt " << i * 2 * M_PI / real_scan.size() << "];" << std::endl;

      file << "vr = [];" << std::endl;
      for (int i = 0; i < virtual_scan.size(); i++)
        file << "vr = [vr " << virtual_scan[i] << "];" << std::endl;

      file << "vt = [];" << std::endl;
      for (int i = 0; i < virtual_scan.size(); i++)
        file << "vt = [vt " << i * 2 * M_PI / virtual_scan.size() << "];" << std::endl;

      file.close();
    }
    else
      printf("Could not log range scans\n");
  }

  /*****************************************************************************
  */
  static void map(const std::vector< std::pair<double,double> >& map,
    const std::string& dump_filepath)
  {
    std::ofstream file(dump_filepath.c_str(), std::ios::trunc);

    if (file.is_open())
    {
      file << "mx = [];" << std::endl;
      file << "my = [];" << std::endl;
      for (int i = 0; i < map.size(); i++)
      {
        file << "mx = [mx " << map[i].first << "];" << std::endl;
        file << "my = [my " << map[i].second << "];" << std::endl;
      }

      file.close();
    }
    else
      printf("Could not log scans\n");
  }

  /*****************************************************************************
  */
  static void points(const std::vector< std::pair<double,double> >& real_points,
    const std::vector< std::pair<double,double> >& virtual_points,
    const unsigned int& id,
    const std::string& dump_filepath)
  {
    std::ofstream file(dump_filepath.c_str(), std::ios::trunc);

    if (file.is_open())
    {
      file << "rx = [];" << std::endl;
      file << "ry = [];" << std::endl;
      for (int i = 0; i < real_points.size(); i++)
      {
        file << "rx = [rx " << real_points[i].first << "];" << std::endl;
        file << "ry = [ry " << real_points[i].second << "];" << std::endl;
      }

      file << "vx = [];" << std::endl;
      file << "vy = [];" << std::endl;
      for (int i = 0; i < virtual_points.size(); i++)
      {
        file << "vx = [vx " << virtual_points[i].first << "];" << std::endl;
        file << "vy = [vy " << virtual_points[i].second << "];" << std::endl;
      }

      file.close();
    }
    else
      printf("Could not log points\n");
  }

  /*****************************************************************************
  */
  static void polygon(const Polygon_2& poly, const std::string& dump_filepath)
  {
    std::ofstream file(dump_filepath.c_str(), std::ios::trunc);

    if (file.is_open())
    {
      file << "px = [];" << std::endl;
      file << "py = [];" << std::endl;

      for (VertexIterator vi = poly.vertices_begin();
        vi != poly.vertices_end(); vi++)
      {
        file << "px = [px " << vi->x() << "];" << std::endl;
        file << "py = [py " << vi->y() << "];" << std::endl;
      }

      file.close();
    }
    else
      printf("Could not log polygon\n");
  }

  /*****************************************************************************
  */
  static void polygons(const Polygon_2& real_poly,
    const Polygon_2& virtual_poly,
    const std::string& dump_filepath)
  {
    std::ofstream file(dump_filepath.c_str(), std::ios::trunc);

    if (file.is_open())
    {
      file << "p_rx = [];" << std::endl;
      file << "p_ry = [];" << std::endl;

      for (VertexIterator vi = real_poly.vertices_begin();
        vi != real_poly.vertices_end(); vi++)
      {
        file << "p_rx = [p_rx " << vi->x() << "];" << std::endl;
        file << "p_ry = [p_ry " << vi->y() << "];" << std::endl;
      }

      file << "p_vx = [];" << std::endl;
      file << "p_vy = [];" << std::endl;

      for (VertexIterator vi = virtual_poly.vertices_begin();
        vi != virtual_poly.vertices_end(); vi++)
      {
        file << "p_vx = [p_vx " << vi->x() << "];" << std::endl;
        file << "p_vy = [p_vy " << vi->y() << "];" << std::endl;
      }

      file.close();
    }
    else
      printf("Could not log polygons \n");
  }

  /*****************************************************************************
  */
  static void convexHulls(const std::vector<Point_2>& real_hull,
    const std::vector<Point_2>& virtual_hull,
    const std::string& dump_filepath)
  {
    std::ofstream file(dump_filepath.c_str(), std::ios::trunc);

    if (file.is_open())
    {
      file << "h_rx = [];" << std::endl;
      file << "h_ry = [];" << std::endl;

      for (int i = 0; i < real_hull.size(); i++)
      {
        file << "h_rx = [h_rx " << real_hull[i].x() << "];" << std::endl;
        file << "h_ry = [h_ry " << real_hull[i].y() << "];" << std::endl;
      }

      file << "h_vx = [];" << std::endl;
      file << "h_vy = [];" << std::endl;

      for (int i = 0; i < virtual_hull.size(); i++)
      {
        file << "h_vx = [h_vx " << virtual_hull[i].x() << "];" << std::endl;
        file << "h_vy = [h_vy " << virtual_hull[i].y() << "];" << std::endl;
      }

      file.close();
    }
    else
      printf("Could not log hulls \n");
  }

};

// -----------------------------------------------------------------------------
class ScanCompletion
{
  public:

  /*****************************************************************************
  */
  static void completeScan(std::vector<double>* scan, const int& method)
  {
    if (method == 1)
      completeScan1(scan);
    else if (method == 3)
      completeScan3(scan);
    else if (method == 4)
      completeScan4(scan);
    else
      completeScan1(scan);
  }

  /*****************************************************************************
  */
  static void completeScan1(std::vector<double>* scan)
  {
    std::vector<double> scan_copy = *scan;

    for (int i = scan_copy.size()-2; i > 0; i--)
      scan->push_back(scan_copy[i]);

    // Rotate so that it starts from -M_PI rather than -M_PI / 2
    int num_pos = scan->size() / 4;

    std::rotate(scan->begin(),
      scan->begin() + scan->size() - num_pos,
      scan->end());
  }

  /*****************************************************************************
  */
  static void completeScan2(std::vector<double>* scan,
    const std::tuple<double,double,double>& pose)
  {
    std::vector<double> scan_copy = *scan;

    // Locate the first and last points of the scan in the 2D plane
    std::vector< std::pair<double,double> > points;
    Utils::scan2points(scan_copy, pose, &points);
    std::pair<double,double> start_point = points[0];
    std::pair<double,double> end_point = points[points.size()-1];

    double dx = start_point.first - end_point.first;
    double dy = start_point.second - end_point.second;
    double d = sqrt(dx*dx + dy*dy);
    double r = d/2;

    for (int i = scan_copy.size()-2; i > 0; i--)
      scan->push_back(r);

    // Rotate so that it starts from -M_PI rather than -M_PI / 2
    int num_pos = scan->size() / 4;

    std::rotate(scan->begin(),
      scan->begin() + scan->size() - num_pos,
      scan->end());
  }

  /*****************************************************************************
  */
  static void completeScan3(std::vector<double>* scan)
  {
    std::vector<double> scan_copy = *scan;

    for (int i = 1; i < scan_copy.size()-1; i++)
      scan->push_back(scan_copy[i]);

    // Rotate so that it starts from -M_PI rather than -M_PI / 2
    int num_pos = scan->size() / 4;

    std::rotate(scan->begin(),
      scan->begin() + scan->size() - num_pos,
      scan->end());
  }

  /*****************************************************************************
  */
  static void completeScan4(std::vector<double>* scan)
  {
    // Find closest and furthest points in original scan
    double min_range = *std::min_element(scan->begin(), scan->end());
    double max_range = *std::max_element(scan->begin(), scan->end());
    double fill_range = min_range;

    unsigned int scan_size = scan->size();

    for (int i = 1; i < scan_size-1; i++)
      scan->push_back(fill_range);

    // Rotate so that it starts from -M_PI rather than -M_PI / 2
    assert(fmod(scan->size(), 2) == 0);
    int num_pos = scan->size() / 4;

    std::rotate(scan->begin(),
      scan->begin() + scan->size() - num_pos,
      scan->end());
  }

  /*****************************************************************************
  */
  static void completeScan5(
    const std::tuple<double,double,double>& pose,
    const std::vector<double>& scan_in,
    const unsigned int& num_rays,
    std::vector<double>* scan_out,
    std::vector< std::pair<double,double> >* map,
    std::tuple<double,double,double>* map_origin)
  {
    std::vector< std::pair<double,double> > scan_points;
    Utils::scan2points(scan_in, pose, &scan_points, M_PI);

    std::tuple<double,double,double> pose_within_points = pose;

    double farther_than = 0.01;
    bool is_farther_than = false;

    while (!is_farther_than)
    {
      do Utils::generatePose(pose,
        0.05, 0.0, &pose_within_points);
      while(!Utils::isPositionInMap(pose_within_points, scan_points));

      *map = X::find(pose_within_points, scan_points, num_rays);

      is_farther_than =
        Utils::isPositionFartherThan(pose_within_points, *map, farther_than);
    }

    *map_origin = pose_within_points;
    Utils::points2scan(*map, *map_origin, scan_out);
  }
};


// -----------------------------------------------------------------------------
class DFTUtils
{
  public:

  /*****************************************************************************
  */
  static void fftshift(std::vector<double>* vec)
  {
#ifdef TIMES
    std::chrono::high_resolution_clock::time_point a =
      std::chrono::high_resolution_clock::now();
#endif

    std::rotate(
      vec->begin(),
      vec->begin() + static_cast<unsigned int>(vec->size()/2),
      vec->end());

#ifdef TIMES
    std::chrono::high_resolution_clock::time_point b =
      std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> elapsed =
      std::chrono::duration_cast< std::chrono::duration<double> >(b-a);

    printf("%f [fftshift]\n", elapsed.count());
#endif
  }

  /*****************************************************************************
   * @brief Calculates the X1 coefficient of the rays_diff input vector.
   * @param[in] rays_diff [const std::vector<double>&] The difference in range
   * between a world and a map scan.
   * @return [std::vector<double>] A vector of size two, of which the first
   * position holds the real part of the first DFT coefficient, and the
   * second the imaginary part of it.
   */
  static std::vector<double> getFirstDFTCoefficient(
    const std::vector<double>& rays_diff)
  {
#ifdef TIMES
    std::chrono::high_resolution_clock::time_point a =
      std::chrono::high_resolution_clock::now();
#endif

    // A vector holding the coefficients of the DFT
    std::vector<double> dft_coeff_vector;

    // Do the DFT thing
    std::vector<double> dft_coeffs = dft(rays_diff);

    // The real and imaginary part of the first coefficient are
    // out[1] and out[N-1] respectively

    // The real part of the first coefficient
    double x1_r = dft_coeffs[1];

    // The imaginary part of the first coefficient
    double x1_i = dft_coeffs[rays_diff.size()-1];

    // Is x1_r finite?
    if (std::isfinite(x1_r))
      dft_coeff_vector.push_back(x1_r);
    else
      dft_coeff_vector.push_back(0.0);

    // Is x1_i finite?
    if (std::isfinite(x1_i))
      dft_coeff_vector.push_back(x1_i);
    else
      dft_coeff_vector.push_back(0.0);

#ifdef TIMES
    std::chrono::high_resolution_clock::time_point b =
      std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> elapsed =
      std::chrono::duration_cast< std::chrono::duration<double> >(b-a);

    printf("%f [getFirstDFTCoefficient]\n", elapsed.count());
#endif

    return dft_coeff_vector;
  }

  /****************************************************************************
  */
  static std::vector<double> getFirstDFTCoefficient(
    const std::vector<double>& rays_diff,
    const fftw_plan& r2rp)
  {
#ifdef TIMES
    std::chrono::high_resolution_clock::time_point a =
      std::chrono::high_resolution_clock::now();
#endif

    // A vector holding the coefficients of the DFT
    std::vector<double> dft_coeff_vector;

    // Do the DFT thing
    std::vector<double> dft_coeffs = dft(rays_diff, r2rp);

    // The real and imaginary part of the first coefficient are
    // out[1] and out[N-1] respectively

    // The real part of the first coefficient
    double x1_r = dft_coeffs[1];

    // The imaginary part of the first coefficient
    double x1_i = dft_coeffs[rays_diff.size()-1];

    // Is x1_r finite?
    if (std::isfinite(x1_r))
      dft_coeff_vector.push_back(x1_r);
    else
      dft_coeff_vector.push_back(0.0);

    // Is x1_i finite?
    if (std::isfinite(x1_i))
      dft_coeff_vector.push_back(x1_i);
    else
      dft_coeff_vector.push_back(0.0);

#ifdef TIMES
    std::chrono::high_resolution_clock::time_point b =
      std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> elapsed =
      std::chrono::duration_cast< std::chrono::duration<double> >(b-a);

    printf("%f [getFirstDFTCoefficient]\n", elapsed.count());
#endif

    return dft_coeff_vector;
  }

  /*****************************************************************************
  */
  static std::vector< std::pair<double, double> >
  getDFTCoefficientsPairs(const std::vector<double>& coeffs)
  {
#ifdef TIMES
    std::chrono::high_resolution_clock::time_point a =
      std::chrono::high_resolution_clock::now();
#endif

    std::vector< std::pair<double, double> > fft_coeff_pairs;
    for (int i = 0; i <= coeffs.size()/2; i++)
    {
      if (i == 0 || i == coeffs.size()/2)
        fft_coeff_pairs.push_back(std::make_pair(coeffs[i], 0.0));
      else
      {
        fft_coeff_pairs.push_back(
          std::make_pair(coeffs[i], coeffs[coeffs.size()-i]));
      }
    }

    std::vector< std::pair<double, double> > fft_coeff_pairs_bak =
      fft_coeff_pairs;
    for (int i = fft_coeff_pairs_bak.size()-2; i > 0; i--)
    {
      fft_coeff_pairs.push_back(
        std::make_pair(fft_coeff_pairs_bak[i].first, -fft_coeff_pairs_bak[i].second));
    }

#ifdef TIMES
    std::chrono::high_resolution_clock::time_point b =
      std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> elapsed =
      std::chrono::duration_cast< std::chrono::duration<double> >(b-a);

    printf("%f [getDFTCoefficientsPairs]\n", elapsed.count());
#endif

    return fft_coeff_pairs;
  }

  /*****************************************************************************
   * @brief Performs DFT in a vector of doubles via fftw. Returns the DFT
   * coefficients vector in the order described in
   * http://www.fftw.org/fftw3_doc/Real_002dto_002dReal-Transform-Kinds.html#Real_002dto_002dReal-Transform-Kinds.
   * @param[in] rays_diff [const std::vector<double>&] The vector of differences
   * in range between a world scan and a map scan.
   * @return [std::vector<double>] The vector's DFT coefficients.
   */
  static std::vector<double> dft(const std::vector<double>& rays_diff)
  {
#ifdef TIMES
    std::chrono::high_resolution_clock::time_point a =
      std::chrono::high_resolution_clock::now();
#endif

    double* in;
    double* out;

    const size_t num_rays = rays_diff.size();

    in = (double*) fftw_malloc(num_rays * sizeof(double));
    out = (double*) fftw_malloc(num_rays * sizeof(double));

    // Create plan
    fftw_plan p = fftw_plan_r2r_1d(num_rays, in, out, FFTW_R2HC, FFTW_MEASURE);

    // Transfer the input vector to a structure preferred by fftw
    for (unsigned int i = 0; i < num_rays; i++)
      in[i] = rays_diff[i];

    // Execute plan
    fftw_execute(p);

    // Store all DFT coefficients
    std::vector<double> dft_coeff_vector;
    for (unsigned int i = 0; i < num_rays; i++)
      dft_coeff_vector.push_back(out[i]);

    // Free memory
    fftw_destroy_plan(p);
    fftw_free(out);
    fftw_free(in);

#ifdef TIMES
    std::chrono::high_resolution_clock::time_point b =
      std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> elapsed =
      std::chrono::duration_cast< std::chrono::duration<double> >(b-a);

    printf("%f [dft]\n", elapsed.count());
#endif

    return dft_coeff_vector;
  }

  /*****************************************************************************
  */
  static std::vector<double> dft(const std::vector<double>& rays_diff,
    const fftw_plan& r2rp)
  {
#ifdef TIMES
    std::chrono::high_resolution_clock::time_point a =
      std::chrono::high_resolution_clock::now();
#endif

    double* in;
    double* out;

    const size_t num_rays = rays_diff.size();

    in = (double*) fftw_malloc(num_rays * sizeof(double));
    out = (double*) fftw_malloc(num_rays * sizeof(double));

    // Create plan
    //fftw_plan p = fftw_plan_r2r_1d(num_rays, in, out, FFTW_R2HC, FFTW_MEASURE);

    // Transfer the input vector to a structure preferred by fftw
    for (unsigned int i = 0; i < num_rays; i++)
      in[i] = rays_diff[i];


    // Execute plan
    fftw_execute_r2r(r2rp, in, out);

    // Store all DFT coefficients
    std::vector<double> dft_coeff_vector;
    for (unsigned int i = 0; i < num_rays; i++)
      dft_coeff_vector.push_back(out[i]);

    // Free memory
    //fftw_destroy_plan(p);
    fftw_free(out);
    fftw_free(in);

#ifdef TIMES
    std::chrono::high_resolution_clock::time_point b =
      std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> elapsed =
      std::chrono::duration_cast< std::chrono::duration<double> >(b-a);

    printf("%f [dft]\n", elapsed.count());
#endif

    return dft_coeff_vector;
  }

  /*****************************************************************************
  */
  static std::vector< std::vector<double> > dftBatch(
    const std::vector< std::vector<double> >& scans)
  {
#ifdef TIMES
    std::chrono::high_resolution_clock::time_point a =
      std::chrono::high_resolution_clock::now();
#endif

    assert(scans.size() > 0);

    // What will be returned
    std::vector< std::vector<double> > coeff_vector_v;

    // Input/output arrays for fftw
    double* in;
    double* out;

    const size_t num_rays = scans[0].size();

    in = (double*) fftw_malloc(num_rays * sizeof(double));
    out = (double*) fftw_malloc(num_rays * sizeof(double));

    // Create plan once
    fftw_plan p = fftw_plan_r2r_1d(num_rays, in, out, FFTW_R2HC, FFTW_MEASURE);

    for (unsigned int v = 0; v < scans.size(); v++)
    {
      // Transfer the input vector to a structure preferred by fftw
      for (unsigned int i = 0; i < num_rays; i++)
        in[i] = scans[v][i];

      // Execute plan with new input/output arrays
      fftw_execute_r2r(p, in, out);

      // Store all DFT coefficients for the v-th scan
      std::vector<double> dft_coeffs;
      for (unsigned int i = 0; i < num_rays; i++)
        dft_coeffs.push_back(out[i]);

      coeff_vector_v.push_back(dft_coeffs);
    }

    // Free memory
    fftw_destroy_plan(p);
    fftw_free(out);
    fftw_free(in);


#ifdef TIMES
    std::chrono::high_resolution_clock::time_point b =
      std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> elapsed =
      std::chrono::duration_cast< std::chrono::duration<double> >(b-a);

    printf("%f [dftBatch]\n", elapsed.count());
#endif

    return coeff_vector_v;
  }

  /*****************************************************************************
  */
  static std::vector< std::vector<double> > dftBatch(
    const std::vector< std::vector<double> >& scans,
    const fftw_plan& r2rp)
  {
#ifdef TIMES
    std::chrono::high_resolution_clock::time_point a =
      std::chrono::high_resolution_clock::now();
#endif

    assert(scans.size() > 0);

    // What will be returned
    std::vector< std::vector<double> > coeff_vector_v;

    // Input/output arrays for fftw
    double* in;
    double* out;

    const size_t num_rays = scans[0].size();

    in = (double*) fftw_malloc(num_rays * sizeof(double));
    out = (double*) fftw_malloc(num_rays * sizeof(double));

    // Create plan once
    //fftw_plan p = fftw_plan_r2r_1d(num_rays, in, out, FFTW_R2HC, FFTW_MEASURE);

    for (unsigned int v = 0; v < scans.size(); v++)
    {
      // Transfer the input vector to a structure preferred by fftw
      for (unsigned int i = 0; i < num_rays; i++)
        in[i] = scans[v][i];

      // Execute plan with new input/output arrays
      fftw_execute_r2r(r2rp, in, out);

      // Store all DFT coefficients for the v-th scan
      std::vector<double> dft_coeffs;
      for (unsigned int i = 0; i < num_rays; i++)
        dft_coeffs.push_back(out[i]);

      coeff_vector_v.push_back(dft_coeffs);
    }

    // Free memory
    //fftw_destroy_plan(p);
    fftw_free(out);
    fftw_free(in);


#ifdef TIMES
    std::chrono::high_resolution_clock::time_point b =
      std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> elapsed =
      std::chrono::duration_cast< std::chrono::duration<double> >(b-a);

    printf("%f [dftBatch]\n", elapsed.count());
#endif

    return coeff_vector_v;
  }

  /*****************************************************************************
  */
  static std::vector<double> idft(
    const std::vector<std::pair<double, double> >& rays_diff)
  {
#ifdef TIMES
    std::chrono::high_resolution_clock::time_point a =
      std::chrono::high_resolution_clock::now();
#endif

    fftw_complex* in;
    double* out;

    const size_t num_rays = rays_diff.size();

    in = (fftw_complex*) fftw_malloc(num_rays * sizeof(fftw_complex));
    out = (double*) fftw_malloc(num_rays * sizeof(double));

    // Create plan
    fftw_plan p = fftw_plan_dft_c2r_1d(num_rays, in, out, FFTW_MEASURE);

    // Transfer the input vector to a structure preferred by fftw
    for (unsigned int i = 0; i < num_rays; i++)
    {
      in[i][0] = rays_diff[i].first;
      in[i][1] = rays_diff[i].second;
    }

    // Execute plan
    fftw_execute(p);

    // Store all DFT coefficients
    std::vector<double> dft_coeff_vector;
    for (unsigned int i = 0; i < num_rays; i++)
      dft_coeff_vector.push_back(out[i]/num_rays);

    // Free memory
    fftw_destroy_plan(p);
    fftw_free(out);
    fftw_free(in);

#ifdef TIMES
    std::chrono::high_resolution_clock::time_point b =
      std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> elapsed =
      std::chrono::duration_cast< std::chrono::duration<double> >(b-a);

    printf("%f [idft]\n", elapsed.count());
#endif

    return dft_coeff_vector;
  }

  /*****************************************************************************
  */
  static std::vector< std::vector<double> > idftBatch(
    const std::vector< std::vector<std::pair<double, double> > >& scans)
  {
#ifdef TIMES
    std::chrono::high_resolution_clock::time_point a =
      std::chrono::high_resolution_clock::now();
#endif

    assert(scans.size() > 0);

    // What will be returned
    std::vector< std::vector<double> > dft_coeffs_v;

    fftw_complex* in;
    double* out;

    const size_t num_rays = scans[0].size();

    in = (fftw_complex*) fftw_malloc(num_rays * sizeof(fftw_complex));
    out = (double*) fftw_malloc(num_rays * sizeof(double));

    // Create plan once
    fftw_plan p = fftw_plan_dft_c2r_1d(num_rays, in, out, FFTW_MEASURE);


    for (unsigned int v = 0; v < scans.size(); v++)
    {
      // Transfer the input vector to a structure preferred by fftw
      for (unsigned int i = 0; i < num_rays; i++)
      {
        in[i][0] = scans[v][i].first;
        in[i][1] = scans[v][i].second;
      }

      // Execute plan
      fftw_execute_dft_c2r(p, in, out);

      // Store all DFT coefficients
      std::vector<double> dft_coeffs;
      for (unsigned int i = 0; i < num_rays; i++)
        dft_coeffs.push_back(out[i]/num_rays);

      dft_coeffs_v.push_back(dft_coeffs);
    }

    // Free memory
    fftw_destroy_plan(p);
    fftw_free(out);
    fftw_free(in);

#ifdef TIMES
    std::chrono::high_resolution_clock::time_point b =
      std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> elapsed =
      std::chrono::duration_cast< std::chrono::duration<double> >(b-a);

    printf("%f [idftBatch]\n", elapsed.count());
#endif

    return dft_coeffs_v;
  }

  /*****************************************************************************
  */
  static std::vector< std::vector<double> > idftBatch(
    const std::vector< std::vector<std::pair<double, double> > >& scans,
    const fftw_plan& c2rp)
  {
#ifdef TIMES
    std::chrono::high_resolution_clock::time_point a =
      std::chrono::high_resolution_clock::now();
#endif

    assert(scans.size() > 0);

    // What will be returned
    std::vector< std::vector<double> > dft_coeffs_v;

    fftw_complex* in;
    double* out;

    const size_t num_rays = scans[0].size();

    in = (fftw_complex*) fftw_malloc(num_rays * sizeof(fftw_complex));
    out = (double*) fftw_malloc(num_rays * sizeof(double));

    // Create plan once
    //fftw_plan p = fftw_plan_dft_c2r_1d(num_rays, in, out, FFTW_MEASURE);


    for (unsigned int v = 0; v < scans.size(); v++)
    {
      // Transfer the input vector to a structure preferred by fftw
      for (unsigned int i = 0; i < num_rays; i++)
      {
        in[i][0] = scans[v][i].first;
        in[i][1] = scans[v][i].second;
      }

      // Execute plan
      fftw_execute_dft_c2r(c2rp, in, out);

      // Store all DFT coefficients
      std::vector<double> dft_coeffs;
      for (unsigned int i = 0; i < num_rays; i++)
        dft_coeffs.push_back(out[i]/num_rays);

      dft_coeffs_v.push_back(dft_coeffs);
    }

    // Free memory
    //fftw_destroy_plan(p);
    fftw_free(out);
    fftw_free(in);

#ifdef TIMES
    std::chrono::high_resolution_clock::time_point b =
      std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> elapsed =
      std::chrono::duration_cast< std::chrono::duration<double> >(b-a);

    printf("%f [idftBatch]\n", elapsed.count());
#endif

    return dft_coeffs_v;
  }
};


// -----------------------------------------------------------------------------
class Translation
{
  public:

  /*****************************************************************************
  */
  static double tff(
    const std::vector< double >& real_scan,
    const std::tuple<double,double,double>& virtual_pose,
    const std::vector< std::pair<double,double> >& map,
    const int& max_iterations,
    const double& dist_bound,
    const bool& pick_min,
    const fftw_plan& r2rp,
    int* result_iterations,
    std::chrono::duration<double>* intersections_time,
    std::tuple<double,double,double>* result_pose)
  {
#ifdef PRINTS
    printf("input pose  (%f,%f,%f) [Translation::tff]\n",
      std::get<0>(virtual_pose),
      std::get<1>(virtual_pose),
      std::get<2>(virtual_pose));
#endif

    std::tuple<double,double,double> current_pose = virtual_pose;

    std::vector<double> errors_xy;

    std::vector<double> deltas;
    std::vector<double> sum_d_vs;
    std::vector<double> x_es;
    std::vector<double> y_es;
    double norm_x1;

    // Start the clock
    std::chrono::high_resolution_clock::time_point start =
      std::chrono::high_resolution_clock::now();

    // Iterate
    unsigned int it = 1;
    double inclusion_bound = 1000.0;
    double err = 1.0 / real_scan.size();
    std::vector<double> d_v;
    double sum_d_v = 1.0 / real_scan.size();

    for (it = 1; it <= max_iterations; it++)
    {
      // Measure the time to find intersections
      std::chrono::high_resolution_clock::time_point int_start =
        std::chrono::high_resolution_clock::now();

      // Find the intersections of the rays from the estimated pose and
      // the map.
      std::vector< std::pair<double,double> > virtual_scan_intersections =
        X::find(current_pose, map, real_scan.size());

      std::chrono::high_resolution_clock::time_point int_end =
        std::chrono::high_resolution_clock::now();
      *intersections_time =
        std::chrono::duration_cast< std::chrono::duration<double> >(int_end-int_start);

      // Find the corresponding ranges
      std::vector<double> virtual_scan_it;
      Utils::points2scan(virtual_scan_intersections, current_pose, &virtual_scan_it);

      assert(virtual_scan_it.size() == real_scan.size());

      //inclusion_bound = real_scan.size()/2*err;
      //inclusion_bound = 0.01*sum_d;
      //inclusion_bound = M_PI * (sum_d + err) / real_scan.size();
      //inclusion_bound = 2*M_PI * sum_d_v / real_scan.size();
      inclusion_bound = real_scan.size()/4*err;

      // Obtain the correction vector
      std::pair<double,double> errors_xy =
        tffCore(real_scan, virtual_scan_it, std::get<2>(current_pose),
          inclusion_bound, r2rp, &d_v, &norm_x1);



      // These are the corrections
      double x_e = errors_xy.first;
      double y_e = errors_xy.second;


      // The norm of the correction vector
      double err_sq = x_e*x_e + y_e*y_e;
      err = sqrt(err_sq);

      // Correct the position
      std::get<0>(current_pose) += x_e;
      std::get<1>(current_pose) += y_e;

      double dx = std::get<0>(current_pose) - std::get<0>(virtual_pose);
      double dy = std::get<1>(current_pose) - std::get<1>(virtual_pose);

      // Check constraints
      if(!Utils::isPositionInMap(current_pose, map))
      {
#ifdef DEBUG
        printf("OUT OF BOUNDS\n");
#endif

        *result_iterations= it;
        *result_pose = current_pose;
        return -2.0;
      }


      //inclusion_bound =
      //pow(2,2)*(sum_d + err_sq)*(2*it + max_iterations) / max_iterations / real_scan.size(); 1125
      //inclusion_bound = pow(2,2) * (sum_d + err_sq) / real_scan.size(); 1142 3436
      //inclusion_bound = pow(2,2) * (sum_d + err) / real_scan.size(); 1144 3407
      //inclusion_bound = pow(2,2) * sum_d / real_scan.size(); 1155 3454
      //inclusion_bound = 0.01*sum_d; 1168 3487
      //inclusion_bound = 100*err;

      for (unsigned int d = 0; d < d_v.size(); d++)
        d_v[d] = fabs(d_v[d]);

      sum_d_v = std::accumulate(d_v.begin(), d_v.end(), 0.0);

#ifdef DEBUG
      printf("err = %f\n", err);
      printf("norm_x1 = %f\n", norm_x1);
      printf("sum_d_v = %f\n", sum_d_v);
#endif


      if (pick_min)
      {
        x_es.push_back(x_e);
        y_es.push_back(y_e);
        sum_d_vs.push_back(sum_d_v);
      }

      // Break if translation is negligible
      double eps = 0.0000001;
      if (fabs(x_e) < eps && fabs(y_e) < eps)
        break;
    }

    if (pick_min)
    {
      std::vector<double> crit_v = sum_d_vs;
      unsigned int min_sum_d_idx =
        std::min_element(crit_v.begin(), crit_v.end()) -crit_v.begin();
      sum_d_v = sum_d_vs[min_sum_d_idx];
      double x_tot = std::accumulate(x_es.begin(), x_es.begin()+min_sum_d_idx, 0.0);
      double y_tot = std::accumulate(y_es.begin(), y_es.begin()+min_sum_d_idx, 0.0);

      std::get<0>(*result_pose) = x_tot + std::get<0>(virtual_pose);
      std::get<1>(*result_pose) = y_tot + std::get<1>(virtual_pose);
    }
    else
      *result_pose = current_pose;

    *result_iterations= it;

    // Stop the clock
    std::chrono::high_resolution_clock::time_point end =
      std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> elapsed =
      std::chrono::duration_cast< std::chrono::duration<double> >(end-start);

#ifdef PRINTS
    printf("output pose (%f,%f,%f) [Translation::tff]\n",
      std::get<0>(*result_pose),
      std::get<1>(*result_pose),
      std::get<2>(*result_pose));
#endif

    return sum_d_v / real_scan.size();
  }

  /*****************************************************************************
  */
  static std::pair<double,double> tffCore(
    const std::vector< double >& real_scan,
    const std::vector< double >& virtual_scan,
    const double& current_t,
    const double& inclusion_bound,
    const fftw_plan& r2rp,
    std::vector<double>* d_v,
    double* norm_x1)
  {
    assert(inclusion_bound >= 0);

    std::vector<double> diff;
    Utils::diffScansPerRay(real_scan, virtual_scan, inclusion_bound, &diff, d_v);

    // X1
    std::vector<double> X1 = DFTUtils::getFirstDFTCoefficient(diff, r2rp);

    *norm_x1 = sqrtf(X1[0]*X1[0] + X1[1]*X1[1]);

    // Find the x-wise and y-wise errors
    double t = M_PI + current_t;
    std::vector<double> errors_xy = turnDFTCoeffsIntoErrors(X1, diff.size(), t);

    double x_e = errors_xy[0];
    double y_e = errors_xy[1];

#ifdef DEBUG
    printf("(x_e,y_e) = (%f,%f)\n", x_e, y_e);
#endif

    return std::make_pair(x_e,y_e);
  }

  /*****************************************************************************
  */
  static std::vector<double> turnDFTCoeffsIntoErrors(
    const std::vector<double>& dft_coeff,
    const int& num_valid_rays,
    const double& starting_angle)
  {
    double x_err = 0.0;
    double y_err = 0.0;

    if (num_valid_rays > 0)
    {
      // The error in the x- direction
      x_err = 1.0 / num_valid_rays *
        (-dft_coeff[0] * cos(starting_angle)
         -dft_coeff[1] * sin(starting_angle));

      // The error in the y- direction
      y_err = 1.0 / num_valid_rays *
        (-dft_coeff[0] * sin(starting_angle)
         +dft_coeff[1] * cos(starting_angle));
    }

    std::vector<double> errors;
    errors.push_back(x_err);
    errors.push_back(y_err);

    return errors;
  }

};


// -----------------------------------------------------------------------------
class Rotation
{
public:

  /*****************************************************************************
  */
  static double angleById(const unsigned int& rotation_id,
    const unsigned int scan_size)
  {
    double dt = 2*M_PI*rotation_id / scan_size;

    Utils::wrapAngle(&dt);

    return dt;
  }


  /*****************************************************************************
  */
  static std::vector<double> fmt(
    const std::vector< double >& real_scan,
    const std::tuple<double,double,double>& virtual_pose,
    const std::vector< std::pair<double,double> >& map,
    const unsigned int& magnification_size,
    const std::string& batch_or_sequential,
    const fftw_plan& r2rp, const fftw_plan& c2rp,
    std::vector<double>* rc0, std::vector<double>* rc1,
    std::chrono::duration<double>* intersections_time)
  {
    if (batch_or_sequential.compare("batch") == 0)
      return fmt2Batch(real_scan, virtual_pose, map, magnification_size,
        r2rp, c2rp, rc0, rc1, intersections_time);
    else if (batch_or_sequential.compare("sequential") == 0)
      return fmt2Sequential(real_scan, virtual_pose, map, magnification_size,
        rc0, rc1, intersections_time);
    else
    {
      printf("[Rotation::fmt] Use 'batch' or 'sequential' instead \n");
      std::vector<double> dv; dv.push_back(-1.0); return dv;
    }
  }

  /***************************************************************************
   * FMT sequential execution functions (slower)
   */
  static std::vector<double> fmt2Sequential(
    const std::vector< double >& real_scan,
    const std::tuple<double,double,double>& virtual_pose,
    const std::vector< std::pair<double,double> >& map,
    const unsigned int& magnification_size,
    std::vector<double>* rc0, std::vector<double>* rc1,
    std::chrono::duration<double>* intersections_time)
  {
#if defined (PRINTS)
    printf("input pose  (%f,%f,%f) [Rotation::fmt2]\n",
      std::get<0>(virtual_pose),
      std::get<1>(virtual_pose),
      std::get<2>(virtual_pose));
#endif

    rc0->clear();
    rc1->clear();

    std::tuple<double,double,double> zero_pose;
    std::get<0>(zero_pose) = 0.0;
    std::get<1>(zero_pose) = 0.0;
    std::get<2>(zero_pose) = 0.0;


    unsigned int num_virtual_scans = pow(2,magnification_size);
    int virtual_scan_size_max = num_virtual_scans * real_scan.size();

    // Measure the time to find intersections
    std::chrono::high_resolution_clock::time_point int_start =
      std::chrono::high_resolution_clock::now();

    std::vector< std::pair<double,double> > virtual_scan_points =
      X::find(virtual_pose, map, virtual_scan_size_max);

    std::chrono::high_resolution_clock::time_point int_end =
      std::chrono::high_resolution_clock::now();
    *intersections_time =
      std::chrono::duration_cast< std::chrono::duration<double> >(int_end-int_start);

    std::vector<double> virtual_scan_fine;
    Utils::points2scan(virtual_scan_points, virtual_pose, &virtual_scan_fine);

    // Downsample from upper limit:
    // construct the upper-most resolution and downsample from there.
    std::vector< std::vector< double> > virtual_scans(num_virtual_scans);

    for (int i = 0; i < virtual_scan_fine.size(); i++)
    {
      unsigned int k = fmod(i,num_virtual_scans);
      virtual_scans[k].push_back(virtual_scan_fine[i]);
    }

    // Make sure that all virtual scans are equal to the real scan in terms of
    // size
    for (unsigned int i = 0; i < virtual_scans.size(); i++)
      assert(virtual_scans[i].size() == real_scan.size());

    // The real scan's (the original) angle increment
    double ang_inc = 2*M_PI / real_scan.size();
    double mul = 1.0 / num_virtual_scans;


    std::vector<double> orientations;
    std::vector<double> snrs;
    std::vector<double> fahms;
    std::vector<double> pds;

    for (unsigned int a = 0; a < num_virtual_scans; a++)
    {
      double angle = 0.0;
      double snr = 0.0;
      double fahm = 0.0;
      double pd = 0.0;

      fmt1Sequential(real_scan, virtual_scans[a], &angle, &snr, &fahm, &pd);

      double ornt_a = -angle + a*mul*ang_inc;
      Utils::wrapAngle(&ornt_a);

      orientations.push_back(ornt_a);
      snrs.push_back(snr);
      fahms.push_back(fahm);
      pds.push_back(pd);

#if defined (DEBUG)
      printf("a = %u\n", a);
      printf("angle to out = %f\n", std::get<2>(virtual_pose) + ornt_a);
      printf("snr = %.10f\n", snr);
      printf("fahm = %f\n", fahm);
      printf("pd = %.20f\n", pd);
#endif
    }

    // Select some of all the angles based on criteria enforced by rankFMTOutput
    std::vector<unsigned int> optimal_ids =
      rankFMTOutput(snrs, fahms, pds, 3, magnification_size, 0.00001);

    std::vector<double> angles;
    for (unsigned int i = 0; i < optimal_ids.size(); i++)
    {
      double angle = orientations[optimal_ids[i]];
      Utils::wrapAngle(&angle);
      angles.push_back(angle);

      rc0->push_back(pds[optimal_ids[i]]);
      rc1->push_back(snrs[optimal_ids[i]] / fahms[optimal_ids[i]]);
    }

#if defined (TIMES)
    printf("%f [Rotation::fmt2]\n", elapsed.count());
#endif

#if defined (PRINTS)
    for (unsigned int i = 0; i < angles.size(); i++)
    {
      printf("cand. poses (%f,%f,%f) [Rotation::fmt2]\n",
        std::get<0>(virtual_pose),
        std::get<1>(virtual_pose),
        std::get<2>(virtual_pose)+angles[i]);
    }
#endif

    return angles;
  }

  /*****************************************************************************
  */
  static void fmt1Sequential(
    const std::vector< double >& real_scan,
    const std::vector< double >& virtual_scan,
    double* angle, double* snr, double* fahm, double* pd)
  {
    std::vector<double> q_0;
    unsigned int q_0_max_id;
    fmt0Sequential(real_scan, virtual_scan, &q_0, &q_0_max_id);


    // Calculate angle -----------------------------------------------------------
    *angle = static_cast<double>(
      (real_scan.size()-q_0_max_id))/(real_scan.size())*2*M_PI;
    Utils::wrapAngle(angle);

    // Calculate SNR -------------------------------------------------------------
    std::vector<double> q_0_background = q_0;
    q_0_background.erase(q_0_background.begin() + q_0_max_id);

    std::pair<double,double> q_0_mmnts = Utils::vectorStatistics(q_0_background);

    *snr = fabs((q_0[q_0_max_id] - q_0_mmnts.first)) / q_0_mmnts.second;

    // Calculate FAHM ------------------------------------------------------------
    unsigned int count = 0;
    for (unsigned int i = 0; i < q_0.size(); i++)
    {
      if (q_0[i] >= 0.5 * q_0[q_0_max_id])
        count++;
    }

    *fahm = static_cast<double>(count) / q_0.size();

    // Calculate PD --------------------------------------------------------------
    std::vector<double> q_ss;
    unsigned int q_ss_max_id;
    fmt0Sequential(real_scan, real_scan, &q_ss, &q_ss_max_id);

    std::vector<double> q_rr;
    unsigned int q_rr_max_id;
    fmt0Sequential(virtual_scan, virtual_scan, &q_rr, &q_rr_max_id);

    *pd = 2*q_0[q_0_max_id] / (q_ss[q_ss_max_id] + q_rr[q_rr_max_id]);
  }

  /*****************************************************************************
  */
  static void fmt0Sequential(
    const std::vector< double >& real_scan,
    const std::vector< double >& virtual_scan,
    std::vector<double>* q_0,
    unsigned int* q_0_max_id)
  {
    assert(real_scan.size() == virtual_scan.size());

    // Find fft of real scan
    std::vector<double> fft_real = DFTUtils::dft(real_scan);
    //DFTUtils::fftshift(&fft_real);

    // Find fft of virtual scan
    std::vector<double> fft_virtual = DFTUtils::dft(virtual_scan);
    //DFTUtils::fftshift(&fft_virtual);

    // fft_real is in halfcomplex format; fft_real_coeffs is in normal format
    // (you get the full complex transform)
    std::vector< std::pair<double, double> > fft_real_coeffs =
      DFTUtils::getDFTCoefficientsPairs(fft_real);
    std::vector< std::pair<double, double> > fft_virtual_coeffs =
      DFTUtils::getDFTCoefficientsPairs(fft_virtual);

    // Find conjugates of real coefficients
    std::vector< std::pair<double, double> > fft_real_coeffs_conj =
      Utils::conjugate(fft_real_coeffs);

    // The numerator of Q_0
    std::vector< std::pair<double, double> > numerator =
      Utils::innerProductComplex(fft_real_coeffs_conj, fft_virtual_coeffs);

    // The denominator of Q_0
    double denominator =
      Utils::norm2(fft_real_coeffs) * Utils::norm2(fft_virtual_coeffs);

    /*
       for (int i = 0; i < numerator.size(); i++)
       {
       numerator[i].first /= denominator;
       numerator[i].second /= denominator;
       }
       */

    std::vector< std::pair<double, double> > Q_0 = numerator;

    *q_0 = DFTUtils::idft(Q_0);

    *q_0_max_id = std::max_element(q_0->begin(), q_0->end()) - q_0->begin();
  }

  /*****************************************************************************
  */
  static void fmt0AutoSequential(
    const std::vector< double >& real_scan,
    std::vector<double>* q_0,
    unsigned int* q_0_max_id)
  {
    // Find fft of real scan
    std::vector<double> fft_real = DFTUtils::dft(real_scan);

    std::vector< std::pair<double, double> > fft_real_coeffs =
      DFTUtils::getDFTCoefficientsPairs(fft_real);

    // Find conjugates of real coefficients
    std::vector< std::pair<double, double> > fft_real_coeffs_conj =
      Utils::conjugate(fft_real_coeffs);

    // The numerator of Q_0
    std::vector< std::pair<double, double> > numerator =
      Utils::innerProductComplex(fft_real_coeffs_conj, fft_real_coeffs);

    std::vector< std::pair<double, double> > Q_0 = numerator;

    *q_0 = DFTUtils::idft(Q_0);

    *q_0_max_id = std::max_element(q_0->begin(), q_0->end()) - q_0->begin();
  }

  /***************************************************************************
   * FMT batch execution functions (faster)
   */
  static std::vector<double> fmt2Batch(
    const std::vector< double >& real_scan,
    const std::tuple<double,double,double>& virtual_pose,
    const std::vector< std::pair<double,double> >& map,
    const unsigned int& magnification_size,
    const fftw_plan& r2rp, const fftw_plan& c2rp,
    std::vector<double>* rc0, std::vector<double>* rc1,
    std::chrono::duration<double>* intersections_time)
  {
#if defined(PRINTS)
    printf("input pose  (%f,%f,%f) [Rotation::fmt2]\n",
      std::get<0>(virtual_pose),
      std::get<1>(virtual_pose),
      std::get<2>(virtual_pose));
#endif

    rc0->clear();
    rc1->clear();

    std::tuple<double,double,double> zero_pose;
    std::get<0>(zero_pose) = 0.0;
    std::get<1>(zero_pose) = 0.0;
    std::get<2>(zero_pose) = 0.0;


    unsigned int num_virtual_scans = pow(2,magnification_size);
    int virtual_scan_size_max = num_virtual_scans * real_scan.size();

    // Measure the time to find intersections
    std::chrono::high_resolution_clock::time_point int_start =
      std::chrono::high_resolution_clock::now();

    std::vector< std::pair<double,double> > virtual_scan_points =
      X::find(virtual_pose, map, virtual_scan_size_max);

    std::chrono::high_resolution_clock::time_point int_end =
      std::chrono::high_resolution_clock::now();
    *intersections_time =
      std::chrono::duration_cast< std::chrono::duration<double> >(int_end-int_start);

    std::vector<double> virtual_scan_fine;
    Utils::points2scan(virtual_scan_points, virtual_pose, &virtual_scan_fine);

    // Downsample from upper limit:
    // construct the upper-most resolution and downsample from there.
    std::vector< std::vector< double> > virtual_scans(num_virtual_scans);

    for (int i = 0; i < virtual_scan_fine.size(); i++)
    {
      unsigned int k = fmod(i,num_virtual_scans);
      virtual_scans[k].push_back(virtual_scan_fine[i]);
    }

    // Make sure that all virtual scans are equal to the real scan in terms of
    // size
    for (unsigned int i = 0; i < virtual_scans.size(); i++)
      assert(virtual_scans[i].size() == real_scan.size());

    // The real scan's (the original) angle increment
    double ang_inc = 2*M_PI / real_scan.size();
    double mul = 1.0 / num_virtual_scans;

    // Compute the angles and metrics of matching the real scan against each and
    // all virtual scans
    std::vector<double> un_angles;
    std::vector<double> snrs;
    std::vector<double> fahms;
    std::vector<double> pds;

    fmt1Batch(real_scan, virtual_scans, r2rp, c2rp,
      &un_angles, &snrs, &fahms, &pds);

    // Correct the angles returned to get the proper pose from which each
    // virtual scan was taken (needed due to over-sampling the map)
    std::vector<double> angles;
    for (unsigned int a = 0; a < num_virtual_scans; a++)
    {
      double angle_a = -un_angles[a] + a*mul*ang_inc;
      Utils::wrapAngle(&angle_a);

      angles.push_back(angle_a);
    }

    // Select some of all the angles based on criteria enforced by rankFMTOutput
    std::vector<unsigned int> optimal_ids =
      rankFMTOutput(snrs, fahms, pds, 3, magnification_size, 0.00001);

    std::vector<double> cand_angles;
    for (unsigned int i = 0; i < optimal_ids.size(); i++)
    {
      cand_angles.push_back(angles[optimal_ids[i]]);

      rc0->push_back(pds[optimal_ids[i]]);
      rc1->push_back(snrs[optimal_ids[i]] / fahms[optimal_ids[i]]);
    }

#if defined (TIMES)
    printf("%f [Rotation::fmt2]\n", elapsed.count());
#endif

#if defined (PRINTS)
    for (unsigned int i = 0; i < cand_angles.size(); i++)
    {
      printf("cand. poses (%f,%f,%f) [Rotation::fmt2]\n",
        std::get<0>(virtual_pose),
        std::get<1>(virtual_pose),
        std::get<2>(virtual_pose)+cand_angles[i]);
    }
#endif

    return cand_angles;
  }

  /*****************************************************************************
  */
  static void fmt1Batch(
    const std::vector< double >& real_scan,
    const std::vector< std::vector< double > >& virtual_scans,
    const fftw_plan& r2rp, const fftw_plan& c2rp,
    std::vector<double>* angles,
    std::vector<double>* snrs,
    std::vector<double>* fahms,
    std::vector<double>* pds)
  {
    std::vector< std::vector<double> > q_0_v;
    std::vector< unsigned int > q_0_max_id_v;
    fmt0Batch(real_scan, virtual_scans, r2rp, c2rp, &q_0_v, &q_0_max_id_v);

    // Calculate PD --------------------------------------------------------------
    std::vector<double> q_ss;
    unsigned int q_ss_max_id;
    fmt0AutoSequential(real_scan, &q_ss, &q_ss_max_id);

    std::vector< std::vector<double> > q_rr_v;
    std::vector< unsigned int > q_rr_max_id_v;
    fmt0AutoBatch(virtual_scans, r2rp, c2rp, &q_rr_v, &q_rr_max_id_v);

    for (unsigned int i = 0; i < virtual_scans.size(); i++)
    {
      // Calculate angle ---------------------------------------------------------
      double angle = static_cast<double>(
        (real_scan.size()-q_0_max_id_v[i]))/(real_scan.size())*2*M_PI;
      Utils::wrapAngle(&angle);

      angles->push_back(angle);

      // Calculate pd ------------------------------------------------------------
      double pd = 2*q_0_v[i][q_0_max_id_v[i]]
        / (q_ss[q_ss_max_id] + q_rr_v[i][q_rr_max_id_v[i]]);
      pds->push_back(pd);

      // Calculate SNR -----------------------------------------------------------
      std::vector<double> q_0_background = q_0_v[i];
      q_0_background.erase(q_0_background.begin() + q_0_max_id_v[i]);

      std::pair<double,double> q_0_mmnts = Utils::vectorStatistics(q_0_background);

      double snr =
        fabs((q_0_v[i][q_0_max_id_v[i]] - q_0_mmnts.first)) / q_0_mmnts.second;
      snrs->push_back(snr);

      // Calculate FAHM ----------------------------------------------------------
      unsigned int count = 0;
      for (unsigned int f = 0; f < q_0_v[i].size(); f++)
      {
        if (q_0_v[i][f] >= 0.5 * q_0_v[i][q_0_max_id_v[i]])
          count++;
      }

      double fahm = static_cast<double>(count) / q_0_v[i].size();
      fahms->push_back(fahm);
    }
  }

  /*****************************************************************************
  */
  static void fmt0Batch(
    const std::vector<double>& real_scan,
    const std::vector< std::vector<double> > & virtual_scans,
    const fftw_plan& r2rp, const fftw_plan& c2rp,
    std::vector< std::vector<double> >* q_0_v,
    std::vector<unsigned int>* q_0_max_id_v)
  {
    assert(virtual_scans.size() > 0);
    assert(real_scan.size() == virtual_scans[0].size());

    // Find fft of real scan
    std::vector<double> fft_real = DFTUtils::dft(real_scan);

    // Find fft of virtual scan
    std::vector< std::vector<double> > fft_virtuals =
      DFTUtils::dftBatch(virtual_scans, r2rp);

    // fft_real is in halfcomplex format; fft_real_coeffs is in normal format
    // (you get the full complex transform)
    std::vector< std::pair<double, double> > fft_real_coeffs =
      DFTUtils::getDFTCoefficientsPairs(fft_real);

    // Find conjugates of real coefficients
    std::vector< std::pair<double, double> > fft_real_coeffs_conj =
      Utils::conjugate(fft_real_coeffs);

    std::vector< std::vector< std::pair<double, double> > > Q_0_v;
    for (unsigned int i = 0; i < virtual_scans.size(); i++)
    {
      std::vector< std::pair<double, double> > fft_virtual_coeffs =
        DFTUtils::getDFTCoefficientsPairs(fft_virtuals[i]);

      // The numerator of Q_0
      std::vector< std::pair<double, double> > numerator =
        Utils::innerProductComplex(fft_real_coeffs_conj, fft_virtual_coeffs);

      /*
      // The denominator of Q_0
      double denominator =
      Utils::norm2(fft_real_coeffs) * Utils::norm2(fft_virtual_coeffs);

      for (int i = 0; i < numerator.size(); i++)
      {
      numerator[i].first /= denominator;
      numerator[i].second /= denominator;
      }
      */

      Q_0_v.push_back(numerator);
    }

    *q_0_v = DFTUtils::idftBatch(Q_0_v, c2rp);

    for (unsigned int i = 0; i < q_0_v->size(); i++)
    {
      unsigned int q_0_max_id =
        std::max_element(q_0_v->at(i).begin(), q_0_v->at(i).end())
        - q_0_v->at(i).begin();

      q_0_max_id_v->push_back(q_0_max_id);
    }
  }

  /*****************************************************************************
  */
  static void fmt0AutoBatch(
    const std::vector< std::vector<double> > & virtual_scans,
    const fftw_plan& r2rp, const fftw_plan& c2rp,
    std::vector< std::vector<double> >* q_0_v,
    std::vector<unsigned int>* q_0_max_id_v)
  {
    assert(virtual_scans.size() > 0);

    // Find fft of virtual scan
    std::vector< std::vector<double> > fft_virtuals =
      DFTUtils::dftBatch(virtual_scans, r2rp);

    std::vector< std::vector< std::pair<double, double> > > Q_0_v;

    for (unsigned int i = 0; i < virtual_scans.size(); i++)
    {
      // Virtual scan dft coefficients
      std::vector< std::pair<double, double> > fft_virtual_coeffs =
        DFTUtils::getDFTCoefficientsPairs(fft_virtuals[i]);

      // Virtual scan dft coefficients conjugates
      std::vector< std::pair<double, double> > fft_virtual_coeffs_conj =
        Utils::conjugate(fft_virtual_coeffs);

      // The numerator of Q_0
      std::vector< std::pair<double, double> > numerator =
        Utils::innerProductComplex(fft_virtual_coeffs_conj, fft_virtual_coeffs);

      Q_0_v.push_back(numerator);
    }

    *q_0_v = DFTUtils::idftBatch(Q_0_v, c2rp);

    for (unsigned int i = 0; i < q_0_v->size(); i++)
    {
      unsigned int q_0_max_id =
        std::max_element(q_0_v->at(i).begin(), q_0_v->at(i).end())
        -q_0_v->at(i).begin();

      q_0_max_id_v->push_back(q_0_max_id);
    }
  }


  /*****************************************************************************
  */
  static std::vector<unsigned int> rankFMTOutput(
    const std::vector<double>& snr,
    const std::vector<double>& fahm,
    const std::vector<double>& pd,
    const unsigned int& method,
    const unsigned int& magnification_size,
    const double& pd_threshold)
  {
    assert (snr.size() == fahm.size());
    assert (fahm.size() == pd.size());
    assert (pd_threshold >= 0);
    assert (method >= 0 && method <= 3);

    // Return the indices of those angles for which criteria are near
    // the maximum criterion
    std::vector<unsigned int> best_ids;

    // Simply the one please
    if (method == 0)
    {
      // What are the criteria for ranking angles?
      std::vector<double> criteria = pd;

      // Identify maximum criterion
      double max_c = *std::max_element(criteria.begin(), criteria.end());

      best_ids.push_back(
        std::max_element(criteria.begin(), criteria.end()) -criteria.begin());

    }

    // The one + those within pd_threshold around it
    if (method == 1)
    {
      // What are the criteria for ranking angles?
      std::vector<double> criteria = pd;

      // Identify maximum criterion
      double max_c = *std::max_element(criteria.begin(), criteria.end());

      for (unsigned int i = 0; i < criteria.size(); i++)
      {
        if (fabs(criteria[i]-max_c) <= pd_threshold)
          best_ids.push_back(i);
      }
    }

    // The one + those within (max critetia - min crtieria)/2
    if (method == 2)
    {
      // What are the criteria for ranking angles?
      std::vector<double> criteria = pd;

      // Identify maximum criterion
      double max_c = *std::max_element(criteria.begin(), criteria.end());
      double min_c = *std::min_element(criteria.begin(), criteria.end());


      for (unsigned int i = 0; i < criteria.size(); i++)
      {
        if (fabs(criteria[i]-max_c) <= (max_c - min_c)/2)
          best_ids.push_back(i);
      }
    }

    // Pick `pick_num_surr` around max criterion every time
    std::set<unsigned int> best_ids_set;
    if (method == 3)
    {
      // What are the criteria for ranking angles?
      std::vector<double> criteria = pd;


      // Identify maximum criterion
      int max_c_idx =
        std::max_element(criteria.begin(), criteria.end()) - criteria.begin();
      double max_c = criteria[max_c_idx];

#if defined (DEBUG)
      printf("best id = %d\n", max_c_idx);
#endif

      int vendalia_method = 1;

      int pick_num_surr = 0;
      if (vendalia_method == 0)
      {
        pick_num_surr = pow(2,magnification_size) / pow(2,3);
        if (pick_num_surr == 0)
          pick_num_surr = 1;
      }
      if (vendalia_method == 1)
        pick_num_surr = pow(2,7) / pow(2,magnification_size);
      if (vendalia_method == 2)
        pick_num_surr = 4;


      for (int i =  -pick_num_surr + max_c_idx;
        i <= +pick_num_surr + max_c_idx; i++)
      {
        int k = i;

        while(k < 0)
          k += criteria.size();

        while (k > criteria.size())
          k -= criteria.size();

        if (k == criteria.size())
          k = 0;

#if defined (DEBUG)
        printf("k = %d\n", k);
#endif
        best_ids_set.insert(k);
      }

      /*
         for (unsigned int i = 0; i < criteria.size(); i++)
         {
         if (fabs(criteria[i]-max_c) <= pd_threshold)
         best_ids_set.insert(i);
         }
         */

      for (std::set<unsigned int>::iterator it = best_ids_set.begin();
        it != best_ids_set.end(); it++) best_ids.push_back(*it);
    }

#if defined (DEBUG)
    printf("BEST IDS = [");
    for (unsigned int i = 0; i < best_ids.size(); i++)
      printf("%u ", best_ids[i]);

    printf("]\n");
#endif

    return best_ids;
  }

};


// -----------------------------------------------------------------------------
class Match
{
  public:

  /*****************************************************************************
  */
  static bool canGiveNoMore(
    const std::vector<double>& xs,
    const std::vector<double>& ys,
    const std::vector<double>& ts,
    const double& xy_eps,
    const double& t_eps)
  {
    assert(xs.size() == ys.size());

    unsigned int sz = xs.size();
    bool xy_converged = false;
    bool t_converged = false;

    if (sz < 2)
      return false;
    else
    {
      for (unsigned int i = 2; i < sz; i++)
      {
        if (fabs(ts[sz-1] - ts[sz-i]) < t_eps)
          t_converged = true;

        if (fabs(xs[sz-1] - xs[sz-i]) < xy_eps &&
          fabs(ys[sz-1] - ys[sz-i]) < xy_eps)
          xy_converged = true;

        if (xy_converged && t_converged)
          return true;
      }

      return false;
    }
  }

  /*****************************************************************************
  */
  static void fmtdbh(
    const std::vector< double >& real_scan,
    const std::tuple<double,double,double>& virtual_pose,
    const std::vector< std::pair<double,double> >& map,
    const fftw_plan& r2rp, const fftw_plan& c2rp,
    const input_params& ip, output_params* op,
    std::tuple<double,double,double>* result_pose)
  {
    std::chrono::high_resolution_clock::time_point start =
      std::chrono::high_resolution_clock::now();

    *result_pose = virtual_pose;

    // Maximum counter value means a new recovery attempt
    int min_counter = 0;
    int max_counter = ip.max_counter;
    int counter = min_counter;

    // By a factor of what do you need to over-sample angularly?
    unsigned int min_magnification_size = ip.min_magnification_size;
    unsigned int max_magnification_size = ip.max_magnification_size;
    unsigned int current_magnification_size = min_magnification_size;

    // How many times do I attempt recovery?
    unsigned int num_recoveries = 0;
    unsigned int max_recoveries = ip.max_recoveries;

    // These three vectors hold the trajectory for each iteration
    std::vector<double> xs;
    std::vector<double> ys;
    std::vector<double> ts;

    // Two rotation criteria
    std::vector<double> rc0_v;
    std::vector<double> rc1_v;

    // One translation criterion
    std::vector<double> tc_v;

    std::vector<double> dxys;
    std::chrono::duration<double> intersections_time;

    // The best candidate angle found at each iterations is stored and made a
    // candidate each time. Its criterion is its translation criterion after
    // ni-1 translations
    double best_cand_angle = 0.0;
    double best_min_tc = 100000.0;

    // A lock for going overdrive when the rotation criterion is near-excellent
    bool up_lock = false;
    int total_iterations = 0;
    int num_iterations = 0;

    while (current_magnification_size <= max_magnification_size)
    {
#if defined (DEBUG)
      printf("current_magnification_size = %d ---\n", current_magnification_size);
      printf("counter                    = %d ---\n", counter);
#endif
      bool l2_recovery = false;

      // -------------------------------------------------------------------------
      // -------------------------------------------------------------------------
      // ------------------ Rotation correction phase ----------------------------
      std::vector<double> rc0;
      std::vector<double> rc1;
      std::vector<double> cand_angles;

#if (defined TIMES) || (defined LOGS)
      std::chrono::high_resolution_clock::time_point start_rotation =
        std::chrono::high_resolution_clock::now();
#endif

      cand_angles = Rotation::fmt(real_scan, *result_pose, map,
        current_magnification_size, "batch", r2rp, c2rp,
        &rc0, &rc1, &intersections_time);

#if (defined TIMES) || (defined LOGS)
      std::chrono::high_resolution_clock::time_point end_rotation =
        std::chrono::high_resolution_clock::now();

      op->rotation_times += std::chrono::duration_cast<
        std::chrono::duration<double> >(end_rotation-start_rotation).count();

      op->intersections_times += intersections_time.count();
#endif

      bool ca_exists = false;
      for (unsigned int i = 0; i < cand_angles.size(); i++)
      {
        if (cand_angles[i] == best_cand_angle)
        {
          ca_exists = true;
          break;
        }
      }
      if (!ca_exists)
        cand_angles.push_back(best_cand_angle);

      // ------------------ Candidate angles sifting -----------------------------
      unsigned int min_tc_idx = 0;
      if (cand_angles.size() > 1)
      {
        std::vector<double> tcs_sift;
        for (unsigned int ca = 0; ca < cand_angles.size(); ca++)
        {
          // How many test iterations?
          unsigned int ni = 2;
          int tr_i = 0;

          std::tuple<double,double,double> cand_pose = *result_pose;
          std::get<2>(cand_pose) += cand_angles[ca];

#if (defined TIMES) || (defined LOGS)
          std::chrono::high_resolution_clock::time_point start_translation =
            std::chrono::high_resolution_clock::now();
#endif

          double tc = Translation::tff(real_scan, cand_pose, map,
            ni, ip.xy_bound, false, r2rp, &tr_i, &intersections_time, &cand_pose);

#if (defined TIMES) || (defined LOGS)
          std::chrono::high_resolution_clock::time_point end_translation =
            std::chrono::high_resolution_clock::now();

          op->translation_times += std::chrono::duration_cast<
            std::chrono::duration<double> >(end_translation-start_translation).count();

          op->intersections_times += intersections_time.count();
#endif

#if (defined LOGS)
          op->translation_iterations += tr_i;
#endif

          if (tc == -2.0)
            tcs_sift.push_back(100000000.0);
          else
            tcs_sift.push_back(tc);
        }

        // The index of the angle with the least translation criterion
        min_tc_idx =
          std::min_element(tcs_sift.begin(), tcs_sift.end()) - tcs_sift.begin();


        // Check if the newly-found angle is the angle with the least
        // translation criterion so far
        if (tcs_sift[min_tc_idx] < best_min_tc)
        {
          best_min_tc = tcs_sift[min_tc_idx];
          best_cand_angle = cand_angles[min_tc_idx];
        }
      }

      rc0_v.push_back(rc0[min_tc_idx]);
      rc1_v.push_back(rc1[min_tc_idx]);

      //if (fabs(best_cand_angle) > ip.t_bound)
        //l2_recovery = true;


      // Update the current orientation estimate with the angle that sports the
      // least translation criterion overall
      std::get<2>(*result_pose) += cand_angles[min_tc_idx];
      Utils::wrapAngle(&std::get<2>(*result_pose));

      // ... and store it
      ts.push_back(std::get<2>(*result_pose));

      // -------------------------------------------------------------------------
      // -------------------------------------------------------------------------
      // ---------------- Translation correction phase ---------------------------
      num_iterations =
        (current_magnification_size+1)*ip.num_iterations;

      int tr_iterations = -1;
      double int_time_trans = 0.0;

#if (defined TIMES) || (defined LOGS)
      std::chrono::high_resolution_clock::time_point start_translation =
        std::chrono::high_resolution_clock::now();
#endif

      double trans_criterion = Translation::tff(real_scan,
        *result_pose, map, num_iterations, ip.xy_bound, false, r2rp,
        &tr_iterations, &intersections_time, result_pose);

#if (defined TIMES) || (defined LOGS)
      std::chrono::high_resolution_clock::time_point end_translation =
        std::chrono::high_resolution_clock::now();

      op->translation_times += std::chrono::duration_cast<
        std::chrono::duration<double> >(end_translation-start_translation).count();

      op->intersections_times += intersections_time.count();
#endif

#if (defined LOGS)
      op->translation_iterations += tr_iterations;
#endif

      tc_v.push_back(trans_criterion);

#if defined (DEBUG)
      printf("rc0 = %f\n", rc0_v.back());
      printf("rc1 = %f\n", rc1_v.back());
      printf("tc  = %f\n", tc_v.back());
#endif

      xs.push_back(std::get<0>(*result_pose));
      ys.push_back(std::get<1>(*result_pose));

#if (defined LOGS)
      std::tuple<double,double,double> traj_i;
      std::get<0>(traj_i) = xs.back();
      std::get<1>(traj_i) = ys.back();
      std::get<2>(traj_i) = ts.back();
      op->trajectory.push_back(traj_i);
#endif


      // ----------------------- Recovery modes ----------------------------------

      // Perilous pose at exterior of map's bounds detected
      if (tc_v.back() == -2.0)
        l2_recovery = true;

      // Do not allow more than `max_counter` iterations per resolution
      if (counter > max_counter)
      {
        counter = 0;
        current_magnification_size++;
      }


      // Recover if need be
      if (l2_recovery)
      {
        if (num_recoveries > max_recoveries)
          break;

        num_recoveries++;
        l2recovery(virtual_pose, map, ip.xy_bound, ip.t_bound, result_pose);

        counter = min_counter;
        current_magnification_size = min_magnification_size;
      }
      else
      {
        counter++;

        // -------------------------- Level-up -------------------------------------
        double xy_eps = 10.1;
        double t_eps = 0.00001;
        if (canGiveNoMore(xs,ys,ts, xy_eps, t_eps) && counter > min_counter)
        {
          current_magnification_size += 1;
          counter = 0;
        }
      }

      total_iterations++;
    }

    std::chrono::high_resolution_clock::time_point end =
      std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> elapsed =
      std::chrono::duration_cast< std::chrono::duration<double> >(end-start);


#if defined (TIMES)
    printf("%f [Match::fmt]\n", elapsed.count());
#endif

    op->exec_time = elapsed.count();
    op->rc = rc0_v.back();
    op->tc = tc_v.back();
#if defined (LOGS)
    op->rotation_iterations = total_iterations;
    op->num_recoveries = num_recoveries;
#endif
  }

  /*****************************************************************************
  */
  static void l2recovery(
    const std::tuple<double,double,double>& input_pose,
    const std::vector< std::pair<double,double> >& map,
    const double& xy_bound, const double& t_bound,
    std::tuple<double,double,double>* output_pose)
  {
#if defined (PRINTS)
    printf("*********************************\n");
    printf("************CAUTION**************\n");
    printf("Level 2 recovery mode activated\n");
    printf("*********************************\n");
#endif

    while(!Utils::generatePose(input_pose, map,
        1*xy_bound, 2*M_PI, 0.0, output_pose));
  }

};

};

#endif
