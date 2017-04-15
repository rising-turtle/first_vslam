/* This file is part of RGBDSLAM.
 * 
 * RGBDSLAM is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * RGBDSLAM is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with RGBDSLAM.  If not, see <http://www.gnu.org/licenses/>.
 */


#ifndef GLOBALDEFINITIONS_H
#define GLOBALDEFINITIONS_H

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include <string.h>

//Determines whether or not to process node pairs concurrently


///This file contains the parameters that determine the
///behaviour of the program
typedef pcl::PointXYZRGB point_type;
typedef pcl::PointCloud<point_type> pointcloud_type;

///Use these keypoints/features
extern std::string global_feature_detector_type;//Fast is really fast but the Keypoints are not robust
extern std::string global_feature_extractor_type;

///This influences speed dramatically
extern  int global_adjuster_max_keypoints;
extern  int global_adjuster_min_keypoints;
extern  int global_fast_adjuster_max_iterations;
extern  int global_surf_adjuster_max_iterations;

///Ignorance w.r.t small motion
extern  float global_min_translation_meter;
extern  float global_min_rotation_degree; 

///Maximally this many comparisons per node
///(lower=faster, higher=better loop closing)
extern  unsigned int global_connectivity;
extern  unsigned int global_potential_nodes;

// spatial limitation for Nodes
extern  double global_graph_radius;
extern  int global_graph_size;
extern  int global_bg_graph_threshold;
extern  double global_max_translation_meter;
extern unsigned int global_min_inliers;

// for log

extern  int gl_pf_fid;
extern  int gl_pf_gs;
extern  double gl_pf_fe;
extern  double gl_pf_fm;
extern  double gl_pf_id;
extern  double gl_pf_me;
extern  double gl_pf_slam;
extern  double gl_pf_start;

// for filtering same feature points
extern double gl_feature_dis;
extern double gl_feature_filter_threshold;

#define PF_START(x) x = ::GetTickCount();
#define PF_FETCH(y,x) y += ::GetTickCount()-x;

#endif
