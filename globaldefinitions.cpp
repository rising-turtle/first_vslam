#include "globaldefinitions.h"
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

//######################## GLOBAL CONFIGURATION #############################
//This file serves as a central place for configuration. This should be 
//replaced by a configuration facility using the parameter server or config files

///Use these keypoints/features
std::string global_feature_detector_type =  "SURF";
std::string global_feature_extractor_type = "SURF";
///Identify like this in the ros communication network

///This influences speed and quality dramatically
 int global_adjuster_max_keypoints = 1800;
 int global_adjuster_min_keypoints = 800;//1000;
 int global_fast_adjuster_max_iterations = 10;
 int global_surf_adjuster_max_iterations = 5; //may slow down initially
///Ignorance w.r.t small motion (reduces redundancy)
 float global_min_translation_meter = 0.1;//0.04;//0.1;
 float global_min_rotation_degree = 5; 

///Maximally this many comparisons per node
///(lower=faster, higher=better loop closing)
 unsigned int global_connectivity = 10;//10;
 unsigned int global_potential_nodes = 20;

 // spatial limitation for Nodes
 double global_graph_radius = 2;
 int global_graph_size = 50;
 int global_bg_graph_threshold=10;
 double global_max_translation_meter=8;
 unsigned int global_min_inliers = 10;
 // for log
 int gl_pf_fid=0;
 int gl_pf_gs=0;
 double gl_pf_fe=0;
 double gl_pf_fm=0;
 double gl_pf_id=0;
 double gl_pf_me=0;
 double gl_pf_slam=0;
 double gl_pf_start = 0;


 // for filtering same feature points
 double gl_feature_dis=0.1; // 0.1m
 double gl_feature_filter_threshold=0.04; // 0.2m*0.2m