/*
 * constants.h
 *
 *  Created on: 05/mar/2012
 *      Author: Mladen Mazuran
 */

#ifndef SLAM_CONSTANTS_H_
#define SLAM_CONSTANTS_H_

#include <cmath>

#define FMT_MATHEMATICA

#define SQUARE(x)                           ((x) * (x))
#define DEG_TO_RAD(x)                       ((x) / 180. * M_PI)

/* ========================================================================== */
/*                           Scan Matching Constants                          */
/* ========================================================================== */

#define SM_POINT_DISTANCE_THRESHOLD         0.5
#define SM_CLUSTER_DISTANCE_THRESHOLD       0.6
#define SM_MINIMUM_CLUSTER_SIZE             5

//#define SM_SPLIT_THRESHOLD                  0.04
//#define SM_MERGE_THRESHOLD                  0.05
//#define SM_COLLINEARITY_ANGLE_THRESHOLD     DEG_TO_RAD(10)

#define SM_MINIMUM_FRONTIER_LENGTH          0.2
#define SM_MINIMUM_SEGMENT_LENGTH           0.2

#define SM_POINT_DISTANCE_THRESHOLD_2       SQUARE(SM_POINT_DISTANCE_THRESHOLD)
#define SM_CLUSTER_DISTANCE_THRESHOLD_2     SQUARE(SM_CLUSTER_DISTANCE_THRESHOLD)

#define SM_SPLIT_THRESHOLD_2                SQUARE(SM_SPLIT_THRESHOLD)
#define SM_MERGE_THRESHOLD_2                SQUARE(SM_MERGE_THRESHOLD)
#define SM_MINIMUM_FRONTIER_LENGTH_2        SQUARE(SM_MINIMUM_FRONTIER_LENGTH)

#define SM_ICL_MAXIMUM_ITERATIONS           15
#define SM_ICL_CONVERGENCE_THRESHOLD        1e-3

#define SM_MEDIAN_FITTING

/* ========================================================================== */
/*                               SLAM Constants                               */
/* ========================================================================== */

#define SLAM_START_TIME_SKIP                2
#define SLAM_MIN_TEMPORAL_DISPLACEMENT      10
#define SLAM_MIN_SPATIAL_DISPLACEMENT       1
#define SLAM_MIN_ANGULAR_DISPLACEMENT       DEG_TO_RAD(30)

#define SLAM_MAP_THINNING_INTERVAL          30

#define SLAM_COVARIANCE_A1                  0.2
#define SLAM_COVARIANCE_A2                  (0.001*M_PI/180)
#define SLAM_COVARIANCE_A3                  0.05
#define SLAM_COVARIANCE_A4                  (0.0001*180/M_PI)

#define SLAM_MINIMUM_FRONTIER_SIZE          .4


#endif /* SLAM_CONSTANTS_H_ */
