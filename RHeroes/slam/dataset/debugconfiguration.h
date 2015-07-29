/*
 * debugconfiguration.h
 *
 *  Created on: 12/dic/2012
 *      Author: Mladen Mazuran
 */

#ifndef DEBUGCONFIGURATION_H_
#define DEBUGCONFIGURATION_H_

#define SCANMATCHING_TEST 0

#if defined(SCANMATCHING_TEST) && SCANMATCHING_TEST
#   define PLOT_ASSOCIATIONS 0
#else
#   define PLOT_ASSOCIATIONS 0
#endif

#define ODOMETRY_SOURCE 1
#define GROUNDTRUTH_SOURCE 0

#endif /* DEBUGCONFIGURATION_H_ */
