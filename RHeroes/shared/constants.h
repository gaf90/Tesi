#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <cmath>
//#include "config.h"

#ifndef MLADEN_DEF
//#   define TESTING_GROUNDTRUTH
#endif

//Diagonal of the Robot
/**
  * Diagonal of the robot.
  */
#define ROBOT_DIAG 0.7
#define ROBOT_WIDTH 0.3


//Parameters for the execution
/**
 * define if we are working with the open loop or
 * with the close one.
 */
#define OPEN_LOOP 0 //if 0, open loop is disabled

/**
  * define the number of times i need to find the same
  * pose in order to determine if the robot is stalling
  */
#define STALL_LIMIT 5

//Tolerances
/**
 * Tolerance for rotations angle
 */

//KENAFFFF


//Compensation factors
/**
 * Compensation factor for the odometry
 * when the robot rotates
 */
#define ANGLE_COMP 0.66 // for rotations
/**
 * Compensation factor for the odometry
 * when the robot moves straight
 */
#define TRASL_COMP 1.04 // for translations

/**
 * Threshold for the angle that must be performed at low speed.
 * If the robot has to rotate for an angle that is less than
 * <b>LOW_SPEED_LIMIT_ANGLE</b> then it does it at <b>LOW_SPEED</b>.
 */
#define LOW_SPEED_LIMIT_ANGLE 10*2

/**
 * Threshold for the linear movement that must be performed at low speed.
 * If the robot has to move for less than <b>LOW_SPEED_LIMIT_TRASL</b> metres
 * then it does it at <b>LOW_SPEED</b>.
 */
#define LOW_SPEED_LIMIT_TRASL 0.5

/**
 * Discount factor for the wheels' speed when the robot is
 * approaching the set point for the movement.
 */
#define SPEED_DECR_FACTOR 0.9 //1

//Battery constants
#define MAX_BATTERY 1200

#define BATTERY_ONE_PERCENT MAX_BATTERY/100

//Fuzzy Speed for the robots
/**
 * Value of the wheels' speed when
 * the robot move slow.
 */
#define LOW_SPEED 0.2*MAX_SPEED_RH //Low speed
/**
 * Value of the wheels' speed when
 * the robot move at normal speed.
 */
#define MED_SPEED 0.5*MAX_SPEED_RH //Medium speed
/**
 * Value of the wheels' speed when
 * the robot move fast.
 */
#define HIGH_SPEED 0.8*MAX_SPEED_RH //high speed
//PRM
#define MIN_SPEED 0.14*MAX_SPEED_RH
//
/**
 * sperimentally, with a wheel speed of 0.2
 * the robot takes 9 seconds to perform 90�
 * so when wheelSpeed = 0.2, it rotates at
 * 10 degree/s. let be speed = 10.
 * th = speed*time =>
 * t = th/speed
 * Notice that 0.2 is the lowest speed we apply to
 * the robot, so this is a worst case estimate
 */
#define ROTATION_SPEED_ESTIMATE_OPOINT2 10 //rotation speed estimate when the robot moves at LOW_SPEED

//Conversion units
/**
 * Milliseconds
 */
#define MILLIS 1000
/**
 * Half round in degrees
 */
#define HALF_ROUND_DEG 180 //in degree
/**
 * Full round in degrees
 */
#define FULL_ROUND_DEG 360 //in degree

/**
 * Half round in radiants
 */
#define HALF_ROUND_RAD M_PI //in degree
/**
 * Full round in radiants
 */
#define FULL_ROUND_RAD M_PI/2 //in degree

//Kenaf
//#define MAX_SPEED_RH 2 //0.9 //1.299 //rad/s

//P3AT
#define MAX_SPEED_RH Config::robotMaxSpeed

//Constants String
/**
 * Time word
 */
#define TIME "Time"
/**
 * String that represents a Command from
 * the User Interface.
 */
#define UI "UI"
/**
 * String to spawn a static robot in USARSim
 */
#define SPAWN_STATIC_R_USAR "INIT {ClassName USARBot.Nao} {Name Robot_666} " //Robot_666
/**
 * String to spawn a static robot in WSS
 */
#define SPAWN_STATIC_R_WSS "INIT {Name Robot_666} {Port 1000666}" //Robot_666

/* ========================================================================== */
/*                         InfoMessage InfoNameValues                         */
/* ========================================================================== */

/**
  * String that identifies the Information Message sent when a robot
  * ends its movement
  */
#define MOVEMENT_END "MovementEnd"

/**
* Constants used for single robot information communication
*/
#define SINGLE_ROBOT_INFO_REQUEST "Gimme Infos!!!"

#define BATTERY_STATUS "BatteryStatus"

#define CONNECTION_INFORMATION "wssConnectionInfo"

#define INFORMATION_REQUEST "Request for informations"

/* ========================================================================== */
/*                             Wireless Constants                             */
/* ========================================================================== */

#define WS_CAMERA_FORMAT    "JPG"
#define WS_CAMERA_QUALITY   80

/* ========================================================================== */
/*                                  Something                                 */
/* ========================================================================== */

#define BASE_STATION_ID 666

/* ========================================================================== */
/*                     COMMUNICATION SETTING CONSTANTS                        */
/* ========================================================================== */

/**
* These constants are used to set/manage the framerate of the single robots
*/
#define FRAMERATE_SETTING_COMMAND "framerate"
#define FRAMERATE_HIGH "8"
#define FRAMERATE_LOW "2"

/* ========================================================================== */
/*                                  MODULE NAME                               */
/* ========================================================================== */

#define EXPLORATION "exploration"

#define SEMANTIC_MAPPING "semantic_mapping"

#define VICTIM_DETECTION "victim_detection"

#define POARET_SLAM_MODULE "slam_module"

/* ========================================================================== */
/*                             ROUTING PARAMS                                 */
/* ========================================================================== */

#define RP_UNDEFINED "und"
#define UNDEFINED_COST 1000

//! The NAN_SIGNAL_STRENGTH label can be assigned to -1 if routing policy is noob, 0 otherwise
#define NAN_SIGNAL_STRENGTH -1
//! Set this label to -1 if noob routing policy is implemented
#define DISTANCE_VECTOR_REQUEST_TIMER 15000

//! How many milliseconds between two requests for Signal Strength.
#define SIGNAL_STRENGTH_REQUEST_TIMER 5000

#define TYPE_DISTANCE_VECTOR "DISTANCE_VECTOR"
#define COUNT_TO_INFINITY_LIMIT 8

/* ========================================================================== */
/*                        OBSTACLE AVOIDANCE PARAMS                           */
/* ========================================================================== */

/**
  * Waiting time to check if there is no more the obstacle in front of the robot (it was another robot now gone away)
  */
#define MIN_OBSTACLE_DETECTED_TIMER 3000
#define MAX_OBSTACLE_DETECTED_TIMER 6000

//Sonar safe distances
/**
 * Safe distance for the front sonar
 */
#define FRONTSONAR_SAFE_DISTANCE 0.286
#define FRONTSONAR_SAFETY_THRESHOLD 0.05

/**
 * Safe distance for the back sonar
 */
#define MIN_SAFE_BACKSONAR_DISTANCE 0.5

#define X_TRANSLATION_SONAR_SICK 0.10 //distance from the center of the robot to the top part

#define RH_RANDOM_ACTION_TIME 15

#define RH_HALF_SQUARE_SIDE 2.5


#endif // CONSTANTS_H
