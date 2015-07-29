#ifndef GRAPHICPARAMS_H
#define GRAPHICPARAMS_H

/**
* This file is used to store every constant value used in the base station process.
* @author Alain Caltieri
*/

namespace BaseStation{
/* ========================================================================== */
/*                               MAP WIDGET                                   */
/* ========================================================================== */

/**
* Defines the height of the mapViewer component in the Teleoperation UI
*/
#define MAP_WIDGET_HEIGHT_TELEOPERATION 500

/**
* Defines the width of the mapViewer component in the Teleoperation UI
*/
#define MAP_WIDGET_WIDTH_TELEOPERATION 600


/**
* Defines the height of the mapViewer component in the Global UI
*/
#define MAP_WIDGET_HEIGHT_GLOBAL 1000

/**
* Defines the width of the mapViewer component in the Global UI
*/
#define MAP_WIDGET_WIDTH_GLOBAL 1500

/**
* Defines the height of the marker of a robot in the mapWidget
*/
#define ROBOT_MARKER_HEIGHT 3.0

/**
* Defines the width of the marker of a robot in the mapWidget
*/
#define ROBOT_MARKER_WIDTH 2.0

/**
* Defines the width of the marker of a victim in the mapWidget
*/
#define VICTIM_MARKER_WIDTH 2

/**
* Defines the height of the marker of a victim in the mapWidget
*/
#define VICTIM_MARKER_HEIGHT 5

/**
* Defines the initial scale factor of the map. This allows to better draw the map when measures are
* small
*/
#define INITIAL_SCALE_FACTOR 5

/**
* Defines the scale factor for the mapping between the slam module measure and the graphical one.
*/
#define SLAM_SCALE_FACTOR 4

/**
* Defines the Zoom-in increment factor
*/
#define ZOOM_IN_INCREMENT 0.2

/**
* Defines the Zoom-out increment factor
*/
#define ZOOM_OUT_INCREMENT 0.2

//! Maximum zoom in
#define ZOOM_IN_LIMIT 5

//! Maximum zoom out
#define ZOOM_OUT_LIMIT 0.2

/* ========================================================================== */
/*                               SPAWN UI                                     */
/* ========================================================================== */

/**
* Labels used in the spawn UI for the connection process to the simulator.
*/
#define CONNECT "Connect"
#define DISCONNECT "Disconnect"

/* ========================================================================== */
/*                              CAMERAS UI                                    */
/* ========================================================================== */
/**
* The width of a thumbnail view of a robot camera.
*/
#define THUMBNAIL_VIEW_WIDTH 100

#define THUMBNAIL_VIEW_HEIGHT 75

#define FOCUS_CAMERA_WIDTH 320

#define FOCUS_CAMERA_HEIGHT 240


}
#endif // GRAPHICPARAMS_H
