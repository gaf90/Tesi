#include "cameraqlabel.h"

namespace graphics{

CameraQLabel::CameraQLabel(uint robotID) :
    robotID(robotID)
{
}

void CameraQLabel::mousePressEvent(QMouseEvent *ev)
{
    Q_UNUSED(ev);
    emit clicked(robotID);
}

}
