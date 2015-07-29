#ifndef CAMERAQLABEL_H
#define CAMERAQLABEL_H

#include <QLabel>

namespace graphics{

/**
* This class gives a QLabel that handles the mousePress event!
*/
class CameraQLabel : public QLabel
{
    Q_OBJECT
public:
    explicit CameraQLabel(uint robotID = 0);

signals:
    void clicked(uint robotID);

public slots:

private:
    uint robotID;

    void mousePressEvent(QMouseEvent *ev);

};

}
#endif // CAMERAQLABEL_H
