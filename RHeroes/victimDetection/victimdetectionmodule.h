#ifndef VICTIMDETECTIONMODULE_H
#define VICTIMDETECTIONMODULE_H

//#define VICTIM_FINDER_SIMULATOR

#include <QThread>
#include "victimdetectionsimulator.h"
#include "data/victimmessage.h"
#include "data/cameradata.h"
#include "data/buddymessage.h"

#ifndef VICTIM_FINDER_SIMULATOR
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
#endif

namespace VictimDetection{
    class VictimDetectionModule : public QThread
    {
        Q_OBJECT
    public:
        explicit VictimDetectionModule(int robotId, QObject *parent = 0);
        void setStatus(bool isActive);
        void setFPS(int effepiesse);

    signals:
        void sendMessageSignal(const Data::Message &msg);

    public slots:
        void onCameraDataSignal(const Data::CameraData &cameraData, const Data::Pose &pose);
        void updateHSVConfig(double hMin, double sMin, double vMin,
                             double hMax, double sMax, double vMax);

    private:
        bool isThereAVictim(const Data::CameraData &cameraData, const Data::Pose &pose);
#ifndef VICTIM_FINDER_SIMULATOR
        IplImage *QImage2IplImage_8(const QImage *qimg);
        QImage *IplImage2QImage_8(IplImage *iplImg);
        IplImage* convertImageRGBtoHSV(const IplImage *imageRGB);
        bool isSkin(const double& color);
        void saveImgForDebug(IplImage *img);
        double checkEllipses(CvSeq *contoursSeq, IplImage *img);
        double checkConvexity(CvSeq *contoursSeq);
        double checkAreaOnPerimeter(CvSeq *contoursSeq);
#endif
        int robotId;
        double hMin, sMin, vMin, hMax, sMax, vMax;
        QImage *imgForGui;
        bool module_activation;
        int frame_per_seconds;
        int frameCount;
        int imageIterationCounter;
        int blobIterationCounter;
        double imageDetectionConfidence;

        //used for the simulator
        VictimDetectionSimulator victimDetectionSimulator;
        bool alreadySent;
    };
}

#endif // VICTIMDETECTIONMODULE_H
