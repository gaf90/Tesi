#define VICTIM_FINDER_SIMULATOR
//#define SAVE_IMG_FOR_DEBUG

#define CONTOUR_LENGTH 20
#define MAX_CONVEXITY_DEFECT 2 //messo a caso, da guardare
#define MAX_ELLIPSE_ECCENTRICITY 8
#define MID_ELLIPSE_ECCENTRICITY 6.5
#define MAX_AREA_ON_PERIMETER 1.15
#define MIN_DETECTION_CONFIDENCE_LEVEL 0.6
#define MIN_CONTOURAREA_ON_ELLIPSEAREA 0.5
#define MAX_CONTOURAREA_ON_ELLIPSEAREA 1.2
#define MIN_ELLIPSE_AXIS_ALLOWED 50

#define WEIGHT_ELLIPSE 0.5
#define WEIGHT_CONVEXITY 0
#define WEIGHT_AREAONPERIMETER 0.5

#define MOTION_DETECTION_MODULE_DISABLED
//#define I_WANT_RECT // Do you want rectangles on detected images?

#include "victimdetectionmodule.h"
#include "shared/utilities.h"
#include "shared/config.h"
#include "data/wirelessmessage.h"

#ifndef VICTIM_FINDER_SIMULATOR
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
#endif

namespace VictimDetection{
VictimDetectionModule::VictimDetectionModule(int aRobotId, QObject *parent) :
    QThread(parent), robotId(aRobotId), imgForGui(NULL)
{
    alreadySent = false;
    // HSV thresholding initialization
    // == from poaret.conf
//    hMin = Config::HMin; sMin = Config::SMin; vMin = Config::VMin;
//    hMax = Config::HMax; sMax = Config::SMax; vMax = Config::VMax;

    // == hardcoded
    hMin = 10; sMin = 40; vMin = 80;
    hMax = 20; sMax = 200; vMax = 255;
    module_activation = true;
    frame_per_seconds = 2;
    frameCount = 1;
    imageIterationCounter = 0;
}

void VictimDetectionModule::setStatus(bool isActive)
{
    module_activation = isActive;
}

void VictimDetectionModule::setFPS(int effepiesse)
{
    frame_per_seconds = effepiesse;
}

void VictimDetectionModule::onCameraDataSignal(const Data::CameraData &cameraData, const Data::Pose &pose)
{
    if(!module_activation) return;
    if (frameCount < frame_per_seconds)
    {
        frameCount += 1;
        return;
    }
    else
    {
        frameCount = 1;
    }

    if(isThereAVictim(cameraData, pose))
    {
#ifndef VICTIM_FINDER_SIMULATOR
        Data::VictimMessage victimMessage(robotNameFromIndex(robotId), *imgForGui, imageDetectionConfidence, Data::Pose(pose));
#else
        Data::VictimMessage victimMessage(robotNameFromIndex(robotId), QImage(cameraData.getFrame()), 1, Data::Pose(pose));
#endif
        Data::BuddyMessage *buddy = new Data::BuddyMessage(robotNameFromIndex(robotId), &victimMessage);
        Data::WirelessMessage message(buddy, Data::WirelessMessage::MessageExchange);

        //Call wirelessDriver->onDriverMessage
        emit sendMessageSignal(message);

        if(imgForGui != NULL)
        {
            delete imgForGui;
            imgForGui = NULL;
        }
        delete buddy;
        buddy = NULL;
    }
}

void VictimDetectionModule::updateHSVConfig(double hMin, double sMin, double vMin,
                                            double hMax, double sMax, double vMax)
{
    this->hMin = hMin;   this->sMin = sMin;   this->vMin = vMin;
    this->hMax = hMax;   this->sMax = sMax;   this->vMax = vMax;
    ldbg << endl;
    ldbg << "victimDetection: Mins H= "<< this->hMin << " S= " << this->sMin << " V= " << this->vMin << endl;
    ldbg << "victimDetection: Maxs H= "<< this->hMax << " S= " << this->sMax << " V= " << this->vMax << endl;
    ldbg << endl;
}

bool VictimDetectionModule::isThereAVictim(const Data::CameraData &cameraData, const Data::Pose &pose)
{
#ifdef VICTIM_FINDER_SIMULATOR
    return victimDetectionSimulator.isThereAVictim(pose);
}
#else
    Q_UNUSED(pose);
    //Loading image from camera: the image is stored using a 24-bit RGB format (8-8-8) QImage::Format_RGB888
    IplImage *image = QImage2IplImage_8(&(cameraData.getFrame()));

    ldbg << "===== Yo, stiamo analizzando l'immagine " << ++imageIterationCounter << " =====" <<endl;

    // DEBUG
    //        cameraFrame.save("cameraFrame.jpg");
    //        QImage *cameraFrameAgain = IplImage2QImage_8(image);
    //        cameraFrameAgain->save("cameraFrameAgain.jpg");

    CvMemStorage *contoursStorage = cvCreateMemStorage(0);
    CvSeq *contours = NULL;
    bool thereIsSkin = false;
    //single blob detection confidence value
    double blobDetectionConfidence = 0;
    double ellipses;
    double convexity;
    double areaOnPerimeter;
    blobIterationCounter = 0;
    imageDetectionConfidence = 0;
    CvScalar contourColor;

#ifdef I_WANT_RECT
    // ==BoundingRect==
    CvRect theBoundingRect;
    CvPoint theCenterOfTheRect;
    CvPoint RectTopLeft;
    CvPoint RectBottomRight;
#endif

    // query image size
    CvSize sz = cvGetSize(image);

    //SKIN DETECTION INITIALIZATION
    // Allocate a 3 channel (color) 8 byte (depth) image
    IplImage* hsv_image = cvCreateImage( sz, 8, 3);
    // Allocate a 1 channel (monochrome) byte image
    IplImage* hsv_in_range = cvCreateImage( sz, 8, 1);
    IplImage* hsv_smooth = cvCreateImage( sz, 8, 1);
    // H value between 5 and 35, S value between 0.23 and 0.68 V value 80 and 255. The S value is multiplied by 255
    // for 8 bit images 0<(H, S, V)<255. Originali 5,58,80,0 e 30,173,255,0
    CvScalar  hsv_min = cvScalar(hMin, sMin, vMin, 0);
    CvScalar  hsv_max = cvScalar(hMax, sMax, vMax, 0);

#ifndef MOTION_DETECTION_MODULE_DISABLED
    IplImage *background_image = 0;
    IplImage *background_image_gray;
    // allocate space to store background image
    background_image = cvCreateImage( sz, 8, 3 );
    background_image_gray = cvCreateImage( sz, 8, 1 );
    // capture background image to later subtract
    // from image. Crude motion detection
    background_image = image;
    cvCvtColor(background_image, background_image_gray, CV_BGR2GRAY);
    IplImage* diff_image = cvCreateImage( sz, 8, 1);
    IplImage* bit_image = cvCreateImage( sz, 8, 1);
    IplImage* gray_image = cvCreateImage( sz, 8, 1);

    // MOTION DETECTION
    // Convert Image from RGB to gray
    cvCvtColor(image, gray_image, CV_BGR2GRAY);
    // background substraction (motion detection)
    cvAbsDiff(gray_image, background_image_gray, diff_image); //ovviamente questa per ora viene sempre nera perche' non c'ï¿½ movimento (sto prendendo una singola immagine)
    // "And" the results from skin detection and motion detection
    // KEEP THIS LINE COMMENTED UNTIL THE MOTION DETECTION IS FULLY IMPLEMENTED
    // OTHERWISE THE hsv_smooth IMAGE WILL BE OVERWRITTEN AND NO CONTOURS WILL BE FOUND
    cvAnd(hsv_smooth, bit_image, hsv_smooth, 0);
#endif
    // SKIN DETECTION
    // Convert Image from RGB color space to HSV
    cvCvtColor(image, hsv_image, CV_RGB2HSV);
    // Check if array elements lie between hsv_min and hsv_max array elements
    cvInRangeS(hsv_image, hsv_min, hsv_max, hsv_in_range);
    // smooth the image using CV_MEDIAN - remove noise
    cvSmooth(hsv_in_range, hsv_smooth, CV_MEDIAN, 3, 0, 0, 0 );
    //find contours in the binary image hsv_smooth
    cvFindContours(hsv_smooth, contoursStorage, &contours, sizeof(CvContour), CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE,cvPoint(0,0));

    if(!contours) // Ignore images with empty contours (images with no skin at all) to avoid SIGSEV
    {
        ldbg << "== victimDetection: iteration ID " << imageIterationCounter << "." << blobIterationCounter << " ==" << endl;
        ldbg << "victimDetection: Contours vuoto" << endl;
        thereIsSkin = false; //non serve perché già inizializzata a false, ma per sicurezza... :D
    }
    else //we have found blobs!!
    {
        // Check the size of contours to establish if there's a victim
        while(contours->h_next != NULL)
        {
            ldbg << "=== Starting Image Analysis ===" << endl;
            ldbg << endl << "== victimDetection: iteration ID " << imageIterationCounter << "." << ++blobIterationCounter << " ==" << endl;

            //set the length of the min accepted contour using sense :)
            if(contours->total<CONTOUR_LENGTH)
            {
                ldbg << "victimDetection: size TOO SMALL!: " << contours->total << endl;
                //here thereIsSkin remains FALSE!
            }
            else //contours->total>=CONTOUR_LENGTH
            {
                // control the blob shape basing on ellipse characteristic
                ellipses = checkEllipses(contours, image);
                ldbg << "Ellipses vale: " << ellipses << endl;
                // control the blob shape basing on Area/Perimeter of the blob
                areaOnPerimeter = checkAreaOnPerimeter(contours);
                ldbg << "AreaOnPerimeter vale: " << areaOnPerimeter << endl;
                // control the blob shape basing on convexity features
                //convexity = checkConvexity(contours);
                ldbg << "Convexity vale: " << convexity << endl;

                blobDetectionConfidence = (WEIGHT_ELLIPSE*ellipses +
                                           WEIGHT_CONVEXITY*convexity +
                                           WEIGHT_AREAONPERIMETER*areaOnPerimeter)/
                        (WEIGHT_ELLIPSE+
                         WEIGHT_CONVEXITY+
                         WEIGHT_AREAONPERIMETER);

                //check if the proportion skin/non skin is high enough
                // SHALL WE DO THAT?
                // NOT_IMPLEMENTED

#ifdef I_WANT_RECT
                //Calculates the up-right bounding rectangle of a point set.
                //NOT ROTATED RECTANGLE!
                theBoundingRect = cvBoundingRect(contours,0);
                theCenterOfTheRect.x = theBoundingRect.x + theBoundingRect.width/2;
                theCenterOfTheRect.y = theBoundingRect.y + theBoundingRect.height/2;
                RectTopLeft.x = theBoundingRect.x;
                RectTopLeft.y = theBoundingRect.y;
                RectBottomRight.x = theBoundingRect.x+theBoundingRect.width;
                RectBottomRight.y = theBoundingRect.y+theBoundingRect.height;
                cvDrawRect(image,RectTopLeft,RectBottomRight,cvScalarAll(255));
#endif
                ldbg << "blobDetectionConfidence vale: " << blobDetectionConfidence << endl;

                //set the image detection confidence equal to the highest confidence of all the blobs
                if(blobDetectionConfidence>imageDetectionConfidence)
                {
                    imageDetectionConfidence = blobDetectionConfidence;
                    ldbg << "Detectionconfidence è cresciuto, vale: " << imageDetectionConfidence << endl << endl;
                }

                if(blobDetectionConfidence>=MIN_DETECTION_CONFIDENCE_LEVEL)
                {
                    contourColor = cvScalar(255,0,0,0);
                }
                else
                {
                    contourColor = cvScalar(255,255,255,0);
                }

                cvDrawContours(image,contours,contourColor,contourColor,0,1,8,cvPoint(0,0));

#ifdef SAVE_IMG_FOR_DEBUG
                saveImgForDebug(image);
#endif
            }

            // try with next contour
            contours = contours->h_next;

        } //end while

        // ATTENZIONE: QUESTA è FUORI DAL WHILE PER FARE IL TESTING, QUANDO LA USIAMO EFFETTIVAMENTE MEGLIO
        // METTERLA NEL WHILE IN MODO DA USCIRE AL PRIMO BLOB TROVATO E ALLEGGERIRE TUTTO!
        //enter here only if you find a blob with a high confidence
        if(imageDetectionConfidence>=MIN_DETECTION_CONFIDENCE_LEVEL)
        {
            ldbg << endl;
            ldbg << "Siamo qui: " << imageIterationCounter << "." << blobIterationCounter <<endl;
            ldbg << "imageDetectionConfidence è alta e vale: " << imageDetectionConfidence << endl;
            thereIsSkin = true;
        }

        ldbg << "L'ultimo blob di questa immagine è " << imageIterationCounter << "." << blobIterationCounter << endl;
    }

    if(thereIsSkin == true) imgForGui = IplImage2QImage_8(image);

        //deallocate all the unnecessary images
    if(image != NULL)
    {        
        delete image->imageData; //serve? BOH
        cvReleaseImageHeader(&image); //questa secondo me dovrebbe far crashare tutto ma non lo fa...!
        image = NULL;
    }
    if(hsv_image != NULL)
    {
        delete hsv_image->imageData;
        cvReleaseImageHeader(&hsv_image);
    }
    if(hsv_in_range != NULL)
    {
        delete hsv_in_range->imageData;
        cvReleaseImageHeader(&hsv_in_range);
    }
    if(hsv_smooth != NULL)
    {
        delete hsv_smooth->imageData;
        cvReleaseImageHeader(&hsv_smooth);
    }
    cvReleaseMemStorage(&contoursStorage);

    ldbg << "Ritorno thereIsSkin = " << thereIsSkin << endl;
    return thereIsSkin;
}

//============================DA PROVARE=========================
// skin detection in YCrCb
bool VictimDetectionModule::isSkin(const double& color) {
    using namespace cv;
    // se dovesse funzionare ricorda che la matrice si può salvare per sempre senza crearla ogni volta
    Mat skinCrCbHist = Mat::zeros(Size(256, 256), CV_8UC1);
    ellipse(skinCrCbHist, Point(113, 155.6), Size(23.4, 15.2), 43.0, 0.0, 360.0, Scalar(255, 255, 255), -1);
    Mat input = Mat(Size(1, 1), CV_8UC3, color);
    Mat output;

    cvtColor(input, output, CV_BGR2YCrCb);

    Vec3b ycrcb = output.at<Vec3b>(0, 0);
    return ((skinCrCbHist.at<uchar>(ycrcb[1], ycrcb[2]) > 0));
}

//============================DA PROVARE=========================
// Create a HSV image from the RGB image using the full 8-bits, since OpenCV only allows Hues up to 180 instead of 255.
// ref: "http://cs.haifa.ac.il/hagit/courses/ist/Lectures/Demos/ColorApplet2/t_convert.html"
// Remember to free the generated HSV image.
IplImage* VictimDetectionModule::convertImageRGBtoHSV(const IplImage *imageRGB)
{
    float fR, fG, fB;
    float fH, fS, fV;
    const float FLOAT_TO_BYTE = 255.0f;
    const float BYTE_TO_FLOAT = 1.0f / FLOAT_TO_BYTE;

    // Create a blank HSV image
    IplImage *imageHSV = cvCreateImage(cvGetSize(imageRGB), 8, 3);
    if (!imageHSV || imageRGB->depth != 8 || imageRGB->nChannels != 3) {
        printf("ERROR in convertImageRGBtoHSV()! Bad input image.\n");
        exit(1);
    }

    int h = imageRGB->height;		// Pixel height.
    int w = imageRGB->width;		// Pixel width.
    int rowSizeRGB = imageRGB->widthStep;	// Size of row in bytes, including extra padding.
    char *imRGB = imageRGB->imageData;	// Pointer to the start of the image pixels.
    int rowSizeHSV = imageHSV->widthStep;	// Size of row in bytes, including extra padding.
    char *imHSV = imageHSV->imageData;	// Pointer to the start of the image pixels.
    for (int y=0; y<h; y++) {
        for (int x=0; x<w; x++) {
            // Get the RGB pixel components. NOTE that OpenCV stores RGB pixels in B,G,R order.
            uchar *pRGB = (uchar*)(imRGB + y*rowSizeRGB + x*3);
            int bB = *(uchar*)(pRGB+0);	// Blue component
            int bG = *(uchar*)(pRGB+1);	// Green component
            int bR = *(uchar*)(pRGB+2);	// Red component

            // Convert from 8-bit integers to floats.
            fR = bR * BYTE_TO_FLOAT;
            fG = bG * BYTE_TO_FLOAT;
            fB = bB * BYTE_TO_FLOAT;

            // Convert from RGB to HSV, using float ranges 0.0 to 1.0.
            float fDelta;
            float fMin, fMax;
            int iMax;
            // Get the min and max, but use integer comparisons for slight speedup.
            if (bB < bG) {
                if (bB < bR) {
                    fMin = fB;
                    if (bR > bG) {
                        iMax = bR;
                        fMax = fR;
                    }
                    else {
                        iMax = bG;
                        fMax = fG;
                    }
                }
                else {
                    fMin = fR;
                    fMax = fG;
                    iMax = bG;
                }
            }
            else {
                if (bG < bR) {
                    fMin = fG;
                    if (bB > bR) {
                        fMax = fB;
                        iMax = bB;
                    }
                    else {
                        fMax = fR;
                        iMax = bR;
                    }
                }
                else {
                    fMin = fR;
                    fMax = fB;
                    iMax = bB;
                }
            }
            fDelta = fMax - fMin;
            fV = fMax;				// Value (Brightness).
            if (iMax != 0) {			// Make sure its not pure black.
                fS = fDelta / fMax;		// Saturation.
                float ANGLE_TO_UNIT = 1.0f / (6.0f * fDelta);	// Make the Hues between 0.0 to 1.0 instead of 6.0
                if (iMax == bR) {		// between yellow and magenta.
                    fH = (fG - fB) * ANGLE_TO_UNIT;
                }
                else if (iMax == bG) {		// between cyan and yellow.
                    fH = (2.0f/6.0f) + ( fB - fR ) * ANGLE_TO_UNIT;
                }
                else {				// between magenta and cyan.
                    fH = (4.0f/6.0f) + ( fR - fG ) * ANGLE_TO_UNIT;
                }
                // Wrap outlier Hues around the circle.
                if (fH < 0.0f)
                    fH += 1.0f;
                if (fH >= 1.0f)
                    fH -= 1.0f;
            }
            else {
                // color is pure Black.
                fS = 0;
                fH = 0;	// undefined hue
            }

            // Convert from floats to 8-bit integers.
            int bH = (int)(0.5f + fH * 255.0f);
            int bS = (int)(0.5f + fS * 255.0f);
            int bV = (int)(0.5f + fV * 255.0f);

            // Clip the values to make sure it fits within the 8bits.
            if (bH > 255)
                bH = 255;
            if (bH < 0)
                bH = 0;
            if (bS > 255)
                bS = 255;
            if (bS < 0)
                bS = 0;
            if (bV > 255)
                bV = 255;
            if (bV < 0)
                bV = 0;

            // Set the HSV pixel components.
            uchar *pHSV = (uchar*)(imHSV + y*rowSizeHSV + x*3);
            *(pHSV+0) = bH;		// H component
            *(pHSV+1) = bS;		// S component
            *(pHSV+2) = bV;		// V component
        }
    }
    return imageHSV;
}

// QImage2IplImage and IplImage2QImage originally coming from
// http://umanga.wordpress.com/2010/04/19/how-to-covert-qt-qimage-into-opencv-iplimage-and-wise-versa/

IplImage *VictimDetectionModule::QImage2IplImage_8(const QImage *qimg)
{
    if(qimg->format()==QImage::Format_RGB32)
    {
        IplImage *imgHeader = cvCreateImageHeader(cvSize(qimg->width(), qimg->height()), IPL_DEPTH_8U, 3);
        uchar* newdata = new uchar[qimg->byteCount()];
        for(int i = 0, j = 0; i < qimg->byteCount(); i += 4) {
            newdata[j++] = qimg->bits()[i + 2];
            newdata[j++] = qimg->bits()[i + 1];
            newdata[j++] = qimg->bits()[i + 0];
        }
        imgHeader->imageData = (char *) newdata;
        //imgHeader->imageData = (char*) qimg->bits();
        return imgHeader;
    }
    else if(qimg->format()==QImage::Format_RGB888)
    {
        IplImage *imgHeader = cvCreateImageHeader(cvSize(qimg->width(), qimg->height()), IPL_DEPTH_8U, 3);
        //Do not assign imageData directly. Use SetData().
        //cvSetData(imgHeader,qimg->bits(),qimg->bytesPerLine()); // NON FUNZIONA
        uchar* newdata = new uchar[qimg->byteCount()]; //(uchar*) malloc(sizeof(uchar) * qimg->byteCount());
        memcpy(newdata, qimg->bits(), qimg->byteCount());
        imgHeader->imageData = (char*) newdata;
//        imgHeader->imageData = (char*) qimg->bits();
        return imgHeader;
    }
    return NULL;
}

QImage *VictimDetectionModule::IplImage2QImage_8(IplImage *iplImg)
{
    int img_height = iplImg->height;
    int img_width = iplImg->width;
    int channels = iplImg->nChannels;
    QImage *qimg = new QImage(img_width, img_height, QImage::Format_RGB888);
    char *data = iplImg->imageData;
    for (int current_row = 0; current_row < img_height; current_row++, data += iplImg->widthStep)
    {
        for (int current_column = 0; current_column < img_width; current_column++)
        {
            char r, g, b, a = 0;
            if (channels == 1)
            {
                r = data[current_column * channels];
                g = data[current_column * channels];
                b = data[current_column * channels];
            }
            else if (channels == 3 || channels == 4)
            {
                r = data[current_column * channels];
                g = data[current_column * channels + 1];
                b = data[current_column * channels + 2];
            }

            if (channels == 4)
            {
                a = data[current_column * channels + 3];
                qimg->setPixel(current_column, current_row, qRgba(r, g, b, a));
            }
            else
            {
                qimg->setPixel(current_column, current_row, qRgb(r, g, b));
            }
        }
    }
    return qimg;
}

void VictimDetectionModule::saveImgForDebug(IplImage *img){
    QImage *temp;
    temp = IplImage2QImage_8(img);
    QString filename = QString("robot%1_%2.%3cvEllipseBox.jpg").arg(robotId).arg(imageIterationCounter).arg(blobIterationCounter);
    temp->save(filename);
    delete temp;
    temp = NULL;
}

double VictimDetectionModule::checkEllipses(CvSeq *contoursSeq, IplImage *img)
{
    float ellipseWidth;
    float ellipseHeight;
    float maxAxis;
    float minAxis;
    CvBox2D minAreaRectangle;
    CvMemStorage *minAreaRectStorage = cvCreateMemStorage(0);

    //Finds a rotated rectangle of the minimum area enclosing the input 2D point set.
    minAreaRectangle  = cvMinAreaRect2(contoursSeq, minAreaRectStorage);

    //Draws a simple or thick elliptic arc or fills an ellipse sector.
    cvEllipseBox(img, minAreaRectangle, cvScalar(0,240,230,0));
    ellipseWidth = minAreaRectangle.size.width;
    ellipseHeight = minAreaRectangle.size.height;

    // Do this control to avoid threshold errors in case the ellipse is
    // drawn with a very long vertical axis and a short horizontal one
    if(ellipseWidth>=ellipseHeight)
    {
        maxAxis = ellipseWidth;
        minAxis = ellipseHeight;
    }
    else
    {
        maxAxis = ellipseHeight;
        minAxis = ellipseWidth;
    }

    //QUESTA è MEGLIO??? sembrerebbe di no (anzi peggio)
    //Fits an ellipse around a set of 2D points.
    //The function calculates the ellipse that fits (in a least-squares sense) a set of 2D points best of all
    //It returns the rotated rectangle in which the ellipse is inscribed
    //                    datiEllisse = cvFitEllipse2(contours);
    //                    cvEllipseBox(image, datiEllisse, cvScalar(0,255,230,0));
    //cvDrawContours(copiaImage,contours,cvScalarAll(255),cvScalarAll(255),0,1,8,cvPoint(0,0));

    //DEBUG
    ldbg << "victimDetection: ellipse width " << ellipseWidth << endl;
    ldbg << "victimDetection: ellipse height " << ellipseHeight << endl;
    ldbg << "victimDetection: la \"eccentricità\" (max/min) è " << maxAxis/minAxis << endl;

    // very rough control: since doors, tables and furniture are likely to be perfectly vertical/horizontal
    // exclude ellipses with angle 0 or 90
    if(minAreaRectangle.angle == 0 || minAreaRectangle.angle == 90)
    {
        ldbg << "victimDetection: "<< imageIterationCounter << "." << blobIterationCounter
             << " RIFIUTATA per ellisse_verticale_o_orizzontale" << endl;
        return 0;
    }

    // another rough control: check the ratio of contourArea and ellipseArea
    // it must be close to 1
    double ellipseArea = (ellipseWidth/2)*(ellipseHeight/2)*M_PI;
    double contourArea = cvContourArea(contoursSeq);
    if(MAX_CONTOURAREA_ON_ELLIPSEAREA < contourArea/ellipseArea || contourArea/ellipseArea < MIN_CONTOURAREA_ON_ELLIPSEAREA)
    {
        //in realtà non dovrebbe servire anche il controllo sul maggiore
        //(vorrebbe dire che il contorno è più grosso dell'ellisse, e questo in teoria non dovrebbe succedere)
        //ma tant'è
        ldbg << "victimDetection: "<< imageIterationCounter << "." << blobIterationCounter
             << " RIFIUTATA per contourArea/ellipseArea basso: " << contourArea/ellipseArea << endl;
        return 0;
    }

    //se il minore è maggiore di 50
    //third rough control: if the minor axis is greater then a value
    //the ellipse is too big -> FIRE IT
    if(minAxis > MIN_ELLIPSE_AXIS_ALLOWED)
    {
        ldbg << "victimDetection: "<< imageIterationCounter << "." << blobIterationCounter
             << " RIFIUTATA per troppo grossa: " << minAxis << endl;
        return 0;
    }

    cvReleaseMemStorage(&minAreaRectStorage);

    //check if the contour is long and straight, and so unlikely to be human body part
    //limit values coming from empiric tests
    if(maxAxis/minAxis > MAX_ELLIPSE_ECCENTRICITY)
    {
        ldbg << "victimDetection: "<< imageIterationCounter << "."<< blobIterationCounter
             << " RIFIUTATA per ellisse_lunga_e_stretta" << endl;
        return 0;
    }
    else if(maxAxis/minAxis > MID_ELLIPSE_ECCENTRICITY && maxAxis/minAxis < MAX_ELLIPSE_ECCENTRICITY)
    {
        ldbg << "victimDetection: "<< imageIterationCounter << "."<< blobIterationCounter << " a metà di ellisse_lunga_e_stretta" << endl;
        return 0.5;
    }
    else
    {
        return 1;
    }
}

double VictimDetectionModule::checkAreaOnPerimeter(CvSeq *contoursSeq)
{
    double contourArea;
    double contourPerimeter;

    //Calculates a contour area
    contourArea = cvContourArea(contoursSeq);
    contourPerimeter = cvContourPerimeter(contoursSeq);

    ldbg << "victimDetection: contourArea " << contourArea << endl;
    ldbg << "victimDetection: contourPerimeter " << contourPerimeter << endl;
    ldbg << "victimDetection: il rapporto è " << contourArea/contourPerimeter <<endl;

    if(contourArea/contourPerimeter < MAX_AREA_ON_PERIMETER)
    {
        ldbg << "victimDetection: "<< imageIterationCounter << "."<< blobIterationCounter << " RIFIUTATA per rapporto_lunga_e_stretta" << endl;
        return 0;
    }
    else
    {
        return 1;
    }
}

double VictimDetectionModule::checkConvexity(CvSeq *contoursSeq)
{
    CvSeq *convexHull = NULL;
    CvSeq *convexityDefectsSeq = NULL;

    CvMemStorage *prozacpiu = cvCreateMemStorage(0);
    //check if the contour is convex, try to avoid strange concave contours
    convexHull = cvConvexHull2(contoursSeq);
    convexityDefectsSeq = cvConvexityDefects(contoursSeq,convexHull,prozacpiu);
    if(!convexityDefectsSeq)
    {
        ldbg << "Niente difetti di convessita" << endl;
        return 1;
    }
    else
    {
        int counter = 0;
        ldbg << "puttana eva: " << convexityDefectsSeq->h_next << endl;
        while(convexityDefectsSeq->h_next != NULL) //da sistemare il controllo sulla depth
        {
            CvConvexityDefect *defect = (CvConvexityDefect *)convexityDefectsSeq;
            ldbg << "Difetti di convessità numero: " << convexityDefectsSeq->total << endl;
            ldbg << "Difetti di convessità profondità " << defect->depth << endl;
            if(defect->depth > MAX_CONVEXITY_DEFECT)
            {
                ldbg << "victimDetection: "<< blobIterationCounter << " RIFIUTATA per concavità" << endl;
                return 0;
            }
            counter++;
        }
        //out here we haven't found too high defects, return 1
        ldbg << "Difetti di convessita non rilevanti" << endl;
        return 1;
    }

//    if(cvCheckContourConvexity(contoursSeq) == 0)
//    {
//        ldbg << "victimDetection: "<< iterationCounter << " RIFIUTATA per concavità" << endl << endl;
//        return 0;
//    }
//    else
//    {
//        return 1;
//    }

    cvReleaseMemStorage(&prozacpiu);
    cvReleaseMemStorage(&(convexHull->storage));

}

#endif
}
