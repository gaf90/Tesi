#include "inversekinematicnn.h"
#include <QFile>
#include <QResource>
#include <QDir>
#include <QTemporaryFile>

InverseKinematicNN::InverseKinematicNN()
{
    QString tempName = QDir::tempPath() + QDir::separator() + "tmpNet.net";
    QFile *res = new QFile(":/neunet.net"), *tmpFile = new QFile(tempName);
    res->open(QIODevice::ReadOnly);
    tmpFile->open(QIODevice::WriteOnly);
    QByteArray candio = QString(res->readAll()).replace('\r', "").toAscii();
    tmpFile->write(candio);
    tmpFile->close();
    nn = fann_create_from_file(tempName.toLatin1().data());
    tmpFile->remove();
    delete res;
    delete tmpFile;
//    QString tempName = QDir::tempPath() + QDir::separator() + "tmpNet.net";
//    QFile *res = new QFile(":/neunet.net"), *tmpFile = new QFile(tempName);
//    tmpFile->write(QString(res->readAll()).replace('\r', "").toLatin1());
//    tmpFile->close();
//    nn = fann_create_from_file(tempName.toLatin1().data());
//    tmpFile->remove();
//    delete res;
//    delete tmpFile;
//    nn = fann_create_from_file("C:\\Users\\Ricky\\Documents\\Development\\QtProjects\\RHeroes\\logic\\invKin_double.net");
//    nn = fann_create_from_file("/home/calt/Scrivania/megacode/RHeroes/logic/invKin_double.net");
}

InverseKinematicNN::~InverseKinematicNN()
{
    fann_destroy(nn);
}

const WheelSpeeds InverseKinematicNN::computeSpeeds(const Data::Pose &destPose) const
{
    //variables declaration
    fann_type input[3];
    fann_type *output;
    ldbg << "NN input " << destPose << endl;
    //preparing the neural network input
    input[0] = destPose.getX();
    input[1] = destPose.getY();
    input[2] = destPose.getTheta();
    //executing the forward propagation
    output = fann_run(nn, input);
    //creating the output
    return WheelSpeeds(setMinimumSpeed(output[1]*SAFETY_PERC), setMinimumSpeed(output[0]*SAFETY_PERC));
}
