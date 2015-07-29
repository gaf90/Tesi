//#define WSS_TESTING

#ifndef WSS_TESTING
#   include <QtGui/QApplication>
#else
#   include <QCoreApplication>
#   include "wsstester.h"
#endif

//#include "logic/basestation.h"
#include <QTimer>
#include "baseStation/basestationcore.h"
#include "shared/configreader.h"
#include "shared/logger.h"



int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    Config::ConfigReader reader("poaret.conf");
    reader.readFileAndCompileConfigs();
//    Logic::BaseStation w;
    BaseStation::BaseStationCore w2;

    logger.setOutputFormat(Logger::PrependNothing);

    int bs = logger.addFileOutput("WssController.log", false);
    logger.setFilenameFilter(QRegExp(".*wsscontroller.*"), bs);

    int rutting = logger.addFileOutput("Routing.log", false);
    logger.setFilenameFilter(QRegExp(".*rountingcontroller.*"), rutting);

    //int messaging = logger.addFileOutput("MessagingBS.log", false);
   // logger.setFilenameFilter(QRegExp(".*wsscontroller.*"), messaging);


    return a.exec();
}
