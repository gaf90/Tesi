#-------------------------------------------------
#
# Project created by QtCreator 2011-11-16T20:38:34
#
#-------------------------------------------------

QT       += core network xml

TARGET = Poaret
CONFIG += console qtestlib
CONFIG -= app_bundle
TEMPLATE = app

DEFINES +=  EIGEN_DONT_VECTORIZE
DEFINES +=  EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT


QMAKE_CXXFLAGS += -msse2
QMAKE_CXXFLAGS_DEBUG += -D__DEBUG__ -Wall -Wextra -W
#QMAKE_CXXFLAGS_DEBUG += -D__DEBUG__ -D__TESTPRM__ -Wall -Wextra -W
QMAKE_CXXFLAGS_RELEASE += -O3 -msse -mmmx
!mac:unix:QMAKE_LFLAGS_DEBUG += -rdynamic

INCLUDEPATH += libraries/eigen3 libraries/RandomLib/include libraries/opencv231 libraries/fann/include
LIBSROOT = $$_PRO_FILE_PWD_/libraries/bin
mac:LIBSROOT = $$LIBSROOT/mac
!mac:unix:LIBSROOT = $$LIBSROOT/linux
win32:LIBSROOT = $$LIBSROOT/win32

LIBS += $$LIBSROOT/libRandom.a
mac:LIBS += -lz \
    $$LIBSROOT/libopencv_core.a \
    $$LIBSROOT/libopencv_highgui.a \
    $$LIBSROOT/libopencv_imgproc.a
win32:LIBS += \
    $$LIBSROOT/libopencv_highgui231.a \
    $$LIBSROOT/libopencv_imgproc231.a \
    $$LIBSROOT/libopencv_core231.a
win32:LIBS += -lbfd -liberty -limagehlp

include(data/data.pro)
include(exploration/exploration.pro)
include(middleware/middleware.pro)
include(shared/shared.pro)
include(slam/slam.pro)
include(PRM/PRM.pro)
include(pathPlanner/pathPlanning.pro)
include(coordination/coordination.pro)
include(test/test.pro)
include(testPRM/testPRM.pro)
include(semanticMapping/semanticMapping.pro)

SOURCES += \
    logic/robot.cpp \
    logic/robotcontroller.cpp \
    connection/upiscontroller.cpp \
    connection/usarcontroller.cpp \
    connection/abstractsocketcontroller.cpp \
    connection/wsscontroller.cpp \
    robotmain.cpp \    
    victimDetection/victimdetectionmodule.cpp \    
    victimDetection/victimdetectionsimulator.cpp \
    connection/rountingcontroller.cpp \
    logic/wheelspeeds.cpp \
    logic/inversekinematiccf.cpp \
    logic/inversekinematic.cpp \
    logic/airrobot.cpp \
    logic/obstacleavoidancehandler.cpp \

HEADERS  += \
    logic/robot.h \
    logic/robotcontroller.h \
    connection/upiscontroller.h \
    connection/abstractsocketcontroller.h \
    connection/usarcontroller.h \
    connection/wsscontroller.h \
    victimDetection/victimdetectionmodule.h \        
    victimDetection/victimdetectionsimulator.h \
    connection/rountingcontroller.h \
    logic/wheelspeeds.h \
    logic/inversekinematiccf.h \
    logic/inversekinematic.h \
    logic/airrobot.h \
    logic/obstacleavoidancehandler.h \

OTHER_FILES += \
    Doxyfile \
    poaret.conf

RESOURCES += \
#    logic/neunet.qrc


unix|win32: LIBS += -lfann

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/libraries/bin/win32/ -lfann
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/libraries/bin/win32/ -lfann
else:unix: LIBS += -L$$PWD/libraries/bin/win32/ -lfann

INCLUDEPATH += $$PWD/libraries/fann
DEPENDPATH += $$PWD/libraries/fann/
