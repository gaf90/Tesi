#-------------------------------------------------
#
# Project created by QtCreator 2012-11-27T13:35:09
#
#-------------------------------------------------

include(./SingleRobot.pro)
include(./slam/dataset/dataset.pro)

QT      += gui widgets
TARGET  = DatasetPlayer
CONFIG  += app_bundle

SOURCES -= robotmain.cpp

QMAKE_CXXFLAGS += -DSLAM_SKIP_DEBUG
