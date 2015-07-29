#-------------------------------------------------
#
# Project created by QtCreator 2011-11-16T20:38:34
#
#-------------------------------------------------

QT       += core gui network svg

TARGET = KingPoaret
TEMPLATE = app

#QMAKE_CXXFLAGS_DEBUG += -pg
QMAKE_CXXFLAGS_DEBUG += -D__DEBUG__ -Wall -Wextra -W
#QMAKE_LFLAGS_DEBUG += -pg

INCLUDEPATH += libraries/eigen3 libraries/opencv231

win32:LIBS += -lbfd  -liberty -limagehlp
!mac:unix:QMAKE_LFLAGS_DEBUG += -rdynamic

SOURCES += main.cpp\
    data/usarmessage.cpp \
    data/odometrydata.cpp \
    data/pose.cpp \
    middleware/sensor/sensor.cpp \
    middleware/sensor/odometrysensor.cpp \
    middleware/sensor/lasersensor.cpp \
    data/laserdata.cpp \
    data/robotstate.cpp \
    middleware/driver/driver.cpp \
    connection/upiscontroller.cpp \
    connection/usarcontroller.cpp \
    connection/abstractsocketcontroller.cpp \
    graphics/basestationui.cpp \
    data/upismessage.cpp \
    middleware/sensor/camerasensor.cpp \
    data/cameradata.cpp \
    middleware/driver/wheeldriver.cpp \
    data/wheelmessage.cpp \
    connection/wsscontroller.cpp \
    data/wssmessage.cpp \
    data/buddymessage.cpp \
    data/wirelessmessage.cpp \
    data/serializable.cpp \
    middleware/driver/sensordriver.cpp \
    middleware/driver/wirelessdriver.cpp \
    baseStation/mapmanager.cpp \
    data/victimmessage.cpp \
    data/mapmessage.cpp \
    slam/map.cpp \
    data/semanticmapinfomessage.cpp \
    shared/loggerstream.cpp \
    shared/logger.cpp \
    data/infomessage.cpp \
    data/highlevelcommand.cpp \
    data/waypointcommand.cpp \
    data/sendbidsmessage.cpp \
    data/ackmessage.cpp \
    data/moduleactivationmessage.cpp \
    baseStation/basestationcore.cpp \
    graphics/basestationcoreui.cpp \
    graphics/modules/mapwidget.cpp \
    graphics/utils/robotgraphicitem.cpp \
    baseStation/basestationwss.cpp \
    graphics/spawnui.cpp \
    shared/configreader.cpp \
    shared/config.cpp \
    graphics/modules/teleoperationwidget.cpp \
    baseStation/cameramanager.cpp \
    graphics/utils/cameraqlabel.cpp \
    slam/timedpose.cpp \
    slam/geometry/linesegment.cpp \
    slam/geometry/point.cpp \
    baseStation/notificationmanager.cpp \
    graphics/utils/mapgraphicsscene.cpp \
    graphics/utils/waypointmarkergraphicsitem.cpp \
    slam/pathnode.cpp \
    slam/geometry/frontier.cpp \
    baseStation/infomanager.cpp \
    graphics/modules/singlerobotinfowidget.cpp \
    graphics/modules/notificationtableview.cpp \
    baseStation/victimmanager.cpp \
    baseStation/utils/victim.cpp \
    graphics/utils/showvictimdialog.cpp \
    graphics/utils/showvictimwidget.cpp \
    graphics/utils/victimgraphicitem.cpp \
    data/assignitemmessage.cpp \
    data/victimdelectionmessage.cpp \
    data/robotvictimcouplingmessage.cpp \
    data/victimconfirmationmessage.cpp \
    baseStation/configurationmanager.cpp \
    graphics/modules/configurationwindow.cpp \
    connection/rountingcontroller.cpp \
    semanticMapping/room.cpp \
    semanticMapping/semantichandler.cpp \
    semanticMapping/semanticmappingmodule.cpp \
    semanticMapping/geometry/doorlinesegment.cpp \
    semanticMapping/test/clusteraddtest.cpp \
    semanticMapping/test/createroomtest.cpp \
    semanticMapping/test/createsingleroomtest.cpp \
    semanticMapping/test/wallclusterhandlertest.cpp \
    data/distancevectormessage.cpp \
    semanticMapping/test/displayroomtest.cpp \
    graphics/utils/customlabelgraphicitem.cpp \
    data/destinationmessage.cpp \
    graphics/utils/destinationgraphicitem.cpp \
    data/victimdetectionconfmessage.cpp \
    graphics/utils/popupwidget.cpp \
    graphics/utils/exploredirectiongraphicitem.cpp \
    graphics/utils/exploreareagraphicitem.cpp \
    data/airdrivemessage.cpp \
    graphics/modules/airteleoperationwidget.cpp \
    slam/geometry/uncertainlinesegment.cpp \
    semanticMapping/semanticData/topologicalmap.cpp \
    semanticMapping/semanticData/node.cpp \
    data/errornotificationmessage.cpp \
    baseStation/utils/messagepriorityhandler.cpp \
    graphics/utils/notificationgraphicsitem.cpp \
    graphics/utils/dangergraphicsitem.cpp

win32:SOURCES += \
    shared/asprintf.c \
    shared/winbacktrace.c

HEADERS  += \
    data/message.h \
    data/usarmessage.h \
    middleware/driver/driver.h \
    middleware/sensor/sensor.h \
    shared/constants.h \
    shared/utilities.h \
    data/odometrydata.h \
    data/pose.h \
    middleware/sensor/odometrysensor.h \
    middleware/sensor/lasersensor.h \
    data/laserdata.h \
    data/robotstate.h \
    connection/upiscontroller.h \
    connection/abstractsocketcontroller.h \
    graphics/basestationui.h \
    connection/usarcontroller.h \
    data/upismessage.h \
    middleware/sensor/camerasensor.h \
    data/cameradata.h \
    middleware/driver/wheeldriver.h \
    data/wheelmessage.h \
    connection/wsscontroller.h \
    data/wssmessage.h \
    data/buddymessage.h \
    data/wirelessmessage.h \
    data/serializable.h \
    middleware/driver/sensordriver.h \
    middleware/driver/wirelessdriver.h \
    baseStation/mapmanager.h \
    data/victimmessage.h \
    data/mapmessage.h \
    slam/map.h \
    data/semanticmapinfomessage.h \
    shared/loggerstream.h \
    shared/logger.h \
    data/infomessage.h \
    data/highlevelcommand.h \
    data/waypointcommand.h \
    data/sendbidsmessage.h \
    data/askbidsmessage.h \
    data/ackmessage.h \
    data/moduleactivationmessage.h \
    baseStation/basestationcore.h \
    graphics/basestationcoreui.h \
    graphics/modules/mapwidget.h \
    graphics/utils/robotgraphicitem.h \
    baseStation/graphicparams.h \
    baseStation/basestationwss.h \
    graphics/spawnui.h \
    shared/configreader.h \
    shared/config.h \
    graphics/modules/teleoperationwidget.h \
    baseStation/cameramanager.h \
    graphics/utils/cameraqlabel.h \
    slam/timedpose.h \
    slam/geometry/linesegment.h \
    slam/geometry/rototranslation.h \
    slam/geometry/point.h \
    slam/geometry/frontier.h \
    baseStation/notificationmanager.h \
    graphics/utils/mapgraphicsscene.h \
    graphics/utils/waypointmarkergraphicsitem.h \
    slam/pathnode.h \
    baseStation/infomanager.h \
    graphics/modules/singlerobotinfowidget.h \
    graphics/modules/notificationtableview.h \
    baseStation/victimmanager.h \
    baseStation/utils/victim.h \
    graphics/utils/showvictimdialog.h \
    graphics/utils/showvictimwidget.h \
    graphics/utils/victimgraphicitem.h \
    data/assignitemmessage.h \
    data/victimdelectionmessage.h \
    data/robotvictimcouplingmessage.h \
    data/victimconfirmationmessage.h \
    baseStation/params.h \
    baseStation/configurationmanager.h \
    graphics/modules/configurationwindow.h \
    connection/rountingcontroller.h \
    semanticMapping/SemanticDef.h \
    semanticMapping/room.h \
    semanticMapping/semantichandler.h \
    semanticMapping/semanticmappingmodule.h \
    semanticMapping/geometry/doorlinesegment.h \
    semanticMapping/test/clusteraddtest.h \
    semanticMapping/test/createroomtest.h \
    semanticMapping/test/createsingleroomtest.h \
    semanticMapping/test/wallclusterhandlertest.h \
    data/distancevectormessage.h \
    semanticMapping/test/displayroomtest.h \
    graphics/utils/customlabelgraphicitem.h \
    data/destinationmessage.h \
    graphics/utils/destinationgraphicitem.h \
    data/victimdetectionconfmessage.h \
    graphics/utils/popupwidget.h \
    graphics/utils/exploredirectiongraphicitem.h \
    graphics/utils/exploreareagraphicitem.h \
    data/airdrivemessage.h \
    graphics/modules/airteleoperationwidget.h \
    slam/geometry/uncertainlinesegment.h \
    semanticMapping/semanticData/topologicalmap.h \
    semanticMapping/semanticData/node.h \
    data/errornotificationmessage.h \
    baseStation/utils/messagepriorityhandler.h \
    graphics/utils/notificationgraphicsitem.h \
    graphics/utils/dangergraphicsitem.h

FORMS    += \
    graphics/basestationui.ui \
    graphics/spawnui.ui \
    graphics/modules/teleoperationwidget.ui \
    graphics/modules/singlerobotinfowidget.ui \
    graphics/utils/showvictimdialog.ui \
    graphics/utils/showvictimwidget.ui \
    graphics/modules/configurationwindow.ui \
    graphics/utils/popupwidget.ui \
    graphics/modules/airteleoperationwidget.ui

OTHER_FILES += \
    Doxyfile \
    poaret.conf

RESOURCES += \
    roboImages/GUIImages.qrc
























