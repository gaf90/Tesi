#-------------------------------------------------
#
# Project created by Cipo drinker of virgins' blood
#
#-------------------------------------------------

SOURCES += \   
    exploration/explorationmodule.cpp \
    exploration/aojrffunction.cpp \
    exploration/evaluationfunction.cpp \
    exploration/mcdmfunction.cpp \
    exploration/semanticbasicfunction.cpp \
    exploration/evaluationrecords.cpp \
    exploration/newfrontierevaluator.cpp \
    exploration/dummyfunction.cpp \
    exploration/criteria/criterion.cpp \
    exploration/criteria/distancecriterion.cpp \
    exploration/criteria/informationgaincriterion.cpp \
    exploration/criteria/batterycriterion.cpp \
    exploration/criteria/mcdmweightreader.cpp \
    exploration/criteria/criterioncomparator.cpp \
    exploration/criteria/weightmatrix.cpp \
    exploration/criteria/userdirectioncriterion.cpp \
    exploration/criteria/userareacriterion.cpp \
    exploration/criteria/transmissionprobabilitycriterion.cpp

HEADERS += \
    exploration/explorationmodule.h \
    exploration/aojrffunction.h \
    exploration/evaluationfunction.h \
    exploration/mcdmfunction.h \
    exploration/semanticbasicfunction.h \
    exploration/evaluationrecords.h \
    exploration/explorationconstants.h \
    exploration/newfrontierevaluator.h \
    exploration/dummyfunction.h \
    exploration/criteria/criterion.h \
    exploration/criteria/distancecriterion.h \
    exploration/criteria/criteriaName.h \
    exploration/criteria/informationgaincriterion.h \
    exploration/criteria/batterycriterion.h \
    exploration/criteria/mcdmweightreader.h \
    exploration/criteria/criterioncomparator.h \
    exploration/criteria/weightmatrix.h \
    exploration/criteria/userdirectioncriterion.h \
    exploration/criteria/userareacriterion.h \
    exploration/criteria/transmissionprobabilitycriterion.h

RESOURCES += \
    MCDMconfig.qrc

OTHER_FILES += \
    MCDMconf.conf

