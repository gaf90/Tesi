#-------------------------------------------------
#
# Project created by Mladen destroyer of source
#
#-------------------------------------------------

SOURCES += \
    slam/map.cpp \
    slam/slammodule.cpp \
    slam/timedpose.cpp \
    slam/pathnode.cpp \
    slam/engine/deterministicslam.cpp \
    slam/engine/odometryonlyslam.cpp \
    slam/legacy/iclslam.cpp \
    slam/geometry/point.cpp \
    slam/geometry/linesegment.cpp \
    slam/geometry/frontier.cpp \
    slam/geometry/uncertainlinesegment.cpp \
    slam/geometry/pointscan.cpp \
    slam/geometry/segmentscan.cpp \
    slam/geometry/scanbox.cpp \
    slam/scanmatching/lookupthresholds.cpp \
    slam/support/resourcepooler.cpp 
        
HEADERS += \
    slam/utilities.h \
    slam/constants.h \
    slam/map.h \
    slam/slammodule.h \
    slam/timedpose.h \
    slam/pathnode.h \
    slam/engine/slamengine.h \
    slam/engine/semideterministicretriever.h \
    slam/engine/deterministicretriever.h \
    slam/engine/deterministicslam.h \
    slam/engine/odometryonlyslam.h \
    slam/engine/odometrycovariancemodels.h \
    slam/geometry/point.h \
    slam/geometry/frontier.h \
    slam/geometry/linesegment.h \
    slam/geometry/rototranslation.h \
    slam/geometry/uncertainrototranslation.h \
    slam/geometry/visibilitypolygon.h \
    slam/geometry/uncertainlinesegment.h \
    slam/geometry/quadrilateral.h \
    slam/geometry/pointscan.h \
    slam/geometry/segmentscan.h \
    slam/geometry/scanbox.h \
    slam/geometry/linefitting.h \
    slam/geometry/clustering.h \
    slam/legacy/associationbase.h \
    slam/legacy/associationamigoni.h \
    slam/legacy/associationelseberg.h \
    slam/legacy/iclslam.h \
    slam/legacy/icl.h \
    slam/legacy/minimizerdecoupled.h \
    slam/scanmatching/associationbase.h \
    slam/scanmatching/associationamigoni.h \
    slam/scanmatching/associationelseberg.h \
    slam/scanmatching/associationligriffiths.h \
    slam/scanmatching/associationprobabilistic.h \
    slam/scanmatching/associationposecentric.h \
    slam/scanmatching/uninformedassociationbase.h \
    slam/scanmatching/overlap.h \
    slam/scanmatching/decoupledestimator.h \
    slam/scanmatching/scanmatcher.h \
    slam/scanmatching/filteredicl.h \
    slam/scanmatching/classicicl.h \
    slam/scanmatching/ligriffithsicl.h \
    slam/scanmatching/ransacmatcher.h \
    slam/scanmatching/nonoverlapdistance.h \
    slam/scanmatching/retriever.h \
    slam/scanmatching/informedretriever.h \
    slam/scanmatching/fullyinformedretriever.h \
    slam/scanmatching/weightingstrategy.h \
    slam/scanmatching/uniformweightingstrategy.h \
    slam/scanmatching/minlengthweightingstrategy.h \
    slam/scanmatching/mlweightingstrategy.h \
    slam/support/alignedvector.h \
    slam/support/assert.h \
    slam/support/fforeach.h \
    slam/support/mathfuncs.h \
    slam/support/indexset.h \
    slam/support/resourcepooler.h \
    slam/support/topvalues.h \
    slam/support/inverses.h \
    slam/support/nearestmatrices.h \
    slam/support/stopwatch.h \
    slam/PRM/prmalgorithm.h 

!isEmpty(USE_ISAM) {
    SOURCES += \    
        slam/engine/isamlandmark.cpp \
        slam/engine/isamslam.cpp
        
    HEADERS += \
        slam/engine/isamlandmark.h \
        slam/engine/isamslam.h
}

!isEmpty(EVALUATION_TESTING) {
    SOURCES += \        
        slam/evaluation/distanceevaluation.cpp \
        slam/evaluation/covarianceevaluation.cpp \
        slam/evaluation/scanmatchingevaluation.cpp \
        slam/evaluation/truthmap.cpp \
        slam/evaluation/concurrentrunnerhelper.cpp \
        slam/evaluation/evaluationtests.cpp \
        slam/evaluation/functionworker.cpp

    HEADERS += \
        slam/evaluation/truthmap.h \
        slam/evaluation/markedlinesegment.h \
        slam/evaluation/evaluationtests.h \
        slam/evaluation/concurrentrunner.h \
        slam/evaluation/concurrentrunnerhelper.h \
        slam/evaluation/functionworker.h
}
