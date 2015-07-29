/*
 * isamslam.cpp
 *
 *  Created on: 24/gen/2013
 *      Author: Mladen Mazuran
 */

#include "shared/logger.h"
#include "isamretriever.h"
#include "isamslam.h"
#include "odometrycovariancemodels.h"
#include "slam/support/nearestmatrices.h"
#include "slam/support/inverses.h"
#include "slam/scanmatching/associationprobabilistic.h"
#include "slam/scanmatching/associationposecentric.h"
#include "slam/scanmatching/ligriffithsicl.h"
#include "slam/scanmatching/classicicl.h"
#include "slam/scanmatching/filteredicl.h"
#include "slam/scanmatching/ransacmatcher.h"
#include <QSet>

namespace SLAM {
namespace Engine {

using namespace Geometry;
using namespace Data;
using namespace ScanMatching;

typedef BasicOdometryCovarianceModel OdometryCovarianceModel;
typedef AssociationPoseCentric AssociationModel;

ISAMSLAM::ISAMSLAM(uint robotId, const Pose &initialPose, ScanMatcherSelection matcher) :
        robotId(robotId), firstOdometry(true), ins(0, Pose3D()), initialPose(initialPose),
        deltaOdometry(0, 0, 0)
{
    isam::Properties p;
    p.quiet = true;
    slam.set_properties(p);
    switch(matcher) {
    case LiGriffithsICLSelection:
        this->matcher = new LiGriffithsICL<ISAMRetriever>;
        break;
    case ClassicICLSelection:
        this->matcher = new ClassicICL<ISAMRetriever, AssociationProbabilistic>;
        break;
    case FilteredICLSelection:
        this->matcher = new FilteredICL<ISAMRetriever, AssociationProbabilistic>;
        break;
    case RANSACMatcherSelection:
        this->matcher = new RANSACMatcher<ISAMRetriever, AssociationProbabilistic>;//AssociationPoseCentric>;
        break;
    }
}

ISAMSLAM::~ISAMSLAM()
{
    delete matcher;
    std::list<isam::Factor *> factors = slam.get_factors();
    std::list<isam::Node *>   nodes   = slam.get_nodes();

    fforeach(isam::Factor *factor, factors) {
        slam.remove_factor(factor);
        delete factor;
    }

    fforeach(isam::Node *node, nodes) {
        slam.remove_node(node);
        delete node;
    }
}

bool ISAMSLAM::sufficientlyFar() const
{
    if(lastPoses.empty() || landmarks.empty())
        return true;

    Pose lastPose         = lastPoses.last()->pose();
    Pose lastLandmarkPose = landmarks.last()->pose();

    bool spatialCheck = lastPose.getDistance(lastLandmarkPose) >=
                        Config::SLAM::landmarkSpatialDistance;
    bool angularCheck = std::fabs(lastLandmarkPose.theta() - lastPose.theta()) >=
                        Config::SLAM::landmarkAngularDistance;

    return spatialCheck || angularCheck;
}

bool ISAMSLAM::sufficientlyFarFromStructural() const
{
    if(lastPoses.empty() || landmarks.empty())
        return true;

    Pose lastPose         = lastPoses.last()->pose();

    QListIterator<ISAMLandmark *> it(landmarks);
    it.toBack();
    while(it.hasPrevious()) {
        ISAMLandmark *landmark = it.previous();
        if(landmark->structural()) {
            Pose landmarkPose = landmark->pose();

            bool spatialCheck = lastPose.getDistance(landmarkPose) >=
                                Config::SLAM::landmarkSpatialDistance;
            bool angularCheck = std::fabs(landmarkPose.theta() - lastPose.theta()) >=
                                Config::SLAM::landmarkAngularDistance;

            if(!(spatialCheck || angularCheck)) return false;
        }
    }

    return true;
}

QList<int> ISAMSLAM::lookupCovariancesAndLandmarks(const SegmentScan &query, ISAMLandmark *robot)
{
    QList<int> lookedUp;

    covariances.resize(landmarks.size());

    isam::Covariances isamcov = slam.covariances();
    isam::Covariances::node_pair_list_t pairList;
    pairList.push_back(std::make_pair<isam::Node *, isam::Node *>(robot, robot));
    fforeach(ISAMLandmark *landmark, landmarks) {
        if(landmark->structural()) {
            pairList.push_back(std::make_pair<isam::Node *, isam::Node *>(landmark, landmark));
            pairList.push_back(std::make_pair<isam::Node *, isam::Node *>(robot,    landmark));
        }
    }

    const std::list<Eigen::MatrixXd> covList = isamcov.access(pairList);
    const Point robotCenter = robot->pose().position();
    ScanBox sbox(query);
    sbox = Rototranslation(robot->pose()) * sbox;

    std::list<Eigen::MatrixXd>::const_iterator it = covList.begin();
    const Eigen::MatrixXd &Srr = *it; ++it;
    int i = 0, count = 0;
    fforeach(ISAMLandmark *landmark, landmarks) {
        if(landmark->structural()) {
            Geometry::ScanBox querybox = sbox, mapbox = landmark->boxWorld();
            const Eigen::MatrixXd &Sll = *it; ++it;
            const Eigen::MatrixXd &Srl = *it; ++it;

            Eigen::Matrix3d relativeCovariance = Srr + Sll - Srl - Srl.transpose();

#if 0
            Eigen::Matrix<double, 6, 6> total;
            total << Srr, Srl, Srl.transpose(), Sll;
            if(total.eigenvalues().real().minCoeff() < 0) {
                ldbg << "Fallimento: " << total << endl;
            }
#endif
            //ldbg << "Sll: " << Sll << endl;
            //ldbg << "Srl: " << Srl << endl;

            querybox.augment(relativeCovariance.block<2,2>(0,0));
            Geometry::Rototranslation rt1(robotCenter,  3 * std::sqrt(relativeCovariance(2, 2)));
            Geometry::Rototranslation rt2(robotCenter, -3 * std::sqrt(relativeCovariance(2, 2)));

            if(querybox.intersects(mapbox) || (rt1 * querybox).intersects(mapbox) ||
                    (rt2 * querybox).intersects(mapbox)) {
                landmark->propagate();
                lookedUp.append(i);
                covariances[i] = std::make_pair(
                        static_cast<Eigen::Matrix3d>(Sll),
                        static_cast<Eigen::Matrix3d>(Srl));
            }
            count++;
        }
        ++i;
    }

    ldbg << "Structural count: " << count << endl;

    robotCovariance = Srr;

    //ldbg << robot->id() << " -> " << lookedUp << endl;
    //ldbg << "Srr: " << Srr << endl;

    return lookedUp;
}

#define MINIMUM_OVERLAP_AMOUNT          1.
#define MINIMUM_OVERLAP_AMOUNT_FAR      4.
#define NEARNESS_THRESHOLD              3.

struct MatchingInfo {
    int index;
    ISAMLandmark *landmark;
    double overlap;
    UncertainRototranslation measure;
    bool far;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

bool matchingInfoComparator(const MatchingInfo *mi1, const MatchingInfo *mi2) {
    return mi1->overlap < mi2->overlap;
}

void ISAMSLAM::handleScan(double timestamp, const PointScan &scan)
{
    Q_UNUSED(timestamp)

    SegmentScan ss(scan);
    UncertainRototranslation delta = OdometryCovarianceModel::addCovariance(deltaOdometry);
    if(landmarks.empty()) {
        Eigen::Matrix3d cov  = 1e-10 * Eigen::Matrix3d::Identity() + delta.covariance();
        Eigen::Vector3d meas = initialPose * deltaOdometry;
        ISAMLandmark *start = new ISAMLandmark(0, timestamp, scan);
        landmarks.append(start);
        slam.add_node(start);
        slam.add_factor(new isam::Pose2d_Factor(start, isam::Pose2d(meas), isam::Covariance(cov)));
    } else {
        ISAMLandmark *robot;// = new ISAMLandmark(timestamp, ISAMLandmark::Temporary);
        ISAMLandmark *previous;
        uint id = lastPoses.size() + landmarks.size();
        bool appendToLandmarks = false;

        if(lastPoses.empty()) {
            previous = landmarks.last();
        } else {
            previous = lastPoses.last();
        }

        if(sufficientlyFar()) {
            if(sufficientlyFarFromStructural()) {
                robot = new ISAMLandmark(id, timestamp, ss);
            } else {
                robot = new ISAMLandmark(id, timestamp, ISAMLandmark::Transitional);
            }
            appendToLandmarks = true;
        } else {
            robot = new ISAMLandmark(id, timestamp, ISAMLandmark::Temporary);
        }


        /* Prediction step */
        lastPoses.append(robot);
        slam.add_node(robot);
        slam.add_factor(new isam::Pose2d_Pose2d_Factor(previous, robot,
                isam::Pose2d(delta.vectorForm()), isam::Covariance(delta.covariance())));
        slam.update();
        QList<int> lookedUp = lookupCovariancesAndLandmarks(ss, robot);
        QList<MatchingInfo *> matches, acceptedMatches;

        fforeach(int idx, lookedUp) {
            ISAMRetriever retriever(
                    robot, landmarks[idx], robotCovariance,
                    covariances[idx].second, covariances[idx].first, ss);

            /* Scan matching */
            matcher->setRetriever(retriever);
            matcher->run();
            retriever.resetTransformation();

            Eigen::Matrix3d relativeCovariance =
                    robotCovariance + covariances[idx].first -
                    covariances[idx].second - covariances[idx].second.transpose();
            Eigen::Vector3d relativeMeasure = matcher->measure() - robot->pose().vectorForm();
            relativeMeasure[2] = wrap(relativeMeasure[2]);

            /* Discard unlikely measurements */
            if(relativeMeasure.transpose() * relativeCovariance.inverse() * relativeMeasure > 15) {
                continue;
            }

            const Rototranslation rt = Rototranslation(matcher->measure()) *
                    retriever.queryPose().inverse();
            const double thisOverlap = overlapAmount(retriever, rt, matcher->associations());

            /* Ignore matchings with not enough overlap */
            if(thisOverlap > MINIMUM_OVERLAP_AMOUNT) {
                MatchingInfo *mi = new MatchingInfo;
                mi->index = idx;
                mi->landmark = landmarks[idx];
                mi->overlap = thisOverlap;
                mi->measure = UncertainRototranslation(
                        Rototranslation(matcher->measure()), matcher->covariance());
                mi->far = (mi->measure.translation() - robot->pose().position()).norm() >
                            NEARNESS_THRESHOLD;

                /* "Far away" landmarks need a larger amount of overlap to be accepted */
                if(mi->far && thisOverlap < MINIMUM_OVERLAP_AMOUNT_FAR) {
                    delete mi;
                    continue;
                }

                matches.append(mi);
            }
        }

        qSort(matches.begin(), matches.end(), matchingInfoComparator);
        if(matches.size() > 0) acceptedMatches.append(matches.last());

        /*
        for(int i = 0; i < matches.size() - 1; i++) {
            Eigen::Vector3d d = matches[i]->measure.vectorForm() - matches.first()->measure.vectorForm();
            if(d.block<2,1>(0,0).norm() < 0.02 && std::abs(wrap(d[2])) < M_PI / 90) {
                acceptedMatches.append(matches[i]);
            }
        }
        */

        /* Update step */
        fforeach(MatchingInfo *mi, acceptedMatches) {
            UncertainRototranslation actualMeasure =
                    Rototranslation(mi->landmark->pose()).inverse() * mi->measure;

            slam.add_factor(new isam::Pose2d_Pose2d_Factor(mi->landmark, robot,
                    isam::Pose2d(actualMeasure.vectorForm()),
                    isam::Covariance(actualMeasure.covariance())));
            robot->addCorrelation(mi->landmark);
        }

        qDeleteAll(matches);

        ldbg << deltaOdometry.vectorForm() << endl;
        ldbg << "-------------------------------------------------------------------------" << endl;

        /* TODO: Temporary */
        if(appendToLandmarks) {
            landmarks.append(robot);
        }

#if 0
        /* Landmark placement */
        if(sufficientlyFar()) {
            if(sufficientlyFarFromStructural()) {
                addLandmark(timestamp, &ss);
            } else {
                addLandmark(timestamp);
            }
        }
#endif
    }

    deltaOdometry = Rototranslation(0, 0, 0);
    slam.update();

    //slam.print_graph();


#if 0
    slam.print_graph();

    isam::Covariances isamcov = slam.covariances();
    isam::Covariances::node_lists_t lists;
    std::list<isam::Node *> list;
    foreach(isam::Node *n, slam.get_nodes()) {
        list.push_back(n);
    }
    lists.push_back(list);
    ldbg << isamcov.marginal(lists).front() << endl;
#endif
}

void ISAMSLAM::handleOdometry(double timestamp, const Pose &pose)
{
    Q_UNUSED(timestamp)

    if(firstOdometry) {
        odometry = pose;
        firstOdometry = false;
    }

    Eigen::Vector3d delta = (odometry.inverse() * Rototranslation(pose)).vectorForm();
    if(delta[2] > M_PI_2) delta[2] = M_PI_2;
    else if(delta[2] < -M_PI_2) delta[2] = -M_PI_2;

    deltaOdometry = deltaOdometry * Rototranslation(delta);
    odometry = pose;
}

void ISAMSLAM::handleINS(const INSData &ins)
{
    this->ins = ins;
}

Eigen::Matrix3d ISAMSLAM::compactMeasureInformation(
        const ISAMLandmark *from, const Rototranslation &to,
        const Eigen::MatrixXd &jointCov) const
{
    Eigen::Matrix3d Jl, invJx;
    const Data::Pose l = from->pose(), x = to.vectorForm();
    const double cl = std::cos(l.theta()), sl = std::sin(l.theta());
    const double cx = std::cos(x.theta()), sx = std::sin(x.theta());

    Jl <<
           -cl, -sl, -(l.y() - x.y()) * cl + (l.x() - x.x()) * sl,
            sl, -cl,  (l.x() - x.x()) * cl + (l.y() - x.y()) * sl,
             0,   0, -1;
    invJx <<
            cx, -sx,  0,
            sx,  cx,  0,
             0,   0,  1;

    const Eigen::Matrix3d M = Support::lowerBlockInverse<3>(jointCov);
    const Eigen::Matrix3d A =
            invJx * (M - Jl * jointCov.block<3, 3>(0, 0) * Jl.transpose()) * invJx.transpose();
    return Support::nearestDefinitePositive(A);
}

void ISAMSLAM::addLandmark(double timestamp, const SegmentScan *scan)
{
    QHash<ISAMLandmark *, ISAMLandmark *> landmarkLookup, temporaryLookup;
    isam::Slam stepslam;
    isam::Properties p;
    p.quiet = true;
    stepslam.set_properties(p);

    /* Find the set of correlated landmarks */
    QSet<ISAMLandmark *> correlated;
    fforeach(const ISAMLandmark *pose, lastPoses) {
        correlated.unite(pose->correlations());
    }

    fforeach(ISAMLandmark *l, correlated) {
        ISAMLandmark *landmark = new ISAMLandmark(l->id(), 0);
        landmarkLookup.insert(l, landmark);
        stepslam.add_node(landmark);
        stepslam.add_factor(new isam::Pose2d_Factor(
                landmark, l->value(), isam::Information(.1 * isam::eye(3))));
    }

    fforeach(ISAMLandmark *node, lastPoses) {
        if(!temporaryLookup.contains(node)) {
            ISAMLandmark *landmark = new ISAMLandmark(node->id(), 0);
            temporaryLookup.insert(node, landmark);
            stepslam.add_node(landmark);
        }

        fforeach(isam::Factor *f, node->factors()) {
            isam::Pose2d_Pose2d_Factor *meas = dynamic_cast<isam::Pose2d_Pose2d_Factor *>(f);
            if(meas != NULL && meas->nodes()[1] == node) {
                ISAMLandmark *prevnode = dynamic_cast<ISAMLandmark *>(meas->nodes()[0]);
                if(landmarkLookup.contains(prevnode)) {
                    stepslam.add_factor(new isam::Pose2d_Pose2d_Factor(
                            landmarkLookup[prevnode], temporaryLookup[node], meas->measurement(),
                            isam::SqrtInformation(meas->sqrtinf())));
                } else if(temporaryLookup.contains(prevnode)) {
                    stepslam.add_factor(new isam::Pose2d_Pose2d_Factor(
                            temporaryLookup[prevnode], temporaryLookup[node], meas->measurement(),
                            isam::SqrtInformation(meas->sqrtinf())));
                } else {
                    ISAMLandmark *newprevnode = new ISAMLandmark(prevnode->id(), 0);
                    temporaryLookup.insert(prevnode, newprevnode);
                    stepslam.add_node(newprevnode);
                    stepslam.add_factor(new isam::Pose2d_Pose2d_Factor(
                            newprevnode, temporaryLookup[node], meas->measurement(),
                            isam::SqrtInformation(meas->sqrtinf())));
                }
            }
        }
    }

    stepslam.batch_optimization();

    ISAMLandmark *lastPose = temporaryLookup[lastPoses.last()];

    isam::Covariances::node_pair_list_t pairs;
    pairs.push_back(std::make_pair<isam::Node *, isam::Node *>(lastPose, lastPose));
    fforeach(ISAMLandmark *l, correlated) {
        ISAMLandmark *landmark = landmarkLookup[l];
        pairs.push_back(std::make_pair<isam::Node *, isam::Node *>(landmark, landmark));
        pairs.push_back(std::make_pair<isam::Node *, isam::Node *>(landmark, lastPose));
    }
    const std::list<Eigen::MatrixXd> covList = slam.covariances().access(pairs);

    std::list<Eigen::MatrixXd>::const_iterator it = covList.begin();
    const Eigen::MatrixXd &Srr = *it; ++it;
    const Pose robot = lastPose->pose();

    fforeach(ISAMLandmark *n, lastPoses) {
        /* Iterate on a copy using standard foreach, otherwise we crash */
        foreach(isam::Factor *f, n->factors()) {
            slam.remove_factor(f);
            delete f;
        }
        slam.remove_node(n);
        delete n;
    }
    lastPoses.clear();

    ISAMLandmark *newLandmark;
    if(scan) {
        newLandmark = new ISAMLandmark(landmarks.size(), timestamp, *scan);
    } else {
        newLandmark = new ISAMLandmark(landmarks.size(), timestamp, ISAMLandmark::Transitional);
    }

    slam.add_node(newLandmark);
    landmarks.append(newLandmark);

    fforeach(ISAMLandmark *l, correlated) {
        const Eigen::MatrixXd &Sll = *it; ++it;
        const Eigen::MatrixXd &Slr = *it; ++it;
        const Pose land = landmarkLookup[l]->pose();

        Eigen::Matrix<double, 6, 6> fullCov;
        fullCov <<
                Srr, Slr.transpose(), Slr, Sll;
        Eigen::Matrix<double, 3, 6> J;
        const double c = std::cos(land.theta()), s = std::sin(land.theta());
        J <<
                 c, s, 0, -c, -s, (robot.y() - land.y()) * c + (land.x() - robot.x()) * s,
                -s, c, 0,  s, -c, (land.x() - robot.x()) * c + (land.y() - robot.y()) * s,
                 0, 0, 1,  0,  0, -1;
        slam.add_factor(new isam::Pose2d_Pose2d_Factor(l, newLandmark,isam::Pose2d(
                        (Rototranslation(land).inverse() * Rototranslation(robot)).vectorForm()),
                isam::Covariance(J * fullCov * J.transpose())));
    }

    /* Iterate on a copy using standard foreach, otherwise we crash */
    foreach(isam::Factor *f, stepslam.get_factors()) {
        stepslam.remove_factor(f);
        delete f;
    }
    /* Iterate on a copy using standard foreach, otherwise we crash */
    foreach(isam::Node *n, stepslam.get_nodes()) {
        stepslam.remove_node(n);
        delete n;
    }
}

Map ISAMSLAM::getMap() const
{
    Map m;
    int i = 0;
    fforeach(const ISAMLandmark *l, landmarks) {
        Pose p = l->pose();
        Rototranslation rt(p);
        m.addPose(robotId, TimedPose(l->timestamp(), p));
        if(l->structural() && i < covariances.size()) {
            fforeach(const LineSegment &s, l->segments()) {
                m.addWall(rt * s);
            }
            VisibilityPolygon vp = rt * l->visibility();
            const_cast<PathNode *>(m.lastRobotPose(robotId))->setVisibility(&vp);
            if(i < covariances.size()) {
                const_cast<PathNode *>(m.lastRobotPose(robotId))->setCovariance(covariances[i].first);
            } else {
                const_cast<PathNode *>(m.lastRobotPose(robotId))->setCovariance(covariances[i-1].first);
            }
        }
        ++i;
    }

    if(!lastPoses.empty()) {
        m.addPose(robotId, TimedPose(lastPoses.last()->timestamp(), lastPoses.last()->pose()));
        const_cast<PathNode *>(m.lastRobotPose(robotId))->setCovariance(robotCovariance);
    }

    m.addPose(BASE_STATION_ID, TimedPose(0, Config::baseStationPose));

    //ldbg << "getMap(1): " << covs.back() << endl;
    //ldbg << "getMap(2): " << m.lastRobotPose(robotId)->covariance() << endl;

    return m;
}

} /* namespace Engine */
} /* namespace SLAM */
