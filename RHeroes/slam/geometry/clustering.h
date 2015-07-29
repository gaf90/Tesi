/*
 * clustering.h
 *
 *  Created on: 22/feb/2012
 *      Author: Mladen Mazuran
 */

#ifndef CLUSTERING_H_
#define CLUSTERING_H_

#include "slam/support/indexset.h"
#include "slam/constants.h"
#include "pointscan.h"
#include "shared/config.h"
#include <QList>

namespace SLAM {
namespace Geometry {

namespace __internal {
inline bool forwardJoin(
        const PointScan &scan, QList<Support::IndexSet> &clusters, int i, int amount) {

    const int i2 = i + amount;
    if(i2 >= clusters.size()) return false;
    if(scan[clusters[i].indexEnd() - 1].distance2(scan[clusters[i2].indexBegin()])
            < SM_CLUSTER_DISTANCE_THRESHOLD_2) {

        clusters[i] = clusters[i].join(clusters[i2]);
        clusters.removeAt(i2);
        return true;
    } else {
        return false;
    }
}
} /* namespace __internal */

inline const QList<Support::IndexSet> findClusters(const PointScan &scan) {
    QList<Support::IndexSet> clusters;
    int start = 0, i = 0;

    double tga = std::tan(scan.resolution());

    /* Find first index in range (actually the index of the point after the first) */
    while(scan.range(i++) >= Config::SLAM::laserOutOfRange && i < scan.size()) {}

    if(i == scan.size()) return clusters;

    start = i - 1;

    for( ; i < scan.size(); i++) {
        if(scan[i].distance2(scan[max(i - 1, start)]) >=
                min(1.2 * scan.range(i) * tga, SM_POINT_DISTANCE_THRESHOLD_2) ||
                scan.range(i) >= Config::SLAM::laserOutOfRange) {
            /* Add only if cluster is large enough */
            if(i - start >= SM_MINIMUM_CLUSTER_SIZE)
                clusters.append(Support::IndexSet(start, i));

            if(scan.range(i) >= Config::SLAM::laserOutOfRange) {
                start = i + 1;
            } else {
                start = i;
            }
        }
    }
    if(start < scan.size() - SM_MINIMUM_CLUSTER_SIZE)
        clusters.append(Support::IndexSet(start, scan.size()));

    for(i = 1; i < clusters.size(); i++) {
        if(scan[clusters[i - 1].indexEnd() - 1].distance2(scan[clusters[i].indexBegin()])
                < SM_CLUSTER_DISTANCE_THRESHOLD_2) {

            clusters[i - 1] = clusters[i - 1].join(clusters[i]);

            clusters.removeAt(i);
        }
    }

    return clusters;
}

inline const QList<Support::IndexSet> findClustersAggressive(const PointScan &scan) {
    QList<Support::IndexSet> clusters;
    int start = 0, i = 0;

    /* Find first index in range (actually the index of the point after the first) */
    while(scan.range(i++) >= Config::SLAM::laserOutOfRange && i < scan.size()) {}

    if(i == scan.size()) return clusters;

    start = i - 1;

    for( ; i < scan.size(); i++) {
        if(scan[i].distance2(scan[max(i - 1, start)]) >= SM_POINT_DISTANCE_THRESHOLD_2) {
            clusters.append(Support::IndexSet(start, i));
            if(start == i) { lprint << "wtf1: " << i << endl; exit(0); }
            start = i;
        } else if(scan.range(i) >= Config::SLAM::laserOutOfRange) {
            if(start < i) {
                clusters.append(Support::IndexSet(start, i));
                if(start == i) { lprint << "wtf2: " << i << endl; exit(0); }
            }
            start = i + 1;
        }
    }
    if(start < scan.size() - 1)
        clusters.append(Support::IndexSet(start, scan.size()));

    bool joined = true;
    while(joined) {
        joined = false;
        for(i = 0; i < clusters.size() - 1; i++) {
            for(int j = 1; j < clusters.size() - 1 - i; j++) {
                joined = joined || __internal::forwardJoin(scan, clusters, i, j);
            }
        }
    }

    for(i = 0; i < clusters.size(); i++) {
        if(clusters[i].size() < SM_MINIMUM_CLUSTER_SIZE) {
            clusters.removeAt(i--);
        }
    }

    return clusters;
}

} /* namespace Geometry */
} /* namespace SLAM */
#endif /* CLUSTERING_H_ */
