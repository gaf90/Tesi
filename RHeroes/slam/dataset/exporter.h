/*
 * exporter.h
 *
 *  Created on: 30/nov/2012
 *      Author: Mladen Mazuran
 */

#ifndef EXPORTER_H_
#define EXPORTER_H_

#include <QString>

namespace SLAM {
namespace Dataset {

class MapViewWidget;
class DatasetWorker;

/* Don't really need a class, but meh, it's nicer than random functions */
class Exporter
{
public:
    static void exportPDF(const QString &file, const MapViewWidget *map);
    static void exportCPPCode(const QString &file, DatasetWorker *worker);
};

} /* namespace Dataset */
} /* namespace SLAM */

#endif /* EXPORTER_H_ */
