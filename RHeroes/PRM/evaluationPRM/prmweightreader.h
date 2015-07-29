#ifndef PRMWEIGHTREADER_H
#define PRMWEIGHTREADER_H

#include <QObject>
#include <QHash>
#include "exploration/criteria/weightmatrix.h"

namespace PRM{
using namespace Exploration;

class PRMWeightReader : public QObject
{
    Q_OBJECT
public:
    explicit PRMWeightReader(QObject *parent = 0);

    WeightMatrix* parseFile();
    
signals:
    
public slots:
    
};

}
#endif // PRMWEIGHTREADER_H
