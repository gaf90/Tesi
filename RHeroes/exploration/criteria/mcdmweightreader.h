#ifndef MCDMWEIGHTREADER_H
#define MCDMWEIGHTREADER_H

#include <QObject>
#include <QHash>
#include "weightmatrix.h"

namespace Exploration{
class MCDMWeightReader : public QObject
{
    Q_OBJECT
public:
    explicit MCDMWeightReader(QObject *parent = 0);
    virtual ~MCDMWeightReader();

    WeightMatrix* parseFile();
signals:
    
public slots:
    
};
}

#endif // MCDMWEIGHTREADER_H
