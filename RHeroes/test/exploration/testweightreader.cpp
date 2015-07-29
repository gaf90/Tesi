#include "testweightreader.h"
#include "exploration/criteria/mcdmweightreader.h"
#include "exploration/criteria/weightmatrix.h"



namespace Test {

using namespace Exploration;

TestWeightReader::TestWeightReader()
{
}

TestWeightReader::~TestWeightReader()
{
}

void TestWeightReader::testCreateMatrix()
{
    MCDMWeightReader *reader = new MCDMWeightReader();
    WeightMatrix *matrix = reader->parseFile();
    delete reader;
    QVERIFY(matrix->getNumOfActiveCriteria() == 3);
    QList<QString> names;
    names.append("distance");
    QVERIFY(0.8 == matrix->getWeight(names));
    names.clear();
    names.append("informationGain");
    QVERIFY(0.7 == matrix->getWeight(names));
    names.clear();
    names.append("battery");
    QVERIFY(0.01 == matrix->getWeight(names));
    names.clear();
    names.append("informationGain");
    names.append("distance");
    QVERIFY(0.9 == matrix->getWeight(names));
    names.clear();
    names.append("battery");
    names.append("distance");
    QVERIFY(0.1 == matrix->getWeight(names));
    names.clear();
    names.append("battery");
    names.append("informationGain");
    QVERIFY(0.1 == matrix->getWeight(names));
    names.append("distance");
    QVERIFY(1 == matrix->getWeight(names));
    names.clear();

    delete matrix;
}

}
