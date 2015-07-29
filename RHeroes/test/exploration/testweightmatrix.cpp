#include "testweightmatrix.h"
#include "exploration/criteria/weightmatrix.h"
#include <QtTest>

namespace Test{
using namespace Exploration;

    TestWeightMatrix::TestWeightMatrix()
    {
    }

    TestWeightMatrix::~TestWeightMatrix()
    {

    }

    void TestWeightMatrix::testCreation()
    {
        WeightMatrix *matrix = new WeightMatrix(3);

        delete matrix;

    }

    void TestWeightMatrix::testInsertAndGetSingle()
    {
        WeightMatrix *matrix = new WeightMatrix(3);
        QString distance = "distance";
        QString infoGain = "informationGain";
        QString battery = "battery";
        matrix->insertSingleCriterion(distance, 0.9, true);
        matrix->insertSingleCriterion(infoGain, 0.7, true);
        matrix->insertSingleCriterion(battery, 0.5, true);

        QVERIFY(QString::number(1) == matrix->getNameEncoding(distance));
        QVERIFY(QString::number(2) == matrix->getNameEncoding(infoGain));
        QVERIFY(QString::number(3) == matrix->getNameEncoding(battery));
        QVERIFY(0.5 == matrix->getWeight(matrix->getNameEncoding(battery)));
        QVERIFY(0.9 == matrix->getWeight(matrix->getNameEncoding(distance)));
        QVERIFY(0.7 == matrix->getWeight(matrix->getNameEncoding(infoGain)));

        delete matrix;
    }

    void TestWeightMatrix::testComputeEncoding()
    {
        WeightMatrix *matrix = new WeightMatrix(3);
        QString distance = "distance";
        QString infoGain = "informationGain";
        QString battery = "battery";
        matrix->insertSingleCriterion(distance, 0.9, true);
        matrix->insertSingleCriterion(infoGain, 0.7, true);
        matrix->insertSingleCriterion(battery, 0.5, true);

        QList<QString> names;
        names.append(distance);
        names.append(infoGain);
        QVERIFY("12" == matrix->computeNamesEncoding(names));
        names.clear();

        names.append(distance);
        names.append(battery);
        QVERIFY("13" == matrix->computeNamesEncoding(names));
        names.clear();

        names.append(infoGain);
        names.append(battery);
        QVERIFY("23" == matrix->computeNamesEncoding(names));
        names.clear();

        names.append(infoGain);
        names.append(distance);
        QVERIFY("12" == matrix->computeNamesEncoding(names));
        names.clear();

        names.append(battery);
        names.append(distance);
        QVERIFY("13" == matrix->computeNamesEncoding(names));
        names.clear();

        names.append(battery);
        names.append(infoGain);
        QVERIFY("23" == matrix->computeNamesEncoding(names));
        names.clear();

        names.append(distance);
        names.append(infoGain);
        names.append(battery);
        QVERIFY("123" == matrix->computeNamesEncoding(names));
        names.clear();

        names.append(distance);
        names.append(battery);
        names.append(infoGain);
        QVERIFY("123" == matrix->computeNamesEncoding(names));
        names.clear();

        names.append(infoGain);
        names.append(distance);
        names.append(battery);
        QVERIFY("123" == matrix->computeNamesEncoding(names));
        names.clear();

        names.append(infoGain);
        names.append(battery);
        names.append(distance);
        QVERIFY("123" == matrix->computeNamesEncoding(names));
        names.clear();

        names.append(battery);
        names.append(infoGain);
        names.append(distance);
        QVERIFY("123" == matrix->computeNamesEncoding(names));
        names.clear();

        names.append(battery);
        names.append(distance);
        names.append(infoGain);
        QVERIFY("123" == matrix->computeNamesEncoding(names));
        names.clear();

        delete matrix;
    }

    void TestWeightMatrix::testInsertCompoundWeight()
    {
        WeightMatrix *matrix = new WeightMatrix(3);
        QString distance = "distance";
        QString infoGain = "informationGain";
        QString battery = "battery";
        matrix->insertSingleCriterion(distance, 0.9, true);
        matrix->insertSingleCriterion(infoGain, 0.7, true);
        matrix->insertSingleCriterion(battery, 0.5, true);

        QList<QString> names;
        names.append(distance);
        names.append(infoGain);
        QString enc = matrix->computeNamesEncoding(names);
        matrix->insertCombinationWeight(enc, 0.99);
        names.clear();
        QVERIFY(0.99 == matrix->getWeight("12"));

        names.append(distance);
        names.append(battery);
        enc = matrix->computeNamesEncoding(names);
        matrix->insertCombinationWeight(enc, 0.6);
        names.clear();
        QVERIFY(0.6 == matrix->getWeight("13"));

        names.append(infoGain);
        names.append(battery);
        enc = matrix->computeNamesEncoding(names);
        matrix->insertCombinationWeight(enc, 0.75);
        names.clear();
        QVERIFY(0.75 == matrix->getWeight("23"));

        names.append(infoGain);
        names.append(distance);
        enc = matrix->computeNamesEncoding(names);
        names.clear();
        QVERIFY(0.99 == matrix->getWeight(enc));

        names.append(battery);
        names.append(distance);
        enc = matrix->computeNamesEncoding(names);
        names.clear();
        QVERIFY(0.6 == matrix->getWeight(enc));

        names.append(battery);
        names.append(infoGain);
        enc = matrix->computeNamesEncoding(names);
        names.clear();
        QVERIFY(0.75 == matrix->getWeight(enc));

        names.append(distance);
        names.append(infoGain);
        names.append(battery);
        enc = matrix->computeNamesEncoding(names);
        names.clear();
        QVERIFY(1 == matrix->getWeight(enc));

        names.append(distance);
        names.append(battery);
        names.append(infoGain);
        enc = matrix->computeNamesEncoding(names);
        names.clear();
        QVERIFY(1 == matrix->getWeight(enc));

        names.append(infoGain);
        names.append(distance);
        names.append(battery);
        enc = matrix->computeNamesEncoding(names);
        names.clear();
        QVERIFY(1 == matrix->getWeight(enc));

        names.append(infoGain);
        names.append(battery);
        names.append(distance);
        enc = matrix->computeNamesEncoding(names);
        names.clear();
        QVERIFY(1 == matrix->getWeight(enc));

        names.append(battery);
        names.append(infoGain);
        names.append(distance);
        enc = matrix->computeNamesEncoding(names);
        names.clear();
        QVERIFY(1 == matrix->getWeight(enc));

        names.append(battery);
        names.append(distance);
        names.append(infoGain);
        enc = matrix->computeNamesEncoding(names);
        names.clear();
        QVERIFY(1 == matrix->getWeight(enc));


        delete matrix;
    }

    void TestWeightMatrix::testInsertCompoundWeightFromList()
    {
        WeightMatrix *matrix = new WeightMatrix(3);
        QString distance = "distance";
        QString infoGain = "informationGain";
        QString battery = "battery";
        matrix->insertSingleCriterion(distance, 0.9, true);
        matrix->insertSingleCriterion(infoGain, 0.7, true);
        matrix->insertSingleCriterion(battery, 0.5, true);

        QList<QString> names;
        names.append(distance);
        names.append(infoGain);
        matrix->insertCombinationWeight(names, 0.99);
        names.clear();
        QVERIFY(0.99 == matrix->getWeight("12"));

        names.append(distance);
        names.append(battery);
        matrix->insertCombinationWeight(names, 0.6);
        names.clear();
        QVERIFY(0.6 == matrix->getWeight("13"));

        names.append(infoGain);
        names.append(battery);
        matrix->insertCombinationWeight(names, 0.75);
        names.clear();
        QVERIFY(0.75 == matrix->getWeight("23"));

        names.append(infoGain);
        names.append(distance);
        QVERIFY(0.99 == matrix->getWeight(matrix->computeNamesEncoding(names)));
        QVERIFY(0.99 == matrix->getWeight(names));
        names.clear();

        names.append(battery);
        names.append(distance);
        QVERIFY(0.6 == matrix->getWeight(matrix->computeNamesEncoding(names)));
        QVERIFY(0.6 == matrix->getWeight(names));
        names.clear();

        names.append(battery);
        names.append(infoGain);
        QVERIFY(0.75 == matrix->getWeight(matrix->computeNamesEncoding(names)));
        QVERIFY(0.75 == matrix->getWeight(names));
        names.clear();

        names.append(distance);
        names.append(infoGain);
        names.append(battery);
        QVERIFY(1 == matrix->getWeight(matrix->computeNamesEncoding(names)));
        QVERIFY(1 == matrix->getWeight(names));
        names.clear();

        names.append(distance);
        names.append(battery);
        names.append(infoGain);
        QVERIFY(1 == matrix->getWeight(matrix->computeNamesEncoding(names)));
        QVERIFY(1 == matrix->getWeight(names));
        names.clear();

        names.append(infoGain);
        names.append(distance);
        names.append(battery);
        QVERIFY(1 == matrix->getWeight(matrix->computeNamesEncoding(names)));
        QVERIFY(1 == matrix->getWeight(names));
        names.clear();

        names.append(infoGain);
        names.append(battery);
        names.append(distance);
        QVERIFY(1 == matrix->getWeight(matrix->computeNamesEncoding(names)));
        QVERIFY(1 == matrix->getWeight(names));
        names.clear();

        names.append(battery);
        names.append(infoGain);
        names.append(distance);
        QVERIFY(1 == matrix->getWeight(matrix->computeNamesEncoding(names)));
        QVERIFY(1 == matrix->getWeight(names));
        names.clear();

        names.append(battery);
        names.append(distance);
        names.append(infoGain);
        QVERIFY(1 == matrix->getWeight(matrix->computeNamesEncoding(names)));
        QVERIFY(1 == matrix->getWeight(names));
        names.clear();


        delete matrix;
    }

    void TestWeightMatrix::testGetActiveCriteria()
    {
        WeightMatrix *matrix = new WeightMatrix(3);
        QString distance = "distance";
        QString infoGain = "informationGain";
        QString battery = "battery";
        matrix->insertSingleCriterion(distance, 0.9, true);
        matrix->insertSingleCriterion(infoGain, 0.7, true);
        matrix->insertSingleCriterion(battery, 0.5, false);

        QList<QString> active = matrix->getActiveCriteria();

        QVERIFY(active.size() == 2);
        QVERIFY(active.contains(distance) == true);
        QVERIFY(active.contains(infoGain) == true);
        QVERIFY(active.contains(battery) == false);
        QVERIFY(matrix->getNumOfActiveCriteria() == 2);
        delete matrix;
    }

    void TestWeightMatrix::testChangeCriterionState()
    {
        WeightMatrix *matrix = new WeightMatrix(3);
        QString distance = "distance";
        QString infoGain = "informationGain";
        QString battery = "battery";
        matrix->insertSingleCriterion(distance, 0.9, true);
        matrix->insertSingleCriterion(infoGain, 0.7, true);
        matrix->insertSingleCriterion(battery, 0.5, false);

        QList<QString> active = matrix->getActiveCriteria();

        QVERIFY(active.size() == 2);
        QVERIFY(active.contains(distance) == true);
        QVERIFY(active.contains(infoGain) == true);
        QVERIFY(active.contains(battery) == false);
        QVERIFY(matrix->getNumOfActiveCriteria() == 2);

        matrix->changeCriteriaActivation(distance, true);
        active = matrix->getActiveCriteria();

        QVERIFY(active.size() == 2);
        QVERIFY(active.contains(distance) == true);
        QVERIFY(active.contains(infoGain) == true);
        QVERIFY(active.contains(battery) == false);
        QVERIFY(matrix->getNumOfActiveCriteria() == 2);

        matrix->changeCriteriaActivation(battery, false);
        active = matrix->getActiveCriteria();

        QVERIFY(active.size() == 2);
        QVERIFY(active.contains(distance) == true);
        QVERIFY(active.contains(infoGain) == true);
        QVERIFY(active.contains(battery) == false);
        QVERIFY(matrix->getNumOfActiveCriteria() == 2);

        matrix->changeCriteriaActivation(distance, false);
        active = matrix->getActiveCriteria();

        QVERIFY(active.size() == 1);
        QVERIFY(active.contains(distance) == false);
        QVERIFY(active.contains(infoGain) == true);
        QVERIFY(active.contains(battery) == false);
        QVERIFY(matrix->getNumOfActiveCriteria() == 1);

        matrix->changeCriteriaActivation(battery, true);
        active = matrix->getActiveCriteria();

        QVERIFY(active.size() == 2);
        QVERIFY(active.contains(distance) == false);
        QVERIFY(active.contains(infoGain) == true);
        QVERIFY(active.contains(battery) == true);
        QVERIFY(matrix->getNumOfActiveCriteria() == 2);

        delete matrix;
    }

}
