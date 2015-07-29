#include "prmweightreader.h"
#include "shared/logger.h"

#include <QFile>
#include <QString>
#include <QByteArray>
#include <QStringList>
#include <QDomDocument>


namespace PRM{
using namespace Exploration;

PRMWeightReader::PRMWeightReader(QObject *parent) :
    QObject(parent)
{
}

WeightMatrix* PRMWeightReader::parseFile()
{
    QFile *file = new QFile("PRMconf.conf");
    WeightMatrix *matrix = NULL;
    QDomDocument *doc = new QDomDocument("PRMConf");
    //file->open(QFile::ReadOnly);
    if(!file->open(QFile::ReadOnly)){
        ldbg<<"impossibile aprire il file prm"<<endl;
        return matrix;
    }
    doc->setContent(file);

    QDomElement root = doc->documentElement();
    QDomNode node = root.firstChild();
    while(!node.isNull()){
        QDomElement elem = node.toElement();
        if(!elem.isNull()){
            if(elem.tagName() == "numOfCriteria"){
                int num = elem.firstChild().toText().data().toInt();
                matrix = new WeightMatrix(num);
            } else if (elem.tagName() == "singleCriterion"){
                QDomNode singleCritNode = elem.firstChild();
                QString name = "";
                double weight = 0.0;
                bool isActive = false;

                while(!singleCritNode.isNull()){
                    QDomElement singleCrit = singleCritNode.toElement();

                    if(singleCrit.tagName() == "name"){
                        name = singleCrit.firstChild().toText().data();
                    } else if (singleCrit.tagName() == "weight") {
                        weight = singleCrit.firstChild().toText().data().toDouble();
                    } else if (singleCrit.tagName() == "isActive"){
                        isActive = singleCrit.firstChild().toText().data() == "true" ? true : false;
                    }

                    singleCritNode = singleCritNode.nextSibling();
                }

                matrix->insertSingleCriterion(name, weight, isActive);

            } else if (elem.tagName() == "combinedCriteria"){
                QDomNode multipleCritNode = elem.firstChild();
                QList<QString> nameList;
                double weight = 0.0;

                while(!multipleCritNode.isNull()){
                    QDomElement multipleCrit = multipleCritNode.toElement();

                    if(multipleCrit.tagName() == "name"){
                        nameList.append(multipleCrit.firstChild().toText().data());
                    } else if (multipleCrit.tagName() == "weight"){
                        weight = multipleCrit.firstChild().toText().data().toDouble();
                    }

                    multipleCritNode = multipleCritNode.nextSibling();
                }
                matrix->insertCombinationWeight(nameList, weight);
            }
            node = node.nextSibling();
        }
    }
    file->close();
    delete file;
    delete doc;
    return matrix;
}

}
