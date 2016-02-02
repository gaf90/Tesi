#include "configreader.h"
#include "config.h"
#include <QTextStream>
#include <QStringList>
#include <QDebug>

#define SET_VARIABLE(name)                                                  \
    do {                                                                    \
    if(varName.compare(nameFormat(#name), Qt::CaseInsensitive) == 0) {     \
    convertTo(varValue, Config::name);                              \
    }                                                                   \
    } while(0)

#define SET_SLAM_VARIABLE(name) SET_VARIABLE(SLAM::name)
#define SET_PRM_VARIABLE(name) SET_VARIABLE(PRM::name)
#define SET_OBS_VARIABLE(name) SET_VARIABLE(OBS::name)

namespace Config{
ConfigReader::ConfigReader(const QString &filePath, QObject *parent) :
    QObject(parent), filePath(filePath)
{
}

ConfigReader::~ConfigReader()
{

}

int ConfigReader::readFileAndCompileConfigs()
{
    QRegExp regexp("^(?:\\s*#.*|\\s*|([_a-zA-Z][_a-zA-Z0-9/]*)\\s*=\\s*([^#]*)#*.*)$");
    regexp.setMinimal(true);
    QFile file(filePath);
    bool opened = file.open(QIODevice::ReadOnly | QIODevice::Text);
    if(!opened){
        qDebug()<<"CONFIG: File not found";
        return -1;
    }

    QTextStream stream(&file);

    while(!stream.atEnd()){
        QString line = stream.readLine().trimmed();
        bool b = regexp.exactMatch(line);
        if(b) {
            if(regexp.cap(1) != "") {
                QString varName = regexp.cap(1).toLower(); // Let's make it case insensitive
                QString varValue = regexp.cap(2).trimmed();
                setVariable(varName, varValue);
            }
        } else {
            return -2;
        }
    }
    return 0;

}

void ConfigReader::setVariable(const QString &varName, const QString &varValue)
{
    static QRegExp regexp("^[-+]?[0-9]*\\.?[0-9]+(?:[eE][-+]?[0-9]+)?$");
    if(varName.startsWith("slam/", Qt::CaseInsensitive)) {
        setSLAMVariable(varName, varValue);
    }else if(varName.startsWith("prm/", Qt::CaseInsensitive)) {
        setPRMVariable(varName, varValue);
    }else if(varName.startsWith("obs/", Qt::CaseInsensitive)) {
        setOBSVariable(varName, varValue);
    }else if(varValue.at(0) == '"' && varValue.at(varValue.size() - 1) == '"') {
        setVariableString(varName, varValue.mid(1, varValue.length() - 2));
    } else if(regexp.exactMatch(varValue)) {
        setVariableNumber(varName, varValue.toDouble());
    }
}

void ConfigReader::setVariableString(const QString &varName, const QString &value)
{
    if(varName == "usar_address") {
        usarAddress = value;
    } else if(varName == "upis_address") {
        upisAddress = value;
    } else if(varName == "wss_address") {
        wssAddress = value;
    } else if(varName == "policy") {
        if(value == "AOJRF")
            policy = Exploration::AOJRF;
        else if(value == "Semantic")
            policy = Exploration::BasicSemantic;
        else if(value == "Dummy")
            policy = Exploration::Dummy;
        else if(value == "PRM")
            policy = Exploration::PRM;
        else if(value == "Tovar")
            policy = Exploration::Tovar;
        else
            policy = Exploration::MCDM;
    } else if(varName == "poaret_bin") {
        poaretBinary = value;
    } //fill all the remaining variables!
}

void ConfigReader::setVariableNumber(const QString &varName, double value)
{
    if(varName == "usar_port") {
        usarPort = (quint16) value;
    } else if(varName == "upis_port") {
        upisPort = (quint16) value;
    } else if(varName == "wss_port") {
        wssPort = (quint16) value;
    } else if(varName == "robot_count") {
        robotCount = (int) value;
    } else if(varName == "h_min") {
        HMin = value;
    } else if(varName == "s_min") {
        SMin = value;
    } else if(varName == "v_min") {
        VMin = value;
    } else if(varName == "h_max") {
        HMax = value;
    } else if(varName == "s_max") {
        SMax = value;
    } else if(varName == "v_max") {
        VMax = value;
    }
}

void ConfigReader::setSLAMVariable(const QString &varName, const QString &varValue)
{
    SET_SLAM_VARIABLE(maxSpatialErrorPerMeter);
    SET_SLAM_VARIABLE(maxAngularErrorPerMeter);
    SET_SLAM_VARIABLE(maxSpatialErrorPerRadian);
    SET_SLAM_VARIABLE(maxAngularErrorPerRadian);

    SET_SLAM_VARIABLE(lookupThresholdElseberg);
    SET_SLAM_VARIABLE(lookupThresholdAmigoni);
    SET_SLAM_VARIABLE(lookupThresholdLiGriffiths);
    SET_SLAM_VARIABLE(lookupThresholdProbabilistic);
    SET_SLAM_VARIABLE(lookupThresholdPoseCentric);

    SET_SLAM_VARIABLE(broadLookupThresholdElseberg);
    SET_SLAM_VARIABLE(broadLookupThresholdAmigoni);
    SET_SLAM_VARIABLE(broadLookupThresholdLiGriffiths);
    SET_SLAM_VARIABLE(broadLookupThresholdProbabilistic);
    SET_SLAM_VARIABLE(broadLookupThresholdPoseCentric);

    SET_SLAM_VARIABLE(ransacMaximumIterations);
    SET_SLAM_VARIABLE(ransacMinimumIterations);

    SET_SLAM_VARIABLE(odometryMinVarianceX);
    SET_SLAM_VARIABLE(odometryMinVarianceY);
    SET_SLAM_VARIABLE(odometryMinVarianceTheta);

    SET_SLAM_VARIABLE(laserRangeVariance);
    SET_SLAM_VARIABLE(laserAngleVariance);
    SET_SLAM_VARIABLE(laserOutOfRange);

    SET_SLAM_VARIABLE(mergeThreshold);
    SET_SLAM_VARIABLE(splitThreshold);
    SET_SLAM_VARIABLE(collinearityThreshold);
    SET_SLAM_VARIABLE(thinningThreshold);

    SET_SLAM_VARIABLE(landmarkSpatialDistance);
    SET_SLAM_VARIABLE(landmarkAngularDistance);
}

void ConfigReader::setPRMVariable(const QString &varName, const QString &varValue)
{
    SET_PRM_VARIABLE(edgeThreshold);
    SET_PRM_VARIABLE(movementRadius);
    SET_PRM_VARIABLE(precision);
    SET_PRM_VARIABLE(pointNumber);
    SET_PRM_VARIABLE(pointNumberFrontier);
    SET_PRM_VARIABLE(maxPointDistance);
    SET_PRM_VARIABLE(pathNumber);
}

void ConfigReader::setOBSVariable(const QString &varName, const QString &varValue)
{
    SET_OBS_VARIABLE(obstacle_algorithm);
    SET_OBS_VARIABLE(emp_angle_tolerance);
    SET_OBS_VARIABLE(emp_sonar_threshold);
    SET_OBS_VARIABLE(emp_straight_meters);
    SET_OBS_VARIABLE(emp_back_meters);

    SET_OBS_VARIABLE(dwa_laser_threshold);
    SET_OBS_VARIABLE(dwa_laser_max_range);
    SET_OBS_VARIABLE(dwa_time);
    SET_OBS_VARIABLE(dwa_min_velocity);
    SET_OBS_VARIABLE(dwa_max_velocity);
    SET_OBS_VARIABLE(dwa_step);
    SET_OBS_VARIABLE(dwa_safety);
    SET_OBS_VARIABLE(dwa_pose_threshold);

    SET_OBS_VARIABLE(dwa_sigma);
    SET_OBS_VARIABLE(dwa_alpha_target);
    SET_OBS_VARIABLE(dwa_beta_clearance);
    SET_OBS_VARIABLE(dwa_gamma_velocity);
}


QString ConfigReader::nameFormat(const QString &name) const
{
    int start = 0, size = 0;
    QString varName = QString(name).replace("::", "/"), fieldName;
    QChar previous = 'A';

    for(QString::const_iterator it = varName.begin(), end = varName.end();
        it != end; ++it, size++) {
        if(it->isUpper()) {
            fieldName.append(varName.mid(start, size));
            if(previous.isLower()) {
                fieldName.append("_");
            }
            fieldName.append(it->toLower());
            start += size + 1;
            size = -1;
        }
        previous = *it;
    }
    if(size > 0) {
        fieldName.append(varName.mid(start));
    }
    return fieldName;
}

void ConfigReader::convertTo(const QString &strValue, int &value) const
{
    value = strValue.toInt();
}

void ConfigReader::convertTo(const QString &strValue, double &value) const
{
    value = strValue.toDouble();
}


}
