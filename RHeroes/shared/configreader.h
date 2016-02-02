#ifndef CONFIGREADER_H
#define CONFIGREADER_H

#include <QObject>
#include <QFile>

namespace Config{

/**
  * the class implements a reader for the configurations
  * of the single robots.
  */
class ConfigReader : public QObject
{
    Q_OBJECT
public:
    /**
      * Constructor for the ConfigReader.
      * @param filePath of the configuration file.
      */
    explicit ConfigReader(const QString &filePath, QObject *parent = 0);

    /**
      * Destroyer for the class
      */
    virtual ~ConfigReader();

    /**
      * Method that reads the config file and then sets the variable
      * in the shared configurations.
      * @return -1 if the file can not be opend; -2 if there's a syntax error. 0 if everything goes right.
      */
    int readFileAndCompileConfigs();
    
signals:
    
public slots:

private:
    void setVariable(const QString &varName, const QString &varValue);
    void setVariableString(const QString &varName, const QString &value);
    void setVariableNumber(const QString &varName, double value);
    void setSLAMVariable(const QString &varName, const QString &varValue);
    void setPRMVariable(const QString &varName, const QString &varValue);
    void setOBSVariable(const QString &varName, const QString &varValue);

    void convertTo(const QString &strValue, int &value) const;
    void convertTo(const QString &strValue, double &value) const;

    QString nameFormat(const QString &name) const;

    QString filePath;
    
};

}

#endif // CONFIGREADER_H
