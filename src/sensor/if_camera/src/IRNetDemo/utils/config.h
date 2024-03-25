#ifndef CONFIG_H
#define CONFIG_H

#include <QVariant>
#include <QSettings>

class Config
{

public:
    static const QString NODE_DEVICE;
    static const QString IP_KEY;
    static const QString MODEL_KEY;


public:
    Config(QString qstrfilename = "");
    virtual ~Config(void);
    void Set(QString,QString,QVariant);
    QVariant Get(QString,QString);
private:
    QString m_qstrFileName;
    QSettings *m_psetting;
};

#endif // CONFIG_H
