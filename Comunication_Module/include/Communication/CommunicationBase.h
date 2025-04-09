#ifndef COMMUNICATIONBASE_H
#define COMMUNICATIONBASE_H

#include <QObject>
#include "Base/Message.h"

//struct SendType
//{
//    QString prot;
//    QString objType;

//    inline bool operator==(const SendType & v1) const
//    {
//        if(v1.prot == prot && v1.objType == objType)
//            return true;
//        else
//            return false;
//    }
//};
//typedef struct SendType SendType;
//Q_DECLARE_METATYPE(SendType);



uint qHash(const SendType &key, uint seed = 0);

typedef struct
{
    QByteArray data;
    QByteArray check;
    QList<SendType> Dev;
    bool isCheck;
}SendPack;
Q_DECLARE_METATYPE(SendPack);

typedef struct
{
    SendType type;
    dataPack pack;
    quint32 ms;
}SendMesStr;
Q_DECLARE_METATYPE(SendMesStr);

class CommunicationBase : public QObject
{
    Q_OBJECT
public:
    explicit CommunicationBase(QString name,QObject *parent = nullptr);
    virtual void close() = 0;

signals:
    void sigReadyRead(SendType type,QByteArray data);
    void sigCommunicationError(SendType type,QString error);
    void sigInitError(SendType type,QString error);

protected:
    QString ZM_PortName;
};

#endif //
