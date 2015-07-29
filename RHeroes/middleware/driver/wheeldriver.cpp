#include "wheeldriver.h"
#include "data/wheelmessage.h"
#include "data/usarmessage.h"
#include <typeinfo>
#include <QDebug>

namespace Middleware {

using namespace Data;

WheelDriver::WheelDriver(QObject *parent) :
    Driver(parent)
{
}

WheelDriver::~WheelDriver()
{
}

void WheelDriver::onDriverMessage(const Message &message)
{
    if(typeid(message) == typeid(const WheelMessage &)) {
        const WheelMessage &wheel = (const WheelMessage &) message;
        USARMessage command;
        command.setType("DRIVE");
        command["Left"] = QString::number(wheel.getLeftWheelSpeed());
        command["Right"] = QString::number(wheel.getRightWheelSpeed());
        //qDebug() << "Left: " << wheel.getLeftWheelSpeed() << " Right: " << wheel.getRightWheelSpeed();
        emit sigDriveMessageSend(command);
    }
}

}
