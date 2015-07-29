#include "aircontroldriver.h"
#include "data/airdrivemessage.h"
#include "data/usarmessage.h"
#include <typeinfo>
#include "shared/utilities.h"

namespace Middleware{

using namespace Data;

AirControlDriver::AirControlDriver(QObject *parent):
    Driver(parent)
{
}

AirControlDriver::~AirControlDriver()
{
}

void AirControlDriver::onDriverMessage(const Message &message)
{
    //DRIVE {AltitudeVelocity 0} {LinearVelocity 0} {LateralVelocity 0} {RotationalVelocity 1.5}
    if(typeid(message) == typeid(const AirDriveMessage &)) {
        const AirDriveMessage &airDrive = (const AirDriveMessage &) message;
        USARMessage command;
        command.setType("DRIVE");
        command["AltitudeVelocity"] = QString::number(airDrive.getAltitudeSpeed());
        command["LinearVelocity"] = QString::number(airDrive.getLinearSpeed());
        command["LateralVelocity"] = QString::number(airDrive.getLateralSpeed());
        command["RotationalVelocity"] = QString::number(airDrive.getRotationalSpeed());
        ldbg << "sent air-teleoperation command to usarsim: " << command << endl;
        emit sigDriveMessageSend(command);
    }
}

}
