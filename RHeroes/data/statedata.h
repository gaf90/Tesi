#ifndef STATEDATA_H
#define STATEDATA_H

#include "message.h"

namespace Data{
    class StateData : public Message
    {
    public:
        StateData();
        StateData(const QString &vehicleType, float time, float frontSteer,
                  float rearSteer, bool lightToggle, int lightIntensity, int battery);
        virtual ~StateData();

        void setVehicleType(const QString &vehicleType);
        void setTime(float time);
        void setFrontSteer(float frontSteer);
        void setRearSteer(float rearSteer);
        void setLightToggle(bool lightToggle);
        void setLightIntensity(int lightIntensity);
        void setBattery(int battery);

        QString getVehicleType() const;
        float getTime() const;
        float getFrontSteer() const;
        float getRearSteer() const;
        bool getLightToggle() const;
        int getLightIntensity() const;
        int getBattery() const;

    private:
        QString vehicleType;
        float time, frontSteer, rearSteer;
        bool lightToggle;
        int lightIntensity, battery;

    };
}

#endif // STATEDATA_H
