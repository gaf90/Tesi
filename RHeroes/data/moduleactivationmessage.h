#ifndef MODULEACTIVATIONMESSAGE_H
#define MODULEACTIVATIONMESSAGE_H

#include "data/serializable.h"

namespace Data {


/**
 * @brief sigle module activation control message
 *
 * ModuleActivationMessage is used to enable/disable single omules of the target robot.
 * It must be handle by the robot and the proper module must be disabled, without
 * affecting critically others modules
 *
 * @see
 */
class ModuleActivationMessage : public Serializable
{
public:
    ModuleActivationMessage();

   /**
     * Initializes a ModuleActivationMessage with the module name and
     * operation to be executed
     *
     * @param module the module to be managed
     * @param mustBeActive if the robot must be activated (true) or disactivated (false)
     */
    ModuleActivationMessage(QString module, bool getModuleStatus);

    /**
     * Destroys the ModuleActivationMessage
     */
    virtual ~ModuleActivationMessage();

    /**
     * @return module to be activated/disactivated
     */
    QString getInvolvedModule() const;

    /**
     * @return boolean that specifies if the module must be activated (true) or
     * disactivated (false)
     */
    bool getModuleStatus() const;

    virtual void serializeTo(QDataStream &stream) const;
    virtual void deserializeFrom(QDataStream &stream);

private:
    QString module;
    bool mustBeActive;
};

}


#endif // MODULEACTIVATIONMESSAGE_H
