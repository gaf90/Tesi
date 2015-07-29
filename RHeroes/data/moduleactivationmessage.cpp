#include "moduleactivationmessage.h"

namespace Data{

ModuleActivationMessage::ModuleActivationMessage():
    module(""),
    mustBeActive(true)
{
}

ModuleActivationMessage::ModuleActivationMessage(QString module, bool mustBeActive):
    module(module),
    mustBeActive(mustBeActive)
{
}

ModuleActivationMessage::~ModuleActivationMessage()
{
}

QString ModuleActivationMessage::getInvolvedModule() const
{
    return module;
}

bool ModuleActivationMessage::getModuleStatus() const
{
    return mustBeActive;
}

void ModuleActivationMessage::serializeTo(QDataStream &stream) const
{
    stream << module << mustBeActive;
}

void ModuleActivationMessage::deserializeFrom(QDataStream &stream)
{
    stream >> module >> mustBeActive;
}

}

