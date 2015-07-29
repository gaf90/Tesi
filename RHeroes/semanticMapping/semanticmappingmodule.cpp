#include "semanticmappingmodule.h"

namespace SemanticMapping{
    SemanticMappingModule::SemanticMappingModule(QObject *parent):
        QObject(parent),hasdata(false),active(true),counter(0),handler(NULL)
    {
    }
//    semanticInfos *SemanticMappingModule::getSemanticInfos(SLAM::Geometry::Point point)
//    {
//        bool hasconnection = true;
//        if (hasconnection) {
//            if ((t + MAX_TIME_INTERVAL < Timestampattuale) && (hasdata) || (!localData))
//            {
//                return handler.getSemanticInfos( point );
//            }
//            else
//            {
//            copymap = getnewmap;
//            localdata = false;
//            hasdata = true;
//            t = settimestamp;
//            //TODO QUI DEVO DEALLOCARE HANDLER!
//            delete handler;
//            handler = new SemanticMappingModule::semantichandler( copymap);
//            return handler.getSemanticInfos( point );
//            }
//        }
//        else
//        {
//            if(hasdata)
//                if (handler != null)
//                    if ((t + MAX_TIME_INTERVAL < Timestampattuale) || (!localData)
//                            || (! ispossibletogetlocalmap))
//                        return handler.getSemanticInfos( point );
//                    else
//                    {
//                        copymap = getLocalMap();
//                        hasdata = true;
//                        localData = true;
//                        t = settimestamp;
//                        delete handler;
//                        handler = new SemanticMappingModule::semantichandler(copymap);
//                        return handler.getSemanticInfos(point);
//                    }
//                else error;
//            else
//            {
//                if (ispossibletogetlocalmap)
//                {
//                    copymap = getLocalMap();
//                    hasdata = true;
//                    localData = true;
//                    t = settimestamp;
//                    delete handler;
//                    handler = new SemanticMappingModule::semantichandler(copymap);
//                    return handler.getSemanticInfos(point);
//                }
//            }
//        }
//    }
    void SemanticMappingModule::semanticUpdater(SLAM::Map map)
    {
        hasdata = true;
        counter++;
        if ((counter%65 == 15) && (active)) {
            //TODO Come cancello il vecchio handler?
            delete handler;
            handler =  new SemanticMapping::SemanticHandler(&map);
        }
    }

    semanticInfos *SemanticMappingModule::getSemanticInfos(SLAM::Geometry::Point point)
    {
        if ((hasdata) && (active))
        {
            SemanticMapping::semanticInfos* temp = handler->getSemanticInfos(point);
            return temp;
        }
        else
            return NULL;
    }

    QList<Room> *SemanticMappingModule::getRoomList()
    {
        if ((hasdata) && (active))
        {
            QList <SemanticMapping::Room> *list = new QList<SemanticMapping::Room> (handler->getRooms());
            return list;
        }
        else
            return NULL;
    }
    bool SemanticMappingModule::changeStatus(bool youarecrueldisactivatingme)
    {
        active = youarecrueldisactivatingme;
        return active;
    }
    bool SemanticMappingModule::switchActive()
    {
        active = !active;
        return active;
    }
}
