#ifndef SEMANTICMAPPINGMODULE_H
#define SEMANTICMAPPINGMODULE_H

#include <QObject>
#include "semanticMapping/room.h"
#include "semanticMapping/semantichandler.h"

namespace SemanticMapping{
    class SemanticMappingModule : public QObject
    {
        Q_OBJECT
    public:
        explicit SemanticMappingModule(QObject *parent = 0);

        /**
          * @returns The list of all the rooms;
          */
        QList<Room> *getRoomList();

        /**
          * Find the room where the given point is; returns the semantic informations about that room;
          * @param the point to be checked;
          * @returns the infos of the room where the given point is;
          *          if the point is outside of all the rooms it returns NULL;
          */
        semanticInfos *getSemanticInfos(SLAM::Geometry::Point point);

        /**
          * Activate/deactivate the Semantic Mapping Module;
          * @param youarecrueldisactivatingme True->activate module; False ->deactivate;
          * @returns if the module is active or not;
          */
        bool changeStatus(bool youarecrueldisactivatingme);

        /**
          * On/Off switch for the Semantic Mapping Module;
          * @returns if the module is active or not;
          */
        bool switchActive();

    signals:

    public slots:
        void semanticUpdater(SLAM::Map map);

    private:
    bool hasdata, active;
    int counter;
    SemanticMapping::SemanticHandler *handler;
    };
}

#endif // SEMANTICMAPPINGMODULE_H
