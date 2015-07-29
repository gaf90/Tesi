#ifndef TOPOLOGICALMAP_H
#define TOPOLOGICALMAP_H
#include "semanticMapping/semantichandler.h"

namespace SemanticMapping{
    namespace SemanticData{
class TopologicalMap
{
public:
    TopologicalMap(SemanticHandler *handRef);
private:
    SemanticHandler *handler;
};
    }
}
#endif // TOPOLOGICALMAP_H
