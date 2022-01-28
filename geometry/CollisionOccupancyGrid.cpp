#include "CollisionOccupancyGrid.h"

using namespace Geometry;

bool Geometry3DOccupancyGrid::ConvertFrom(const Geometry3D* geom,Real param,Real domainExpansion) 
{
    if(geom->GetType() == Type::Primitive) {
        auto* isurf = geom->ConvertTo(Type::ImplicitSurface, param, domainExpansion);
        ConvertFrom(isurf);
        delete isurf;
        return true;
    }
    return false;
}
