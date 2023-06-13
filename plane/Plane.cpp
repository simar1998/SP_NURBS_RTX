//
// Created by simar on 6/13/2023.
//

#include "Plane.h"

Plane::Plane() : point_{0.0f, 0.0f, 0.0f}, normal_{0.0f, 0.0f, 0.0f}
{
}

Plane::Plane(const Vector3& point, const Vector3& normal) : point_(point), normal_(normal)
{
}

Vector3 Plane::GetPoint() const
{
    return point_;
}

Vector3 Plane::GetNormal() const
{
    return normal_;
}
