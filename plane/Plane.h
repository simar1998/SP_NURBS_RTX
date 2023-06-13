//
// Created by simar on 6/13/2023.
//

#ifndef TINYNURBS_PLANE_H
#define TINYNURBS_PLANE_H

//representing a plane equation tied to z axis and spanning X and Y

struct Vector3
{
    float x;
    float y;
    float z;
};

class Plane
{
public:
    Plane(); // Default constructor
    Plane(const Vector3& point, const Vector3& normal); // Parameterized constructor

    Vector3 GetPoint() const;
    Vector3 GetNormal() const;

private:
    Vector3 point_;
    Vector3 normal_;
};


#endif //TINYNURBS_PLANE_H
