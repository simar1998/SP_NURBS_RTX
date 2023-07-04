//
// Created by simar on 6/23/2023.
//

#ifndef TINYNURBS_INTERPOLATION_H
#define TINYNURBS_INTERPOLATION_H

#include <iostream>
#include "../mesh_calc/vec.h"
#include "../mesh_calc/ray.h"

class Interpolation {

    using Scalar = float;
    using Vec3 = bvh::v2::Vec<Scalar, 3>;

public:
    std::vector<Vec3> createInterpolatedPoints(std::vector<Vec3> points);

private:
    // Calculate the Euclidean distance between two points
    float calculateDistance(Vec3 point1, Vec3 point2) {
        float dx = point2.values[0] - point1.values[0];
        float dy = point2.values[1] - point1.values[1];
        float dz = point2.values[2] - point1.values[2];
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

};


#endif //TINYNURBS_INTERPOLATION_H
