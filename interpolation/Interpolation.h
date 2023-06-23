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

};


#endif //TINYNURBS_INTERPOLATION_H
