//
// Created by simar on 7/4/2023.
//

#ifndef TINYNURBS_OFFSETGENERATOR_H
#define TINYNURBS_OFFSETGENERATOR_H


#include "../mesh_calc/vec.h"
#include "../intersect/MeshIntersect.h"

class OffsetGenerator {
    using Scalar = float;
    using Vec3 = bvh::v2::Vec<Scalar, 3>;
public:
    static std::vector<Vec3> ParallelTransportMethod(std::vector<Vec3> points, float offsetDistance);
};


#endif //TINYNURBS_OFFSETGENERATOR_H
