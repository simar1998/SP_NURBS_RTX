//
// Created by simar on 7/20/2023.
//

#ifndef TINYNURBS_ITERATIVELAYERSAMPLNG_H
#define TINYNURBS_ITERATIVELAYERSAMPLNG_H


#include "../mesh_calc/vec.h"
#include "../intersect/MeshIntersect.h"

class IterativeLayerSamplng {
    using Scalar = float;
    using Vec3 = bvh::v2::Vec<Scalar, 3>;


public:
    std::vector<MeshIntersect::zIntersectInfo> performIntersections(float layerHeight, MeshIntersect mesh);
};


#endif //TINYNURBS_ITERATIVELAYERSAMPLNG_H
