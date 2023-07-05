//
// Created by simar on 7/1/2023.
//

#ifndef TINYNURBS_POINTITERATION_H
#define TINYNURBS_POINTITERATION_H


#include "../mesh_calc/vec.h"
#include "../mesh_calc/tri.h"
#include "../simple_example.h"

class PointIteration {

    using Scalar = float;
    using Vec3 = bvh::v2::Vec<Scalar,3>;
    using Tri = bvh::v2::Tri<Scalar, 3>;
    using Bvh = bvh::v2::Bvh<Node>;
    using Ray = bvh::v2::Ray<Scalar, 3>;
    std::vector<Tri> tris;


public:

    Vec3 getInitPoint();

    std::vector<Vec3> initPointIteration(std::string filePath);
};


#endif //TINYNURBS_POINTITERATION_H
