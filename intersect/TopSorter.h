//
// Created by simar on 7/19/2023.
//

#ifndef TINYNURBS_TOPSORTER_H
#define TINYNURBS_TOPSORTER_H


#include "../mesh_calc/vec.h"
#include "MeshIntersect.h"

class TopSorter {

    using Scalar = float;
    using Vec3 = bvh::v2::Vec<Scalar, 3>;

public:
    std::vector<std::vector<Vec3>> sortTopClustering(std::vector<MeshIntersect::intersection>& intersections);
};


#endif //TINYNURBS_TOPSORTER_H
