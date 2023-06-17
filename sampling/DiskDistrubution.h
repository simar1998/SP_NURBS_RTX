//
// Created by simar on 6/16/2023.
//

#ifndef TINYNURBS_DISKDISTRUBUTION_H
#define TINYNURBS_DISKDISTRUBUTION_H
#include "../mesh_calc/bvh.h"
#include "../mesh_calc/tri.h"

class DiskDistrubution {
    using Scalar = float;
    using Tri = bvh::v2::Tri<Scalar, 3>;
public:
    void diskDisturbution(std::vector<Tri> tris);
};


#endif //TINYNURBS_DISKDISTRUBUTION_H
