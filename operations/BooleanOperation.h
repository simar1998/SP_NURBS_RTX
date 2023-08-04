//
// Created by Simar on 8/3/2023.
//

#ifndef TINYNURBS_BOOLEANOPERATION_H
#define TINYNURBS_BOOLEANOPERATION_H


#include "../mesh_calc/tri.h"

class BooleanOperation {
private:
    using Scalar = float;
    using Vec3 = bvh::v2::Vec<Scalar, 3>;
    using Tri = bvh::v2::Tri<Scalar, 3>;
    //Mesh object represents two meshes
    std::vector<Tri> mesh1;
    std::vector<Tri> mesh2;
public:
    BooleanOperation(const std::vector<Tri> &mesh1, const std::vector<Tri> &mesh2);

    std::vector<Tri> add();
    std::vector<Tri> subtract();

    bool intersects(const Tri &tri1, const Tri &tri2);
};


#endif //TINYNURBS_BOOLEANOPERATION_H
