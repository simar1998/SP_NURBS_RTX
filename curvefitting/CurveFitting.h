//
// Created by simar on 6/26/2023.
//

#ifndef TINYNURBS_CURVEFITTING_H
#define TINYNURBS_CURVEFITTING_H


#include "../mesh_calc/vec.h"
#include "../include/tinynurbs/tinynurbs.h"

class CurveFitting {
using Scalar = float;
using Vec3 = bvh::v2::Vec<Scalar, 3>;
using curv = tinynurbs::Curve<float>;
using GlmVec = glm::vec3;

private:
    float computeBasisFunction(int degree, const std::vector<float>& knotVector, int knotIndex, float parameter);
public:
    curv fitCurve(const std::vector<Vec3> &points);
};


#endif //TINYNURBS_CURVEFITTING_H
