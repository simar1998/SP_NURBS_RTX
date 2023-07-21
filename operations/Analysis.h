//
// Created by simar on 7/21/2023.
//

#ifndef TINYNURBS_ANALYSIS_H
#define TINYNURBS_ANALYSIS_H


#include "../mesh_calc/vec.h"
#include "../intersect/MeshIntersect.h"

class Analysis {

    using Scalar = float;
    using Vec3 = bvh::v2::Vec<Scalar, 3>;
    using Tri = bvh::v2::Tri<Scalar, 3>;

private:
    std::vector<float> embedTrianglesInHilbertSpace(const std::vector<Tri>& triangles);
    float gaussianRBFKernel(float distance, float sigma);
    float calculateCurvature(const Tri& triangle);

public:
    void performAnalysis(MeshIntersect mesh);

    std::vector<int> kmeansClustering(const std::vector<float> &embeddings, int k, int maxIterations);
};


#endif //TINYNURBS_ANALYSIS_H
