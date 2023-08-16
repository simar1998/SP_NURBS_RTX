//
// Created by simar on 8/15/2023.
//

#ifndef SCULTPATH_HALFEDGE_H
#define SCULTPATH_HALFEDGE_H


#include "../mesh_calc/vec.h"
#include "../mesh_calc/tri.h"
#include "trimesh_types.h"
#include "trimesh.h"

class HalfEdge {
    using Scalar = float;
    using Vec3 = bvh::v2::Vec<Scalar, 3>;
    using Tri = bvh::v2::Tri<Scalar, 3>;
    std::vector<Tri> tris;

public:
    std::vector<trimesh::edge_t > edges;
    trimesh::trimesh_t mesh;
    int kNumVertices;
    void loadMesh(std::string filePath);
    void generateHalfEdge();
    void walkMesh();

    int addVertex(std::map<std::vector<long>, int> &vertexIndexMap, const long *values, int &vertexCount);
};


#endif //SCULTPATH_HALFEDGE_H
