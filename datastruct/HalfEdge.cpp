//
// Created by simar on 8/15/2023.
//

#include "HalfEdge.h"
#include "../intersect/MeshIntersect.h"
#include "trimesh_types.h"
#include "trimesh.h"

/**
 * loading the mesh into the class to generate the half edge data type
 * @param filePath the filepath where the mesh must be loaded into
 */
void HalfEdge::loadMesh(std::string filePath) {
    tris = load_obj<Scalar>(filePath); //Loads the triangle data to standard interative format
}

void HalfEdge::generateHalfEdge() {
    std::map<std::vector<long>, int> vertexIndexMap; // Map to store unique vertices and their indices
    std::vector<trimesh::triangle_t> triangles(tris.size());
    int vertexCount = 0;

    for (int i = 0; i < tris.size(); ++i) {
        // Add vertices to the unique vertices map and get their indices
        int v0_index = addVertex(vertexIndexMap, reinterpret_cast<const long *>(tris[i].p0.values), vertexCount);
        int v1_index = addVertex(vertexIndexMap, reinterpret_cast<const long *>(tris[i].p1.values), vertexCount);
        int v2_index = addVertex(vertexIndexMap, reinterpret_cast<const long *>(tris[i].p2.values), vertexCount);

        // Assign vertex indices to the triangle
        triangles[i].v[0] = v0_index;
        triangles[i].v[1] = v1_index;
        triangles[i].v[2] = v2_index;
    }

    trimesh::unordered_edges_from_triangles(triangles.size(), &triangles[0], edges);
    mesh.build(vertexCount, triangles.size(), &triangles[0], edges.size(), &edges[0]);
    kNumVertices = vertexCount; // Update the total number of vertices
}

int HalfEdge::addVertex(std::map<std::vector<long>, int>& vertexIndexMap, const long* values, int& vertexCount) {
    std::vector<long> vertex(values, values + 3);
    auto it = vertexIndexMap.find(vertex);
    if (it == vertexIndexMap.end()) {
        vertexIndexMap[vertex] = vertexCount;
        return vertexCount++;
    }
    return it->second;
}


void HalfEdge::walkMesh() {
    // Use 'mesh' to walk the connectivity.
    std::vector< trimesh::index_t > neighs;
    for( int vi = 0; vi < kNumVertices; ++vi )
    {
        mesh.vertex_vertex_neighbors( vi, neighs );

        std::cout << "neighbors of vertex " << vi << ": ";
        for( int i = 0; i < neighs.size(); ++i )
        {
            std::cout << ' ' << neighs.at(i);
        }
        std::cout << '\n';
    }
}




