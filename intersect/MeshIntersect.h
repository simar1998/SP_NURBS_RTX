//
// Created by simar on 6/13/2023.
//

#ifndef TINYNURBS_MESHINTERSECT_H
#define TINYNURBS_MESHINTERSECT_H

#include "../mesh_calc/bvh.h"
#include "../mesh_calc/vec.h"
#include "../mesh_calc/ray.h"
#include "../mesh_calc/node.h"
#include "../mesh_calc/default_builder.h"
#include "../mesh_calc/thread_pool.h"
#include "../mesh_calc/executor.h"
#include "../mesh_calc/stack.h"
#include "../mesh_calc/tri.h"
#include "../mesh_calc/load_obj.h"

#include <iostream>
#include <list>




class MeshIntersect {
    using Scalar = float;
    using Vec3 = bvh::v2::Vec<Scalar, 3>;
    using BBox = bvh::v2::BBox<Scalar, 3>;
    using Tri = bvh::v2::Tri<Scalar, 3>;
    using Node = bvh::v2::Node<Scalar, 3>;
    using Bvh = bvh::v2::Bvh<Node>;
    using Ray = bvh::v2::Ray<Scalar, 3>;
    std::vector<Tri> tris;
    using PrecomputedTri = bvh::v2::PrecomputedTri<Scalar>;
    using Plane = bvh::v2::Plane<float,3>;
    const float overcastZvalOffset = 3.0f;
    const float offsetValXY = 1.0f;
    const float EPS = 0.000001f;



public:

    std::string filePath;

    enum Classification{
        OUTER,
        INNER,
    };

    struct zIntersectInfo{
        float layerHeight;
        int layerNum;
        Vec3 intersectPoint;
        int prim;
        Classification nature;
    };

    struct intersection{
        Ray originRay;
        float distance;
        int primitiveHit = 0;
        float u, v;
        float t;
        Vec3 intersectionPoint;

        friend std::ostream& operator<<(std::ostream& os, const intersection& inter) {
            os << "Distance: " << inter.distance << std::endl;
            os << "Primitive Hit: " << inter.primitiveHit << std::endl;
            os << "u: " << inter.u << std::endl;
            os << "v: " << inter.v << std::endl;
            os << "t: " << inter.t << std::endl;
            os << "Intersection Point: (" << inter.intersectionPoint.values[0] << ", "
               << inter.intersectionPoint.values[1] << ", " << inter.intersectionPoint.values[2] << ")" << std::endl;
            return os;
        }
    };

    float calculateDistance(Vec3 point1, Vec3 point2) {
        float dx = point2.values[0] - point1.values[0];
        float dy = point2.values[1] - point1.values[1];
        float dz = point2.values[2] - point1.values[2];
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    void loadMesh(std::string filePath);
    std::vector<MeshIntersect::intersection> perform_intersect(bvh::v2::Ray<Scalar, 3> ray);
    void print_triangles();

    std::vector<Vec3> planeIntersect(float z, bool printOut = false);

    std::vector<Vec3>  getMinMax(bool printOutput = false);

    void planeIntersect2(float z);

    bool isPointInMesh(Vec3 point);

    std::vector<std::vector<Vec3>> generateOvercastRayField();
    std::vector<std::vector<Vec3>> generateLinOvercastRayField(float samplingDist);
    std::vector<std::vector<MeshIntersect::intersection>> gridPlaneIntersect(std::vector<std::vector<Vec3>> gridPlane);
    Vec3 computeVecPoint(MeshIntersect::intersection intersection);
    std::vector<MeshIntersect::intersection> gridPlaneIntersectMollerTrombore(std::vector<std::vector<Vec3>> gridPlane);
    intersection rayIntersect(Ray& ray);

    std::vector<intersection> gridPlaneIntersectSimple(std::vector<std::vector<Vec3>> gridPlane);
    std::vector<MeshIntersect::intersection> mollerTromboreRayIntersect(Ray ray);

    std::vector<zIntersectInfo> zPlaneIntersect(float z, bool printOut);

    std::vector<zIntersectInfo> zPlaneIntersect(float z, int layerNum);
};


#endif //TINYNURBS_MESHINTERSECT_H
