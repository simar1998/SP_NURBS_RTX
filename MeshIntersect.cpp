//
// Created by simar on 6/13/2023.
//

#include "MeshIntersect.h"
#include "mesh_calc/vec.h"
#include <list>
#include <array>

void MeshIntersect::loadMesh(std::string filePath) {
    tris = load_obj<Scalar>(filePath);
}

int MeshIntersect::perform_intersect(bvh::v2::Ray<Scalar, 3> ray) {

    bvh::v2::ThreadPool thread_pool;
    bvh::v2::ParallelExecutor executor(thread_pool);

    // Get triangle centers and bounding boxes (required for BVH builder)
    std::vector<BBox> bboxes(tris.size());
    std::vector<Vec3> centers(tris.size());
    executor.for_each(0, tris.size(), [&] (size_t begin, size_t end) {
        for (size_t i = begin; i < end; ++i) {
            bboxes[i]  = tris[i].get_bbox();
            centers[i] = tris[i].get_center();
        }
    });

    typename bvh::v2::DefaultBuilder<Node>::Config config;
    config.quality = bvh::v2::DefaultBuilder<Node>::Quality::High;
    auto bvh = bvh::v2::DefaultBuilder<Node>::build(thread_pool, bboxes, centers, config);

    // Permuting the primitive data allows to remove indirections during traversal, which makes it faster.
    static constexpr bool should_permute = false;

    // This precomputes some data to speed up traversal further.
    std::vector<PrecomputedTri> precomputed_tris(tris.size());
    executor.for_each(0, tris.size(), [&] (size_t begin, size_t end) {
        for (size_t i = begin; i < end; ++i) {
            auto j = should_permute ? bvh.prim_ids[i] : i;
            precomputed_tris[i] = tris[j];
        }
    });

    static constexpr size_t invalid_id = std::numeric_limits<size_t>::max();
    static constexpr size_t stack_size = 64;
    static constexpr bool use_robust_traversal = false;

    auto prim_id = invalid_id;
    Scalar u, v;

    // Traverse the BVH and get the u, v coordinates of the closest intersection.
    bvh::v2::SmallStack<Bvh::Index, stack_size> stack;
    bvh.intersect<true, use_robust_traversal>(ray, bvh.get_root().index, stack,
                                               [&] (size_t begin, size_t end) {
                                                   for (size_t i = begin; i < end; ++i) {
                                                       size_t j = should_permute ? i : bvh.prim_ids[i];
                                                       if (auto hit = precomputed_tris[j].intersect(ray)) {
                                                           prim_id = i;
                                                           std::tie(u, v) = *hit;
                                                       }
                                                   }
                                                   return prim_id != invalid_id;
                                               });

    if (prim_id != invalid_id) {
        std::cout
                << "Intersection found\n"
                << "  primitive: " << prim_id << "\n"
                << "  distance: " << ray.tmax << "\n"
                << "  barycentric coords.: " << u << ", " << v << std::endl;
        return 0;
    } else {
        std::cout << "No intersection found" << std::endl;
        return 1;
    }

}
//TODO not working right, possibly wrong output
void MeshIntersect::planeIntersect(float z) {
    Vec3 planePoint(0.0f, 0.0f, z);
    Vec3 normal(0.0f, 0.0f, 1.0f);
    auto zPlane = Plane(planePoint, normal);
    auto epsVal = 1e-6f;
    std::list<Vec3> intersectionPoints;
    for (std::size_t i = 0; i < tris.size(); i++) {
        Tri tri = tris[i];
        //Computing triangle edges and their cross product
        auto edge1 = tri.p1 - tri.p0; //edge one vertex2 - vertex 1
        auto edge2 = tri.p2 - tri.p0;//Edge two is vertex 3 - vertex 1
        auto pvec = cross(zPlane.normal,edge2); //Cross of plane normal and edge2
        auto det = dot(edge1,pvec); // dot of edge1 and pvec
        //Checking if triangle is parralel to the plane
        if (abs(det) < epsVal){
            //std::cout << "No intersection between mesh and z plane" <<std::endl;
            continue;
        }
        auto invDet = 1 / det;
        //Calculating barycentric cords
        auto tvec = zPlane.point - tri.p0; // tvec plane point - vec1
        auto u = dot(tvec,pvec) * invDet; // U is dot of tvec and pvec * invdet
        //Checking if barycentric cords outside triangle
        if (u < 0 || u > 1)
        {
            //std::cout << "No intersection between mesh and z plane" <<std::endl;
            continue;
        }
        auto qvec = cross(tvec,edge1); //Qvec = corss of tvec and edge1
        auto v = dot(zPlane.normal,qvec) * invDet; // v dot of plane normal and qvec times invdet
        if (v < 0 || u + v > 1)
        {
            //std::cout << "No intersection between mesh and z plane" <<std::endl;
            continue;
        }
        auto t = dot(edge2,qvec) * invDet; // t is dot of edge2 and qvec times invDet
        if (t >=0){
           //Calculate point at triagnle for intersection
            auto pu = 1 - t;
            auto pv = t * ( 1- pu );
            auto pw = t * pu;
            auto point = pu * tri.p0 + pv * tri.p1 +  pw * tri.p2;
            std::cout << "Point calculated at : " << point.values[0] << "," << point.values[1] <<"," << point.values[2] << " For triangle num " << i << std::endl;
        }
    }
}


void MeshIntersect::print_triangles() {
    if (tris.size() > 0) {
        for (std::size_t i = 0; i < tris.size(); i++) {
            Tri tri = tris[i];
            std::cout << "P0 : [" << tri.p0.values[0] << ", " << tri.p0.values[1] << ", " << tri.p0.values[2] << "]"
                      << std::endl;
            std::cout << "P1 : [" << tri.p1.values[0] << ", " << tri.p1.values[1] << ", " << tri.p1.values[2] << "]"
                      << std::endl;
            std::cout << "P2 : [" << tri.p2.values[0] << ", " << tri.p2.values[1] << ", " << tri.p2.values[2] << "]"
                      << std::endl;
        }
    }
}

void MeshIntersect::getMinMax() {
    // Initialize min and max variables
    Vec3 minPoint(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
    Vec3 maxPoint(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());

    // Iterate over triangles
    for (std::size_t i = 0; i < tris.size(); i++) {
        Tri& tri = tris[i];

        // Update min and max values
        minPoint.values[0] = std::min(minPoint.values[0], std::min(tri.p0.values[0], std::min(tri.p1.values[0], tri.p2.values[0])));
        minPoint.values[1] = std::min(minPoint.values[1], std::min(tri.p0.values[1], std::min(tri.p1.values[1], tri.p2.values[1])));
        minPoint.values[2] = std::min(minPoint.values[2], std::min(tri.p0.values[2], std::min(tri.p1.values[2], tri.p2.values[2])));

        maxPoint = Vec3(std::max(maxPoint.values[0], std::max(tri.p0.values[0], std::max(tri.p1.values[0], tri.p2.values[0]))),
                        std::max(maxPoint.values[1], std::max(tri.p0.values[1], std::max(tri.p1.values[1], tri.p2.values[1]))),
                        std::max(maxPoint.values[2], std::max(tri.p0.values[2], std::max(tri.p1.values[2], tri.p2.values[2]))));
    }

    // Print or use the calculated min and max values
    std::cout << "Min Point: " << minPoint.values[0] << ", " << minPoint.values[1] << ", " << minPoint.values[2] << std::endl;
    std::cout << "Max Point: " << maxPoint.values[0] << ", " << maxPoint.values[1] << ", " << maxPoint.values[2] << std::endl;
}



