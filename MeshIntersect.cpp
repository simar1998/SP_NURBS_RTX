//
// Created by simar on 6/13/2023.
//

#include "MeshIntersect.h"

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


