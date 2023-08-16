//
// Created by Simar on 8/3/2023.
//

#include "BooleanOperation.h"

BooleanOperation::BooleanOperation(const std::vector<Tri> &mesh1, const std::vector<Tri> &mesh2) : mesh1(mesh1),
                                                                                                   mesh2(mesh2) {}

std::vector<BooleanOperation::Tri> BooleanOperation::add() {
    std::vector<Tri> result = mesh1;
    result.insert(result.end(), mesh2.begin(), mesh2.end());

    // Iterate over the result and check for intersections
    for (auto it1 = result.begin(); it1 != result.end(); ++it1) {
        for (auto it2 = it1 + 1; it2 != result.end(); ) {
            if (intersects(*it1, *it2)) {
                // If the triangles intersect, remove them from the result
                it1 = result.erase(it1);
                it2 = result.erase(it2);
            } else {
                ++it2;
            }
        }
    }

    return result;
}

//Trianlge triangle  moller trombore intersect
bool BooleanOperation::intersects(const Tri& tri1, const Tri& tri2) {
    // Define a function to compute the cross product of two vectors
    auto cross = [](const Vec3& u, const Vec3& v) -> Vec3 {
        return Vec3(u[1] * v[2] - u[2] * v[1],
                    u[2] * v[0] - u[0] * v[2],
                    u[0] * v[1] - u[1] * v[0]);
    };

    // Define a function to compute the dot product of two vectors
    auto dot = [](const Vec3& u, const Vec3& v) -> Scalar {
        return u[0] * v[0] + u[1] * v[1] + u[2] * v[2];
    };

    // Check if the ray formed by one edge of tri1 intersects tri2
    auto rayIntersectsTriangle = [&](const Vec3& rayOrg, const Vec3& rayDir, const Tri& tri) -> bool {
        Vec3 edge1 = tri.p1 - tri.p0;
        Vec3 edge2 = tri.p2 - tri.p0;
        Vec3 pvec = cross(rayDir, edge2);
        Scalar det = dot(edge1, pvec);

        if (det < 0.00001) {
            return false;
        }

        Vec3 tvec = rayOrg - tri.p0;
        Scalar u = dot(tvec, pvec);

        if (u < 0.0 || u > det) {
            return false;
        }

        Vec3 qvec = cross(tvec, edge1);
        Scalar v = dot(rayDir, qvec);

        if (v < 0.0 || u + v > det) {
            return false;
        }

        Scalar t = dot(edge2, qvec);
        Scalar inv_det = 1.0 / det;

        t *= inv_det;

        return t > 0.00001;
    };

    // Check if any edge of tri1 intersects tri2
    for (int i = 0; i < 3; ++i) {
        Vec3 rayOrg = Vec3(tri1.p0.values[0],tri1.p0.values[1],tri1.p0.values[2]);
        Vec3 rayDir;
        if (i == 0) {
            rayDir = Vec3(tri1.p1.values[0], tri1.p1.values[1], tri1.p1.values[2]) - rayOrg;
        } else if (i == 1) {
            rayDir = Vec3(tri1.p2.values[0], tri1.p2.values[1], tri1.p2.values[2]) - Vec3(tri1.p1.values[0], tri1.p1.values[1], tri1.p1.values[2]);
        } else {
            rayDir = rayOrg - Vec3(tri1.p2.values[0], tri1.p2.values[1], tri1.p2.values[2]);
        }

        if (rayIntersectsTriangle(rayOrg, rayDir, tri2)) {
            return true;
        }
    }

    return false;
}

std::vector<BooleanOperation::Tri> BooleanOperation::subtract() {
    return std::vector<Tri>();
}

