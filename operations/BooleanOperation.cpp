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

void BooleanOperation::preprocess(std::vector<Tri> &mesh1, std::vector<Tri> &mesh2) {
    // Remove duplicate vertices within a certain tolerance
    removeDuplicateVertices(mesh1);
    removeDuplicateVertices(mesh2);

    // Handle degenerate triangles (e.g., triangles with zero area)
    handleDegenerateTriangles(mesh1);
    handleDegenerateTriangles(mesh2);

    // Additional preprocessing steps as needed
    // ...
}

void BooleanOperation::removeDuplicateVertices(std::vector<Tri> &mesh) {
    // Define a tolerance for considering vertices as duplicates
    const Scalar tolerance = 1e-6;

    // Use a map to store unique vertices and their corresponding index
    std::unordered_map<Vec3, int, Vec3Hasher> uniqueVertices;

    // Iterate through the triangles and replace duplicate vertices
    for (Tri &tri : mesh) {
        for (Vec3 &vertex : {tri.p0, tri.p1, tri.p2}) {
            auto it = uniqueVertices.find(vertex);
            if (it != uniqueVertices.end()) {
                // Replace the vertex with the index of the unique vertex
                vertex = it->second;
            } else {
                // Add the vertex to the map of unique vertices
                uniqueVertices[vertex] = vertex;
            }
        }
    }
}

void BooleanOperation::handleDegenerateTriangles(std::vector<Tri> &mesh) {
    // Iterate through the triangles and remove or handle degenerate triangles
    mesh.erase(std::remove_if(mesh.begin(), mesh.end(), [](const Tri &tri) {
        // Compute the area of the triangle
        Vec3 crossProduct = cross(tri.p1 - tri.p0, tri.p2 - tri.p0);
        Scalar area = 0.5 * length(crossProduct);

        // Consider the triangle as degenerate if its area is close to zero
        return area < 1e-6;
    }), mesh.end());
}

// Define a custom hasher for Vec3 if needed
struct Vec3Hasher {
    std::size_t operator()(const Vec3 &vec) const {
        // Combine the hash values of the individual components
        return std::hash<Scalar>()(vec[0]) ^ std::hash<Scalar>()(vec[1]) ^ std::hash<Scalar>()(vec[2]);
    }
};

double VOLUME3D(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& p) {
    // Compute the volume of the tetrahedron formed by points a, b, c, and p
    return dot(cross(b - a, c - a), p - a) / 6.0;
}

Vec3 computeIntersection(const Vec3& Q1, const Vec3& Q2, const Triangle& t2) {
    Vec3 N = cross(t2.p1 - t2.p0, t2.p2 - t2.p0); // Normal of triangle t2
    double d = dot(N, t2.p0); // Distance from origin to the plane of triangle t2
    double t = (d - dot(N, Q1)) / dot(N, Q2 - Q1); // Parameter for line equation
    return Q1 + t * (Q2 - Q1); // Intersection point
}

double triangleArea(const Vec3& a, const Vec3& b, const Vec3& c) {
    return 0.5 * length(cross(b - a, c - a));
}

bool isCommonEdge(const Triangle& t1, const Triangle& t2, const Vec3& v1, const Vec3& v2) {
    double area1 = triangleArea(t2.p0, t2.p1, v1);
    double area2 = triangleArea(t2.p0, t2.p1, v2);
    return area1 + area2 == triangleArea(t2.p0, t2.p1, t2.p2);
}