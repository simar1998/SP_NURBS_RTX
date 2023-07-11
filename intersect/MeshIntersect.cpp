//
// Created by simar on 6/13/2023.
//

#include "MeshIntersect.h"
#include <list>
#include <array>
#include "../clipper2/clipper.h"
#include "../simple_example.h"
#include "../interpolation/Interpolation.h"
void MeshIntersect::loadMesh(std::string filePath) {
    tris = load_obj<Scalar>(filePath);
}

//Ray intersect builder
//TODO chage output to object for ease of usage
float MeshIntersect::perform_intersect(bvh::v2::Ray<Scalar, 3> ray) {

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
        return ray.tmax;
    } else {
        std::cout << "No intersection found" << std::endl;
        return -1;
    }

}
//TODO Deprecated , stored for historical purposes
void MeshIntersect::planeIntersect2(float z) {
    Vec3 planePoint(0.0f, 0.0f, z);
    Vec3 normal(0.0f, 0.0f, 1.0f);
    auto zPlane = Plane(planePoint, normal);
    auto epsVal = 1e-6f;
    std::list<Vec3> intersectionPoints;
    print_triangles();
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

//Very weird return tupe import from simple_eample.h for some reason it doesnot want to use the using statement from MeshIntersect.h, it works so I have left it that way
//Probably a simple fix
//This gives good values for plane intersect
std::vector<Vec3>  MeshIntersect::planeIntersect(float z, bool printOut) {
    Vec3 planePoint(0.0f, 0.0f, z);
    Vec3 normal(0.0f, 0.0f, 1.0f);
    Plane zPlane(planePoint, normal);
    std::vector<Vec3> intersectionPoints;
    for (std::size_t i = 0; i < tris.size(); i++) {
        Tri tri = tris[i];
        Vec3 edges[3] = {tri.p1 - tri.p0, tri.p2 - tri.p1, tri.p0 - tri.p2}; //edges of triangle
        Vec3 points[3] = {tri.p0, tri.p1, tri.p2}; //points of triangle
        for (int j = 0; j < 3; j++){
            Vec3 direction = edges[j];
            Vec3 origin = points[j];
            float denom = dot(zPlane.normal, direction);
            if (std::abs(denom) > 1e-6) { //check if line is not parallel to plane by comparing to eps value
                float t = dot(zPlane.point - origin, zPlane.normal) / denom;
                if (t >= 0 && t <= 1) { //check if intersection is on the line segment (edge)
                    Vec3 intersection = origin + direction * t;
                    intersectionPoints.push_back(intersection);
                    if (printOut) {
                    std::cout << "Point calculated at : " << intersection.values[0] << "," << intersection.values[1]
                              << "," << intersection.values[2] << " For triangle num " << i << std::endl;
                    }
                }
            }
        }
    }
    return intersectionPoints;
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
//Might not be usefull as BVH class has robust min and max
std::vector<Vec3> MeshIntersect::getMinMax(bool printOutput) {
    // Initialize min and max variables
    Vec3 minPoint(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
                  std::numeric_limits<float>::max());
    Vec3 maxPoint(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(),
                  -std::numeric_limits<float>::max());

    // Iterate over triangles
    for (std::size_t i = 0; i < tris.size(); i++) {
        Tri &tri = tris[i];

        // Update min and max values
        minPoint.values[0] = std::min(minPoint.values[0],
                                      std::min(tri.p0.values[0], std::min(tri.p1.values[0], tri.p2.values[0])));
        minPoint.values[1] = std::min(minPoint.values[1],
                                      std::min(tri.p0.values[1], std::min(tri.p1.values[1], tri.p2.values[1])));
        minPoint.values[2] = std::min(minPoint.values[2],
                                      std::min(tri.p0.values[2], std::min(tri.p1.values[2], tri.p2.values[2])));

        maxPoint = Vec3(
                std::max(maxPoint.values[0], std::max(tri.p0.values[0], std::max(tri.p1.values[0], tri.p2.values[0]))),
                std::max(maxPoint.values[1], std::max(tri.p0.values[1], std::max(tri.p1.values[1], tri.p2.values[1]))),
                std::max(maxPoint.values[2], std::max(tri.p0.values[2], std::max(tri.p1.values[2], tri.p2.values[2]))));
    }

    std::vector<Vec3> minMaxPoints;
    minMaxPoints.push_back(minPoint);
    minMaxPoints.push_back(maxPoint);

    if (printOutput) {
    std::cout << "Min Point: " << minPoint.values[0] << ", " << minPoint.values[1] << ", " << minPoint.values[2]
              << std::endl;
    std::cout << "Max Point: " << maxPoint.values[0] << ", " << maxPoint.values[1] << ", " << maxPoint.values[2]
              << std::endl;
    }
    return minMaxPoints;
}

/**
 * Checks whether of not a given point is within the mesh by performing computationaly simple
 * min max point test and then furhturmore performs ray intersection tests on the mesh
 * to check if the point is wihtin the mesh for meshes that might resemble a donut for instance
 * @param point
 * @return
 */
bool MeshIntersect::isPointInMesh(Vec3 point) {

    //Checks using basic minmax metrics whether or not the point is wihtin the bounds of the mesh
    //Rules out the need for additional computation
    std::vector<Vec3> minMax = getMinMax(false);
    if (!(point.values[0] >  minMax[0].values[0] && point.values[0] <  minMax[1].values[0])){
        return false;
    }
    if (!(point.values[1] >  minMax[0].values[1] && point.values[1] <  minMax[1].values[1])){
        return false;
    }
    if (!(point.values[2] >  minMax[0].values[2] && point.values[2] <  minMax[1].values[2])){
        return false;
    }
    float meshHeight = minMax[1].values[2] - minMax[0].values[2];
    //More robust way of checking if the point is within the mesh
        auto ray = Ray {
            point,// Ray origin same as param
            Vec3(0., 0., 1.), // Ray direction pointing up, easy way to check ray intersect
            0.,               // Minimum intersection distance
            meshHeight + 100.0f             // Max rtx fximum intersection distance based on meshheight and arbitary large float amount
    };
    int isIntersect = perform_intersect(ray);
    if (isIntersect != 1){
        return true;
    }
    return false;
}


/**
 * Creates a overcast matrix field from which the a series of Ray intersect tests are performed
 * - Must have an height differential and must compute min max and plane from which the ray matrix is generated must be above the file
 * - Currently has sinmple linear system based interpolation technique to determine the distance in which the ray cast occours
 */
std::vector<std::vector<Vec3>> MeshIntersect::generateOvercastRayField() {

    std::vector<Vec3> minMax = getMinMax();//MinMax values 1st index min and 2nd is max

    float zOffsetVal = 10;//Positive offset for z axis

    float sepDif = 0.5f;

    //FIXME, figure out linpot n value which will be used for populating matrix and determine the grid size

    float divNumX = abs(minMax[0].values[0] - minMax[0].values[1]);
    float divNumY = abs(minMax[1].values[0] - minMax[1].values[1]);

    Interpolation pol;
    std::vector<Vec3> interPotPoints;
    interPotPoints.emplace_back(minMax[0].values[0], minMax[0].values[1], minMax[1].values[2] + zOffsetVal);//min  most x point
    interPotPoints.emplace_back(minMax[1].values[0], minMax[0].values[1], minMax[1].values[2] + zOffsetVal);//max most x  point
    // Take these values and initiate y axis lin plot from each interpolated point in matrix
    std::vector<Vec3> xAxisLinPot = pol.createInterpolatedPoints(interPotPoints);

    int nVal = xAxisLinPot.size();//Dimension 1 overcast n matrix

    //Pirinting out test vals
    std::vector<std::vector<Vec3>> grid(nVal, std::vector<Vec3>(nVal));
    for (int i = 0; i < xAxisLinPot.size(); ++i) {
        std::cout << std::to_string(xAxisLinPot[i].values[0])
                  << std::to_string(xAxisLinPot[i].values[1]) <<
                  std::to_string(xAxisLinPot[i].values[2]) << std::endl;
    }
    std::cout << "Debug message " << std::endl;

    std::vector<std::vector<Vec3>> yAxisLinPot;
    for (int i = 0; i < xAxisLinPot.size(); ++i) {
        //Generate each linPot method at minMax of interpolated points and then generate linearly interpolated points along the y axis
        std::vector<Vec3> downcastInitPotPoints;
        downcastInitPotPoints.emplace_back(xAxisLinPot[i].values[0], minMax[0].values[1], minMax[1].values[2] + zOffsetVal);
        downcastInitPotPoints.emplace_back(xAxisLinPot[i].values[0], minMax[1].values[1], minMax[1].values[2] + zOffsetVal);
        std::vector<Vec3> downCastLinPot = pol.createInterpolatedPoints(downcastInitPotPoints);
        //Append to yAxis lin pot to later create grid appending logic
        yAxisLinPot[i] =  downcastInitPotPoints;
    }

    std::cout << "Here debug message" << std::endl;
    // Assign values to the elements of the grid
    //Iteratively add linpot Values into the grid matrix
    for (int i = 0; i < grid.size(); i++) {
        for (int j = 0; j < grid[i].size(); j++) {
            grid[i][j][0] = xAxisLinPot[i].values[0];
            grid[i][j][1] = yAxisLinPot[i][j].values[1];
            grid[i][j][2] = minMax[1].values[2] + zOffsetVal;
        }
    }

    //Intersect grid
    std::vector<std::vector<float>> intersectGrid;

    //Mesh Intersect object and loading the mesh in

    //Iterate over the grid and perform ray casting operations
    for(int i = 0; i < grid.size(); ++i) {
        for(int j = 0; j < grid[i].size(); ++j) {
            //Genrate Ray object at grid location pointing down
                auto downCastray = Ray {
            Vec3(grid[i][j].values[0], grid[i][j].values[1] , minMax[1].values[2] + zOffsetVal), // Ray origin
            Vec3(0., 0., -1.), // Ray direction pointing down for ray mesh overcast method
            0.,               // Minimum intersection distance
            1000            // Maytxfximum intersection distance
            };
                //Performs intersect and appends to the grid value of the vector
            intersectGrid[i][j] = perform_intersect(downCastray);
        }
    }
    // Access and print the elements of the grid
    for (int i = 0; i < grid.size(); i++) {
        for (int j = 0; j < grid[i].size(); j++) {
            std::cout << "(" << grid[i][j][0] << ", " << grid[i][j][1] << ", " << grid[i][j][2] << ") ";
        }
        std::cout << std::endl;
    }
        std::cout << "Size of intersect grid values " << intersectGrid.size() << std::endl;
    for (int i = 0; i < intersectGrid.size(); ++i) {
        for (int j = 0; j < intersectGrid[i].size(); ++j) {
            std::cout << "Intersect Value : " << intersectGrid[i][j] << std::endl;
    }
    }


    return grid;
}

