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
    MeshIntersect::filePath = filePath;
    tris = load_obj<Scalar>(filePath);
}

//Ray intersect builder
//TODO chage output to object for ease of usage
//Add data cleaning and remove primative hits outside the bounds, Tehcnically speaking for test mesh only two hits allowed if primative is not parralel to ray
std::vector<MeshIntersect::intersection> MeshIntersect::perform_intersect(bvh::v2::Ray<Scalar, 3> ray) {

    std::vector<MeshIntersect::intersection>  intersectionList;

    bvh::v2::ThreadPool thread_pool;
    bvh::v2::ParallelExecutor executor(thread_pool);
    MeshIntersect::intersection intersection;
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
    static constexpr bool use_robust_traversal = true;

    auto prim_id = invalid_id;
    Scalar u, v;

    // Traverse the BVH and get the u, v coordinates of the closest intersection.
//    bvh::v2::SmallStack<Bvh::Index, stack_size> stack;
//    bvh.intersect<true, use_robust_traversal>(ray, bvh.get_root().index, stack,
//                                               [&] (size_t begin, size_t end) {
//                                                   for (size_t i = begin; i < end; ++i) {
//                                                       size_t j = should_permute ? i : bvh.prim_ids[i];
//                                                       if (auto hit = precomputed_tris[j].intersect(ray)) {
//                                                           prim_id = i;
//                                                           std::tie(u, v) = *hit;
//                                                           auto w = 1.0f - u - v;
//                                                           Vec3 intersection = u * tris[prim_id].p0 + v * tris[prim_id].p1 + w * tris[prim_id].p2;
//                                                           float newDistance = MeshIntersect::calculateDistance(ray.org,intersection);
//                                                           std::cout << "Test intersect :"  << newDistance << std::endl;
//                                                       }
//                                                   }
//                                                   return prim_id != invalid_id;
//                                               });

                                                bvh::v2::SmallStack<Bvh::Index, stack_size> stack;
                                                bvh.intersect<true, use_robust_traversal>(ray, bvh.get_root().index, stack,
                                              [&] (size_t begin, size_t end) {
                                                  for (size_t i = begin; i < end; ++i) {
                                                      size_t j = should_permute ? i : bvh.prim_ids[i];
                                                      if (auto hit = precomputed_tris[j].intersect(ray)) {
                                                          std::tie(u, v) = *hit;
                                                          auto w = 1.0f - u - v;
                                                          Vec3 intersection = u * tris[j].p0 + v * tris[j].p1 + w * tris[j].p2;
                                                          float newDistance = MeshIntersect::calculateDistance(ray.org,intersection);
                                                          std::cout
                                                                  << "Intersection found\n"
                                                                  << "  primitive: " << j << "\n"
                                                                  << "  distance: " << newDistance << "\n"
                                                                  << "  barycentric coords.: " << u << ", " << v << std::endl;
                                                          std::cout << "Test intersect :"  << newDistance << std::endl;
                                                          MeshIntersect::intersection intersection_obj;
                                                          intersection_obj.originRay = ray;
                                                          intersection_obj.distance = newDistance;
                                                          intersection_obj.primitiveHit = j;
                                                          intersection_obj.u = u;
                                                          intersection_obj.v = v;
                                                          intersectionList.push_back(intersection_obj);
                                                      }
                                                  }
                                                  return false;  // do not terminate BVH traversal
                                              });

    if (prim_id != invalid_id) {
        std::cout
                << "Intersection found\n"
                << "  primitive: " << prim_id << "\n"
                << "  distance: " << ray.tmax << "\n"
                << "  barycentric coords.: " << u << ", " << v << std::endl;
        intersection.originRay = ray;
        intersection.distance = ray.tmax;
        intersection.primitiveHit = prim_id;
        intersection.u = u;
        intersection.v = v;
        //intersection.intersectionPoint = computeVecPoint(intersection);
        //intersectionList.push_back(intersection);
    } else {
       //// std::cout << "No intersection found" << std::endl;
        intersection.primitiveHit = -1;
        prim_id = 0;
        ray.tmax = 0;
    }
    return intersectionList;

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
            //FIXME arbitary value not very good as might cause issues based on mesh bounding box
            meshHeight + 100.0f             // Max rtx fximum intersection distance based on meshheight and arbitary large float amount
    };
    int isIntersect = -1;
    if (isIntersect > 0){
        return true;
    }
    return false;
}


/**
 * Creates a overcast matrix field from which the a series of Ray intersect tests are performed
 * - Must have an height differential and must compute min max and plane from which the ray matrix is generated must be above the mesh
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
           // intersectGrid[i][j] = perform_intersect(downCastray);
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
std::vector<std::vector<Vec3>> MeshIntersect::generateLinOvercastRayField(float samplingDist) {

    std::vector<Vec3> minMax = getMinMax();
    Vec3 gridOrigin = Vec3(minMax[0].values[0] - offsetValXY, minMax[0].values[1] - offsetValXY, minMax[1].values[2] + overcastZvalOffset);

    std::vector<std::vector<Vec3>> grid;

    for (float i = gridOrigin.values[0]; i <= minMax[1].values[0] + offsetValXY; i+= samplingDist) {
        std::vector<Vec3> tempVec;
       // std::cout << "i:" << i <<std::endl;
        for (float j = gridOrigin.values[1]; j <= minMax[1].values[1] + offsetValXY; j+= samplingDist) {
            //std::cout << "i:" << i << "j:" << j << "z:" <<  minMax[1].values[2] + overcastZvalOffset <<std::endl;
            //tempVec.emplace_back(i, j, 3.0000f);

            tempVec.emplace_back(i, j, minMax[0].values[2] + overcastZvalOffset);
            //std::cout << "X : " << std::to_string(temp.values[0]) << " Y :" << std::to_string(temp.values[1]) << " Z :" << std::to_string(temp.values[2]) << std::endl;
        }
        //std::cout << tempVec.size() << std::endl;
        grid.push_back(tempVec);
    }
    std::cout << "Temp vec size: " << grid.size() << std::endl;
    return grid;
}

//Performs Ray intersect with grid plance offsetted on the z axis plane and returns intersect values
std::vector<std::vector<MeshIntersect::intersection>>
MeshIntersect::gridPlaneIntersect(std::vector<std::vector<Vec3>> gridPlane) {
    std::vector<std::vector<MeshIntersect::intersection>> intersectGrid;
    std::cout << "Performing gird intersect " << std::endl;
    for (int i = 0; i <= gridPlane.size(); ++i) {
        std::vector<MeshIntersect::intersection> tempIntersection;
        for (int j = 0; j < gridPlane[i].size(); ++j) {
           // loadMesh(filePath);
            auto ray = Ray {
            Vec3(gridPlane[i][j].values[0], gridPlane[i][j].values[1], gridPlane[i][j].values[2]), // Ray origin
            Vec3(0., 0., -1.0f), // Ray direction pointing down
            0.,               // Minimum intersection distance
            100.0f              // Maytxfximum intersection distance
            };
            std::vector<intersection> list = perform_intersect(ray);
            for (int l = 0; l < list.size(); ++l) {
                tempIntersection.push_back(list[l]);
            }
        }
        intersectGrid.push_back(tempIntersection);
        //std::cout  << "Interation " << std::endl;
    }
    return intersectGrid;
}
std::vector<MeshIntersect::intersection>
MeshIntersect::gridPlaneIntersectSimple(std::vector<std::vector<Vec3>> gridPlane) {
std::vector<MeshIntersect::intersection> intersectList;
    std::cout << "Performing gird intersect " << std::endl;
    for (int i = 0; i <= gridPlane.size(); ++i) {
        for (int j = 0; j < gridPlane[i].size(); ++j) {
            //loadMesh(filePath);
            auto ray = Ray {
                    Vec3(gridPlane[i][j].values[0], gridPlane[i][j].values[1], gridPlane[i][j].values[2]), // Ray origin
                    Vec3(0., 0., -1.0f), // Ray direction pointing down
                    0.,               // Minimum intersection distance
                    100.0f              // Maytxfximum intersection distance
            };

            intersectList.push_back(rayIntersect(ray));

        }
        //std::cout  << "Interation " << std::endl;
    }
    return intersectList;
}

//Compute Vec3 point from barycentric cords and prim vertex values
Vec3 MeshIntersect::computeVecPoint(MeshIntersect::intersection intersection) {
    float w = 1.0f - intersection.u - intersection.v;
    Vec3 p = (tris[intersection.primitiveHit].p0) * w + (tris[intersection.primitiveHit].p1) * intersection.u + (tris[intersection.primitiveHit].p2) * intersection.v;
    return p;
}

//Fixme Creates intersection values outside of the min max value for some reason
/**
 * Slower but pherhaps better and more consistent ray intersect method, takes one ray param and returns a intersection struct with data pretaining to the intersection
 * @param ray
 * @return
 */
MeshIntersect::intersection MeshIntersect::rayIntersect(MeshIntersect::Ray& ray){
    MeshIntersect::intersection intersection;
    intersection.primitiveHit = -1;
    Vec3 rayOrigin = ray.org;
    Vec3 rayDirection = ray.dir;

    for (int i = 0; i < tris.size(); i++) {//Iterate over triangle list
        Vec3 v1 = tris[i].p0;
        Vec3 v2 = tris[i].p1;
        Vec3 v3 = tris[i].p2;

        //Checking if the ray is parallel to the triangle's plane
        Vec3 edge1 = {v2.values[0] - v1.values[0], v2.values[1] - v1.values[1], v2.values[2] - v1.values[2]};
        Vec3 edge2 = {v3.values[0] - v1.values[0], v3.values[1] - v1.values[1], v3.values[2] - v1.values[2]};

        Vec3 normal = {
                (edge1.values[0] * edge2.values[2]) - (edge1.values[2] * edge2.values[1]),
                (edge1.values[2] * edge2.values[0]) - (edge1.values[0] * edge2.values[2]),
                (edge1.values[0] * edge2.values[1]) - (edge1.values[1] * edge2.values[0])
        };
        float dotProduct = normal.values[0] * ray.org.values[0] +  normal.values[1] * ray.org.values[1] +  normal.values[2] * ray.org.values[2];
        const float eps = 0.0001f;//Parallel tolerence

        /**
         *
         */

        if (std::fabs(dotProduct) < eps) {
            // The ray is parallel to the triangle's plane
            continue;
        }

        // Compute the intersection point P
        float t = ((v1.values[0] - ray.org.values[0]) * normal.values[0] + (v1.values[1] - ray.org.values[1]) * normal.values[1] + (v1.values[2] -
                ray.org.values[2]) * normal.values[2]) / dotProduct;
        if (t < 0) {
            // The intersection point is behind the ray's origin
            continue;
        }
        Vec3 intersectionPoint = rayOrigin + rayDirection * t;

        // Check if the intersection point P is inside the triangle
        Vec3 c0 = {
                (edge1.values[1] * (intersectionPoint.values[2] - v1.values[2])) - (edge1.values[2] * (intersectionPoint.values[1] - v1.values[1])),
                (edge1.values[2] * (intersectionPoint.values[0] - v1.values[0])) - (edge1.values[0] * (intersectionPoint.values[2] - v1.values[2])),
                (edge1.values[0] * (intersectionPoint.values[1] - v1.values[1])) - (edge1.values[1] * (intersectionPoint.values[0] - v1.values[0]))
        };
        Vec3 c1 = {
                (edge2.values[1] * (intersectionPoint.values[2] - v2.values[2])) - (edge2.values[2] * (intersectionPoint.values[1] - v2.values[1])),
                (edge2.values[2] * (intersectionPoint.values[0] - v2.values[0])) - (edge2.values[0] * (intersectionPoint.values[2] - v2.values[2])),
                (edge2.values[0] * (intersectionPoint.values[1] - v2.values[1])) - (edge2.values[1] * (intersectionPoint.values[0] - v2.values[0]))
        };
        Vec3 c2 = {
                (edge1.values[1] * (intersectionPoint.values[2] - v3.values[2])) - (edge1.values[2] * (intersectionPoint.values[1] - v3.values[1])),
                (edge1.values[2] * (intersectionPoint.values[0] - v3.values[0])) - (edge1.values[0] * (intersectionPoint.values[2] - v3.values[2])),
                (edge1.values[0] * (intersectionPoint.values[1] - v3.values[1])) - (edge1.values[1] * (intersectionPoint.values[0] - v3.values[0]))
        };
        float dot0 = c0.values[0] * normal.values[0] + c0.values[1] * normal.values[1] + c0.values[2] * normal.values[2];
        float dot1 = c1.values[0] * normal.values[0] + c1.values[1] * normal.values[1] + c1.values[2] * normal.values[2];
        float dot2 = c2.values[0] * normal.values[0] + c2.values[1] * normal.values[1] + c2.values[2] * normal.values[2];
        if (dot0 >= 0 && dot1 >= 0 && dot2 >= 0) {
            intersection.primitiveHit = i;
            intersection.distance = t;
            intersection.intersectionPoint = intersectionPoint;
            std::cout << "Ray intersection detected at prim " << i << " at distance of " << t << " At point " << intersectionPoint.values[0] << "," << intersectionPoint.values[1] << "," << intersectionPoint.values[2] << "||" << std::endl;
            std::cout << "Ray origin " << ray.org.values[0] << "," <<  ray.org.values[1] << "," << ray.org.values[2] << "|" << std::endl;
            //return intersection;
        }
        // No intersection found
    }
    return intersection;
}




