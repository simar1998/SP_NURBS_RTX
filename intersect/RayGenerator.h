//
// Created by simar on 6/18/2023.
//

#ifndef TINYNURBS_RAYGENERATOR_H
#define TINYNURBS_RAYGENERATOR_H


#include "../mesh_calc/vec.h"
#include "../mesh_calc/ray.h"
#include "../mesh_calc/tri.h"

using Scalar = float;
using Vec3 = bvh::v2::Vec<Scalar, 3>;

enum GeneratorType{
    LINEAR,//Simple plain linear ray origin points along the linearly interpolated vector of at least two points, or more.
    FRONT_FLOOD,//Vectors that flood the front of the ray point and only the front without the bislice of the Bi-Directional GeneratorType
    BI_DIRECTIONAL,//Bi Slice of a globular set of points around point biased towards moving vector
    GLOBULAR //Not going to be implemented anytime soon
};
/*
 * Allows to create rays at point origin and has presets for ray direction, can be used to retrieve ray intersect points
  * With a given mesh. With this you can create a navigation view of a tool path
 */

struct RayIntersectConfig{
    Vec3 originPoint;
    Vec3 frontVec;//Must be initally given to give direction or if null gets generated by a somewhat dubious technique
    GeneratorType type;
};
struct IntersectPoints{

};
struct linearConfig{
    std::vector<Vec3> interpolationPoints;//Last point must be current point
    std::vector<int>  point_w;//The weight of the points asosicated with the linear vector
};
class RayGenerator {

    using Scalar = float;
    using Vec3 = bvh::v2::Vec<Scalar, 3>;
    using Ray = bvh::v2::Ray<Scalar, 3>;
    using Tri = bvh::v2::Tri<Scalar, 3>;

public:
    RayGenerator(const RayIntersectConfig& config, std::vector<Tri>& tris){

    }

    Vec3 linearRay(const RayIntersectConfig &config, std::vector<Tri> &tris, linearConfig& linearConfig);
};


#endif //TINYNURBS_RAYGENERATOR_H
