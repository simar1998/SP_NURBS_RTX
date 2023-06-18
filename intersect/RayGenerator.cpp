//
// Created by simar on 6/18/2023.
//

#include <iostream>
#include "RayGenerator.h"


//Build logic for generating and storing ray colision hits for a linear
//TODO build return logic for this, currently void for testing purposes
//TODO currently only two vertex point interpolation for the vector building is supported
//TODO Figure out if collision is occouring in this part of the code or elsewhere
//FIXME change this code to return ray collision struct
Vec3 RayGenerator::linearRay(const RayIntersectConfig &config, std::vector<Tri> &tris, linearConfig &linearConfig) {

    if (linearConfig.interpolationPoints.size() == 2) {
        RayGenerator::Vec3 aV = linearConfig.interpolationPoints.back();
        RayGenerator::Vec3 bV = linearConfig.interpolationPoints.front();

        RayGenerator::Vec3 res;

        res.values[0] = bV.values[0] - aV.values[0];
        res.values[1] = bV.values[1] - aV.values[1];
        res.values[2] = bV.values[2] - aV.values[2];

        return res;

    } else {
        std::cout << "Linear ray interpolation technique not available";
    }
}


