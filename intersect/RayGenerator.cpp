//
// Created by simar on 6/18/2023.
//

#include <iostream>
#include "RayGenerator.h"
#include <cmath>

//Build logic for generating and storing ray colision hits for a linear
//TODO build return logic for this, currently void for testing purposes
//TODO currently only two vertex point interpolation for the vector building is supported
//TODO Figure out if collision is occouring in this part of the code or elsewhere
//FIXME change this code to return ray collision struct
//Source file generates vectors used for ray mesh intersection tests


//Creates
Vec3 RayGenerator::linearRay(linearConfig &linearConfig) {

    if (linearConfig.interpolationPoints.size() == 2) {
        RayGenerator::Vec3 aV = linearConfig.interpolationPoints.back();
        RayGenerator::Vec3 bV = linearConfig.interpolationPoints.front();

        RayGenerator::Vec3 res;

        res.values[0] = bV.values[0] - aV.values[0];
        res.values[1] = bV.values[1] - aV.values[1];
        res.values[2] = bV.values[2] - aV.values[2];

        return res;

    } else {
        std::cout << "Linear ray interpolation technique not available" << std::endl;
    }
}

//Front flood 3d vectors to allow for flooding ray effect IN 3D SPACE
std::vector<Vec3> RayGenerator::frontFlood(frontFloodConfig &frontFloodConfig) {
    /**
     * Create normalized vector so vector magnitude is same
     */
    float magnitude =  (std::sqrt(frontFloodConfig.frontVec[0]*frontFloodConfig.frontVec[0] +
            frontFloodConfig.frontVec[1]*frontFloodConfig.frontVec[1] +
            frontFloodConfig.frontVec[2]*frontFloodConfig.frontVec[2]));
    Vec3 nomrmalizedVector;
    nomrmalizedVector[0] = frontFloodConfig.frontVec[0] / magnitude;
    nomrmalizedVector[1] = frontFloodConfig.frontVec[1] / magnitude;
    nomrmalizedVector[2] = frontFloodConfig.frontVec[2] / magnitude;

    //Angles to rad
    float angle_radians = (frontFloodConfig.vecAngles) * (M_PI / 180.0);

    //Rotate vector to create headlight effect
    std::vector<Vec3> rotatedVecs;
    for (int i = 0; i < frontFloodConfig.floodPoints; ++i) {
        float rot_angle = angle_radians * i; //multiplier for angles per loop iteration
        float rot_x = nomrmalizedVector[0] * std::cos(rot_angle) - nomrmalizedVector[1] * std::sin(rot_angle);
        float rot_y = nomrmalizedVector[0] * std::sin(rot_angle) + nomrmalizedVector[1] * std::cos(rot_angle);
        rotatedVecs[i] = Vec3(rot_x, rot_y, nomrmalizedVector[2]);
    }
    return rotatedVecs;
}

//Creates biased bidirectional flood + perpendicular rays
std::vector<Vec3> RayGenerator::biDirectional(biDirectionalConfig &biDirectionalConfig) {
    std::vector<Vec3> floodVecs = frontFlood(biDirectionalConfig.frontFloodConfig);//Get front flood rays
    frontFloodConfig perpConfig = reinterpret_cast<const frontFloodConfig &>(biDirectionalConfig);//gets bidirectional flood config
    perpConfig.vecAngles = 90.0f;//changes vec angles to 90 degrees to allow for perp rays
    std::vector<Vec3> perpVecs = frontFlood(perpConfig);
    for (int i = 0; i < perpVecs.size(); ++i) {
        floodVecs.push_back(perpVecs[i]);//appends perp vecs to ray
    }
    return floodVecs;
}




