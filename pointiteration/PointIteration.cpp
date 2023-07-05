//
// Created by simar on 7/1/2023.
//

#include "PointIteration.h"
#include "../intersect/MeshIntersect.h"
#include <random>

//Load mesh from mesh intersect and randomly check plane intersect values and perform ray intersect values
//Infer sampling points from the plane intersect in order to perform ray intersect tests on the given mesh
std::vector<Vec3> PointIteration::initPointIteration(std::string filePath) {
    MeshIntersect intersect;
    intersect.loadMesh(filePath);
    std::vector<Vec3> minMax = intersect.getMinMax(false);

    //Generate random sampling points to begin point iteration
    float offsetVal = 0.1f;
    float lowerIntersectPoint = minMax[0].values[2] + offsetVal;
    std::vector<Vec3> intersectPoints = intersect.planeIntersect(lowerIntersectPoint, false);

    Vec3 initPoint = intersectPoints[0];


    bool isInitInside = false;

    float range = 5;//This arbitary value where the code initiated over the x and y dimensions and gets the list of candidates vlaues
    int generationValues = 20;
    std::vector<Vec3> preSortedValues;

    //First step for x axis and then second for the y
    auto dimX = initPoint.values[0];
    auto dimY = initPoint.values[1];
    std::random_device rd;
    std::default_random_engine generator(rd());
    std::uniform_real_distribution<float> distributionX(dimX - (range/2) , dimX + (range/2));
    std::uniform_real_distribution<float> distributionY(dimY - (range/2) , dimY + (range/2));
    for (int j = 0; j < generationValues; ++j) {
        preSortedValues[j] = Vec3(distributionX(generator), distributionY(generator) , initPoint.values[2]);
    }

    std::vector<Vec3> verifiedValues;//Stores values after ray intersect test is completed on the presorted values

    //Iterate over presorted generated values and perform ray mesh intersect tests to validate if the point is within the mesh
    for (auto preSortedValue : preSortedValues) {
        if (intersect.isPointInMesh(preSortedValue)){
            verifiedValues.push_back(preSortedValue);
        }
    }

    //Begin dimensional point iteration with sphere bias towards






    //ensure

    return std::vector<Vec3>();
}
