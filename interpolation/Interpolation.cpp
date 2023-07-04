//
// Created by simar on 6/23/2023.
//

#include "Interpolation.h"

//Interpolate points between each given point
//FIXME possible issue with class Vec3 object refrence
std::vector<Interpolation::Vec3> Interpolation::createInterpolatedPoints(std::vector<Vec3> points) {
    std::vector<Vec3> interpolatedPoints;

    // Iterate through each pair of consecutive points
    for (size_t i = 0; i < points.size() - 1; i++) {
        Vec3 currentPoint = points[i];
        Vec3 nextPoint = points[i + 1];

        // Calculate the distance between the current and next point
        float distance = calculateDistance(currentPoint, nextPoint);

        // Calculate the number of interpolated points based on the distance
        int numInterpolatedPoints = static_cast<int>(distance); // Adjust as needed

        // Calculate the step size for interpolation
        float stepSizeX = (nextPoint.values[0] - currentPoint.values[0]) / numInterpolatedPoints;
        float stepSizeY = (nextPoint.values[1] - currentPoint.values[1]) / numInterpolatedPoints;
        float stepSizeZ = (nextPoint.values[2] - currentPoint.values[2]) / numInterpolatedPoints;

        // Perform linear interpolation between the current and next point
        for (int j = 0; j < numInterpolatedPoints; j++) {
            Vec3 interpolatedPoint;
            interpolatedPoint.values[0] = currentPoint.values[0] + (stepSizeX * j);
            interpolatedPoint.values[1] = currentPoint.values[1] + (stepSizeY * j);
            interpolatedPoint.values[2] = currentPoint.values[2] + (stepSizeZ * j);

            interpolatedPoints.push_back(interpolatedPoint);
        }
    }

    return interpolatedPoints;
}
