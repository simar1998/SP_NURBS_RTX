//
// Created by simar on 7/4/2023.
//

#include "OffsetGenerator.h"

/**
 * Use PTM to generate offset points off the points supplied by the param
 * @param mesh
 * @param offsetDistance
 * @return
 */
std::vector<OffsetGenerator::Vec3> OffsetGenerator::ParallelTransportMethod(std::vector<Vec3> points, float offsetDistance) {
    // Initialize an empty vector to store the offset points
    std::vector<Vec3> offsetPoints;

    // Loop through each vertex of the input points
    for (size_t i = 0; i < points.size(); ++i) {
        // Get the current point and the previous and next points (consider wrapping for first and last points)
        Vec3 currentPoint = points[i];
        Vec3 prevPoint = points[(i > 0) ? (i - 1) : (points.size() - 1)];
        Vec3 nextPoint = points[(i + 1) % points.size()];

        // Calculate the vectors for previous-to-current and current-to-next segments
        Vec3 prevToCurrent = currentPoint - prevPoint;
        Vec3 currentToNext = nextPoint - currentPoint;

        // Calculate the normalized outward unit normal vectors for the two segments
        Vec3 outwardNormal1 = normalize(Vec3(-prevToCurrent.values[1], prevToCurrent.values[0], 0.0));
        Vec3 outwardNormal2 = normalize(Vec3(-currentToNext.values[1], currentToNext.values[0], 0.0));

        // Calculate the bisector vector for the offset direction
        Vec3 bisector = normalize(outwardNormal1 + outwardNormal2);

        // Offset the current point along the bisector by the offset distance
        Vec3 offsetPoint = currentPoint + (offsetDistance * bisector);

        // Add the offset point to the vector of offset points
        offsetPoints.push_back(offsetPoint);
    }

    return offsetPoints;
}