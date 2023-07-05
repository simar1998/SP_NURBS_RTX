//
// Created by simar on 7/4/2023.
//

#include "Slice.h"
#include "../intersect/MeshIntersect.h"
#include "../clipper2/clipper.h"
#include "../interpolation/Interpolation.h"

void Slice::beginSlice(std::string filePath, Printer printer) {
    MeshIntersect meshIntersect;
    meshIntersect.loadMesh(filePath);
    std::vector<Vec3> minMax = meshIntersect.getMinMax(false);
    std::vector<float> layersHeights;
    float meshHeight = minMax[1].values[2] - minMax[0].values[2];
    float computedLayersMin = meshHeight / printer.getMinLayerHeight();//gets layer height based on min layer height
    float computedLayersMax = meshHeight / printer.getMaxLayerHeight();//^^ for max layer height

    //First layer print
    float firstLayerIntersect = minMax[0].values[2] + printer.getMinLayerHeight();
    std::vector<Vec3> intPoints = meshIntersect.planeIntersect(firstLayerIntersect, true);

    //convert points from meshIntersect to clipper values
    Clipper2Lib::Paths64 subj;

    double scalingFactor = 1.0e6; // Adjust as needed

    float iterationSteps = 0.1f;

    Interpolation interpolation;
    std::vector<Vec3> interpolatedPoints = interpolation.createInterpolatedPoints(intPoints);

    for (int i = 0; i < interpolatedPoints.size(); ++i) {
        std::cout << "X : " << std::to_string(interpolatedPoints[i][0]) << "Y : " << std::to_string(interpolatedPoints[i][1]) << std::endl;
    }




}
