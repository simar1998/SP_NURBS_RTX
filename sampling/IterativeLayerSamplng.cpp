//
// Created by simar on 7/20/2023.
//

#include "IterativeLayerSamplng.h"
#include "../intersect/MeshIntersect.h"

/**
 * Returns the zLayer sampling points for algo ops
 * @param layerHeight
 * @param mesh
 * @return
 */
std::vector<MeshIntersect::zIntersectInfo>
IterativeLayerSamplng::performIntersections(float layerHeight, MeshIntersect mesh) {
    std::vector<Vec3> minMax = mesh.getMinMax();//Gets the min max of the mesh
    //Performs the calculation on the z axis layer spread
    std::vector<float> intersectHeights;//Stores the values of the layer heights where the intersects are performed
    for (float i = layerHeight; i < minMax[1].values[2] / layerHeight; ++i) {
        intersectHeights.push_back(layerHeight + (layerHeight*i));
    }
    std::vector<MeshIntersect::zIntersectInfo> totalIntersects;
    //Performs iteration at the layer heights
    for (int i = 0; i < intersectHeights.size(); ++i) {
        std::vector<MeshIntersect::zIntersectInfo> zIntersect = mesh.zPlaneIntersect(intersectHeights[i],i);
        if (!zIntersect.empty()) {
            totalIntersects.insert(totalIntersects.end(), zIntersect.begin(), zIntersect.end());
        }
    }
    //With the following information I can perform Serveeral mathematical operations on the values
    return totalIntersects;
}
