//
// Created by simar on 7/19/2023.
//

#ifndef TINYNURBS_CENTROID_H
#define TINYNURBS_CENTROID_H


#include "../mesh_calc/vec.h"

class Centroid {
using Scalar = float;
using Vec3 = bvh::v2::Vec<Scalar, 3>; 

public :
    //Computes centroid from the list of Vec3 points
    Vec3 computeCentroid(std::vector<Vec3> list){
    float sum_x, sum_y, sum_z;
        for (int i = 0; i < list.size(); ++i) {
            sum_x += list[i].values[0];
            sum_y += list[i].values[1];
            sum_z += list[i].values[2];
        }
        sum_x = sum_x / list.size();
        sum_y = sum_y / list.size();
        sum_z = sum_z / list.size();
        return Vec3(sum_x,sum_y,sum_z);
}
};


#endif //TINYNURBS_CENTROID_H
