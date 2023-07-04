//
// Created by simar on 7/4/2023.
//

#ifndef TINYNURBS_SLICE_H
#define TINYNURBS_SLICE_H


#include <string>
#include "../mesh_calc/vec.h"
#include "../gcode/Printer.h"

class Slice {
    using Scalar = float;
    using Vec3 = bvh::v2::Vec<Scalar, 3>;

public:
    void beginSlice(std::string filePath, Printer printer);
};


#endif //TINYNURBS_SLICE_H
