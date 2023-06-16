//
// Created by simar on 6/16/2023.
//

#ifndef TINYNURBS_INPUTPROCESSING_H
#define TINYNURBS_INPUTPROCESSING_H

#include "../mesh_calc/vec.h"
#include "../mesh_calc/bvh.h"
#include "../mesh_calc/vec.h"
#include "../mesh_calc/ray.h"
#include "../mesh_calc/node.h"
#include "../mesh_calc/default_builder.h"
#include "../mesh_calc/thread_pool.h"
#include "../mesh_calc/executor.h"
#include "../mesh_calc/stack.h"
#include "../mesh_calc/tri.h"
#include "../mesh_calc/load_obj.h"
#include <iostream>
#include <list>

class InputProcessing {
    using Scalar = float;
    using Vec3 = bvh::v2::Vec<Scalar, 3>;;

public:
    //Must be consecutive sorted points as this will generate gcode in a consecitive list object
    int addPoints(std::list<Vec3> transversalPoints);
};


#endif //TINYNURBS_INPUTPROCESSING_H
