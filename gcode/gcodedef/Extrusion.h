//
// Created by simar on 6/16/2023.
//

#ifndef TINYNURBS_EXTRUSION_H
#define TINYNURBS_EXTRUSION_H
#include <string>

std::string retract(int feedrate, int extAmount){
    return "G1 F" + std::to_string(feedrate) + " E"+std::to_string(extAmount);
}


#endif //TINYNURBS_EXTRUSION_H
