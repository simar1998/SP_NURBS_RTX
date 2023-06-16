//
// Created by simar on 6/16/2023.
//

#ifndef TINYNURBS_MOVEMENTMODE_H
#define TINYNURBS_MOVEMENTMODE_H

#include <string>

std::string setAbsoluteExtrusionMode() {
    return "M82";
}

std::string setRelativeExtrusionMode() {
    return "M83";
}


std::string moveZF(int zMov, int feed){
    return "G1 F" + std::to_string(feed) + " Z"+std::to_string(zMov);
}

std::string moveXYF(int x, int y, int feed){
    return "G1 F" + std::to_string(feed) + " X"+ std::to_string(x) + " Y"+std::to_string(y);
}

std::string moveXYEF(int x, int y, int feed, int e){
    return "G1 F" + std::to_string(feed) + " X"+ std::to_string(x) + " Y"+std::to_string(y) + " E"+std::to_string(e);
}


std::string moveXYEZF(int x, int y, int feed, int e, int z){
    return "G1 F" + std::to_string(feed) + " X"+ std::to_string(x) + " Y"+std::to_string(y)+ " Z" + std::to_string(z) + " E"+std::to_string(e) ;
}

std::string moveZ(int zMov) {
    return "G1 Z" + std::to_string(zMov);
}

std::string moveXY(int x, int y) {
    return "G1 X" + std::to_string(x) + " Y" + std::to_string(y);
}

std::string moveXYE(int x, int y, int e) {
    return "G1 X" + std::to_string(x) + " Y" + std::to_string(y) + " E" + std::to_string(e);
}

std::string moveXYEZ(int x, int y, int e, int z) {
    return "G1 X" + std::to_string(x) + " Y" + std::to_string(y) + " Z" + std::to_string(z) + " E" + std::to_string(e);
}

#endif //TINYNURBS_MOVEMENTMODE_H

