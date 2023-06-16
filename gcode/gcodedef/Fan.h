//
// Created by simar on 6/16/2023.
//

#ifndef TINYNURBS_FAN_H
#define TINYNURBS_FAN_H


#include <string>

std::string fanOff() {
    return "M107";
}

std::string setFanSpeed(int speed, int index=0){
    if (index != 0){
        return "M106 P"+ std::to_string(index) + " S"+ std::to_string(speed);
    }
    return &"M106 S"[ speed];
}
#endif //TINYNURBS_FAN_H
