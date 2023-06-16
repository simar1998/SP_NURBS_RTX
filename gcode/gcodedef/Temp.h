//
// Created by simar on 6/16/2023.
//

#ifndef TINYNURBS_TEMP_H
#define TINYNURBS_TEMP_H

#include <string>

std::string setBedTemperature(int temp) {
    return "M140 S" + std::to_string(temp);
}

std::string setExtruderTemperature(int temp) {
    return "M104 S" + std::to_string(temp);
}

std::string waitForHotend(int temp){
    return "M109 S" + std::to_string(temp);
}

std::string waitForBed(int temp){
    return "M190 S" + std::to_string(temp);
}


#endif //TINYNURBS_TEMP_H
