//
// Created by simar on 6/16/2023.
//

#ifndef TINYNURBS_GCODE_H
#define TINYNURBS_GCODE_H

#include <string>
#include <vector>
#include <mutex>
#include <thread>

class Gcode {
    std::vector<std::string> gcodeBlock;  // Vector to store G-code segments
    std::mutex gcodeMutex;  // Mutex to synchronize access to the gcodeBlock
    std::thread gcodeThread;  // Thread to process G-code commands

private:
    void startProcessing();  // Start the G-code processing thread
    void stopProcessing();  // Stop the G-code processing thread
public:
    Gcode();  // Constructor
    ~Gcode();  // Destructor


    void appendToGcode(const std::string& gcodeSegment);  // Append a G-code segment
};

#endif //TINYNURBS_GCODE_H