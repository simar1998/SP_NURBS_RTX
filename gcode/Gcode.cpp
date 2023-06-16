//
// Created by simar on 6/16/2023.
//

#include <iostream>
#include "Gcode.h"

Gcode::Gcode() {
   startProcessing();
}

Gcode::~Gcode() {
    // Destructor
    stopProcessing();
}

void Gcode::startProcessing() {
    if (gcodeThread.joinable())
        return;  // The thread is already running

    gcodeThread = std::thread([this]() {
        while (true) {
            std::string gcodeSegment;

            // Check if there is any G-code segment available
            {
                std::lock_guard<std::mutex> lock(gcodeMutex);
                if (gcodeBlock.empty())
                    continue;

                gcodeSegment = gcodeBlock.front();
                gcodeBlock.erase(gcodeBlock.begin());
            }

            // Process the G-code segment (e.g., send it to the machine)
            // You can replace this part with your actual processing logic
            std::cout << "Processing G-code segment: " << gcodeSegment << std::endl;
        }
    });
}

void Gcode::stopProcessing() {
    if (gcodeThread.joinable()) {
        gcodeThread.join();
    }
}

void Gcode::appendToGcode(const std::string& gcodeSegment) {
    std::lock_guard<std::mutex> lock(gcodeMutex);
    gcodeBlock.push_back(gcodeSegment);
}
