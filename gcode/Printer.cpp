//
// Created by simar on 6/29/2023.
//

#include "Printer.h"

// Constructor
Printer::Printer() {
    // Initialize member variables with default values
    xSize = 0.0;
    ySize = 0.0;
    zSize = 0.0;
    maxLayerHeight = 0.0;
    minLayerHeight = 0.0;
    maxLayerWidth = 0.0;
    minLayerWidth = 0.0;
    nozzleSize = 0.0;
    filamentTemp = 0.0;
    bedTemp = 0.0;
}

// Getter methods
double Printer::getXSize() const {
    return xSize;
}

double Printer::getYSize() const {
    return ySize;
}

double Printer::getZSize() const {
    return zSize;
}

double Printer::getMaxLayerHeight() const {
    return maxLayerHeight;
}

double Printer::getMinLayerHeight() const {
    return minLayerHeight;
}

double Printer::getMaxLayerWidth() const {
    return maxLayerWidth;
}

double Printer::getMinLayerWidth() const {
    return minLayerWidth;
}

double Printer::getNozzleSize() const {
    return nozzleSize;
}

double Printer::getFilamentTemp() const {
    return filamentTemp;
}

double Printer::getBedTemp() const {
    return bedTemp;
}

// Setter methods
void Printer::setXSize(double xSize) {
    this->xSize = xSize;
}

void Printer::setYSize(double ySize) {
    this->ySize = ySize;
}

void Printer::setZSize(double zSize) {
    this->zSize = zSize;
}

void Printer::setMaxLayerHeight(double maxLayerHeight) {
    this->maxLayerHeight = maxLayerHeight;
}

void Printer::setMinLayerHeight(double minLayerHeight) {
    this->minLayerHeight = minLayerHeight;
}

void Printer::setMaxLayerWidth(double maxLayerWidth) {
    this->maxLayerWidth = maxLayerWidth;
}

void Printer::setMinLayerWidth(double minLayerWidth) {
    this->minLayerWidth = minLayerWidth;
}

void Printer::setNozzleSize(double nozzleSize) {
    this->nozzleSize = nozzleSize;
}

void Printer::setFilamentTemp(double filamentTemp) {
    this->filamentTemp = filamentTemp;
}

void Printer::setBedTemp(double bedTemp) {
    this->bedTemp = bedTemp;
}
