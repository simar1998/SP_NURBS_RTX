//
// Created by simar on 6/29/2023.
//

#ifndef TINYNURBS_PRINTER_H
#define TINYNURBS_PRINTER_H



class Printer {
public:
    // Constructor
    Printer();

    // Getter methods
    double getXSize() const;
    double getYSize() const;
    double getZSize() const;
    double getMaxLayerHeight() const;
    double getMinLayerHeight() const;
    double getMaxLayerWidth() const;
    double getMinLayerWidth() const;
    double getNozzleSize() const;
    double getFilamentTemp() const;
    double getBedTemp() const;

    // Setter methods
    void setXSize(double xSize);
    void setYSize(double ySize);
    void setZSize(double zSize);
    void setMaxLayerHeight(double maxLayerHeight);
    void setMinLayerHeight(double minLayerHeight);
    void setMaxLayerWidth(double maxLayerWidth);
    void setMinLayerWidth(double minLayerWidth);
    void setNozzleSize(double nozzleSize);
    void setFilamentTemp(double filamentTemp);
    void setBedTemp(double bedTemp);

private:
    // Printer dimensions
    double xSize;
    double ySize;
    double zSize;

    // Layer specifications
    double maxLayerHeight;
    double minLayerHeight;
    double maxLayerWidth;
    double minLayerWidth;

    // Nozzle and temperature settings
    double nozzleSize;
    double filamentTemp;
    double bedTemp;
};

#endif //TINYNURBS_PRINTER_H
