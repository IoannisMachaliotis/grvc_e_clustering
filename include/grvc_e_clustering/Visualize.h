// @author IoannisMachaliotis

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#ifndef VISUALIZE_CLASS_H
#define VISUALIZE_CLASS_H

class Visualize
{
private:
    double coordinateX_;
    double coordinateY_;
    double timestampT_;
    unsigned int ID;
    std::vector<double> aClusterVector;
    std::vector<std::vector<double>> aClusterCentersVector;
public:
    // Functions
    void visualizer();
};

#endif