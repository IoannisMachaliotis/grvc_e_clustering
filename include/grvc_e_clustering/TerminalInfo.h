// @author IoannisMachaliotis

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#ifndef TERMINALINFO_CLASS_H
#define TERMINALINFO_CLASS_H

class TerminalInfo
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
    std::vector<std::vector<double>> show_clusters();
    std::vector<std::vector<double>> show_centers();

    std::vector<double> show_vector();
};

#endif
