// @author IoannisMachaliotis

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <vector>
#include <math.h>
#include <sstream>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>

#ifndef IMPROVE_CLASS_H
#define IMPROVE_CLASS_H

class Improve
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
    std::vector<std::vector<double>> remover(std::vector<std::vector<double>> &cluster_list);
    std::vector<std::vector<double>> kalmanfilter(std::vector<std::vector<double>> &cluster_list, std::vector<std::vector<double>> &kalman_centers);
    // [[nodiscard]] VectorXd KF_algorithm(const VectorXd &y); // Todo: to add this 
};

#endif