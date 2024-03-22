// @author IoannisMachaliotis

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include "MyCluster.h"
#include <vector>

#ifndef IDCLUSTERING_CLASS_H
#define IDCLUSTERING_CLASS_H

class IDClustering
{
private:
    double eventX_;
    double eventY_;
    double timestampT_;
    unsigned int ID;

    double is_in;

    std::vector<double> aClusterVector;
    Eigen::VectorXd aCluster;
    std::vector<std::vector<double>> aClusterCentersVector;
public:
    // Functions
    [[nodiscard]] std::vector<double> is_cluster_in(std::vector<double> &aClusterVector, std::vector<std::vector<double>> &aClusterCentersVector);
    std::vector<std::vector<double>> clusters_assign_process(const Eigen::VectorXd &cluster, MyCluster &ClusterCenters, std::vector<std::vector<double>> &aClusterCentersVector);
    std::vector<std::vector<double>> assigner(std::vector<std::vector<double>> &aClusterCentersVector, const std::vector<double> &aClusterVector, const double &is_in);
};

#endif
