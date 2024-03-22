// @author IoannisMachaliotis

#include "grvc_e_clustering/IDClustering.h"
#include <ros/ros.h>

const double my_radius = 15;

[[nodiscard]] std::vector<double> is_cluster_in(std::vector<double> &aClusterVector, std::vector<std::vector<double>> &aClusterCentersVector)
{
    std::vector<double> Output;
    double IS_IN = 0;
    double x_v;
    double y_v;
    double timeOfNew;
    double InitializedMaxDist = 500;
    double IDx;
    double speed = 0;
    double dt,t_previous;

    int featureIterator = 0;
    // Get info from centersConverted
    for (double &aFeature : aClusterVector)
    {
        if (featureIterator == 0)
        {
            x_v = aFeature;
        }
        if (featureIterator == 1)
        {
            y_v = aFeature;
        }
        if (featureIterator == 2)
        {
            timeOfNew = aFeature;
        }
        featureIterator++;
    }

    // Get info from cluster_centers
    for (const std::vector<double> &aVector : aClusterCentersVector)
    {
        double x;
        double y;
        double Id;
        double t_prev;

        int featureIteratorII = 0;
        for (const double &aCenter : aVector)
        {
            if (featureIteratorII == 0)
            {
                Id = aCenter;
            }
            if (featureIteratorII == 1)
            {
                x = aCenter;
            }
            if (featureIteratorII == 2)
            {
                y = aCenter;
            }
            if (featureIteratorII == 3)
            {
                t_prev = aCenter;
            }
            featureIteratorII++;
        }
        // Find the distance of each cluster from the new cluster
        const double distanceFromNew = sqrt(pow(x - x_v, 2) + pow(y - y_v, 2));

        if (distanceFromNew < InitializedMaxDist) // find the one who is closer
        {
            InitializedMaxDist = distanceFromNew;
            IDx = Id;
            t_previous = t_prev;
        }
    }
    if (InitializedMaxDist <= my_radius)    // assume it's the same cluster
    {
        IS_IN = IDx;   
        dt = timeOfNew - t_previous;        // Calculate time difference
        speed = InitializedMaxDist/dt;               // Calculate speed
    }
    else
    {                                       // assume it's a new one
        IS_IN = timeOfNew;
    }
    Output.push_back(IS_IN);        
    Output.push_back(speed);                // Add speed to list of vectors

    return Output;
}


std::vector<std::vector<double>> IDClustering::assigner(std::vector<std::vector<double>> &aClusterCentersVector, const std::vector<double> &aCenterConverted, const double &is_in)
{
    const double Is_in = is_in;
    std::vector<double> aTempVector;                     // vector -->  {}

    // CHECK IF THE CLUSTER EXISTS IN THE cluster_centers LIST
    if (Is_in == 0 && aClusterCentersVector.empty())              // For the first iteration/cluster 
    {
        aTempVector.push_back(ID);                       // vector -->  {ID}
    }
    else if (Is_in > 100000)                             // if it's a timestamp
    {
        ID++;                                            // ----CREATE CLUSTER ID----
        aTempVector.push_back(ID);                       // vector -->  {ID}
    }
    else if (Is_in < 100000 && !aClusterCentersVector.empty()) // if it's an ID from condition: distance < limit
    {
        unsigned int indexOfClusterFound;
        // ---replace AFTER ERASING---
        unsigned int ClusterID = 0;
        for (const std::vector<double> &aClusterVector : aClusterCentersVector)
        {
            if (aClusterVector[0] == Is_in)
            {
                indexOfClusterFound = ClusterID;
            }
            ClusterID++;
        }
        aClusterCentersVector.erase(aClusterCentersVector.begin() + indexOfClusterFound);

        // ----ASSIGN SAME CLUSTER ID----
        aTempVector.push_back(Is_in);                    // vector -->  {ID}
    }
    for (const double &aCenter : aCenterConverted)
    {
        aTempVector.push_back(aCenter);                  // vector -->  {ID, x_new, y_new, t_stamp}
    }

    // Push back new cluster center, generated with ID, in a VECTOR, alongside with timestamp
    aClusterCentersVector.push_back(aTempVector);

    return aClusterCentersVector;
}

std::vector<std::vector<double>> IDClustering::clusters_assign_process(const Eigen::VectorXd &cluster, MyCluster &ClusterCenters, std::vector<std::vector<double>> &aClusterCentersVector)
{
    double Is_in;

    // Convert cen vector from eigen vector to std::vector
    std::vector<double> cluster_converted(&cluster[0], cluster.data() + cluster.cols() * cluster.rows());

    // STORE THE TIMESTAMP ALONG SIDE WITH THE EVENT/CLUSTER
    ros::Time t_stamp = ros::Time::now();
    cluster_converted.push_back(t_stamp.toSec()); // Add the timestamp to new cluster

    // STORE THE # OF EVENTS WHICH GENERATED THIS CLUSTER
    cluster_converted.push_back(ClusterCenters.getN());

    // IS THE NEW CLUSTER ALREADY LISTED??
    const std::vector<double> updatedVector = is_cluster_in(cluster_converted, aClusterCentersVector);

    unsigned int iter = 0;
    for ( const double &aFeature : updatedVector)
    {
        if (iter == 0)
        {
            Is_in = aFeature; // Get is_in value
        }
        if (iter == 1)
        {
          cluster_converted.push_back(aFeature); // add speed to data
        }
        iter++;
    }

    // ASSIGN THE NEW VECTOR TO cluster_centers
    assigner(aClusterCentersVector, cluster_converted, Is_in);

    return aClusterCentersVector;
}