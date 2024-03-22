#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <dvs_msgs/EventArray.h>
#include "grvc_e_clustering/AEClustering.h"

// My libs
#include "grvc_e_clustering/IDClustering.h"
#include <memory>

// AEClustering *eclustering(new AEClustering);
std::unique_ptr<AEClustering> eclustering(new AEClustering);

IDClustering IDClustersObj;
// IDClustersObj.aClusterCentersVector = {};

image_transport::Publisher pubIm;
sensor_msgs::ImagePtr im_msg;

std::vector<std::vector<double>> removal_KF_visualize(std::vector<std::vector<double>> &cluster_list)
{
    Improve* opt;
    Tracking* tracking;
    TerminalInfo* infoT;
    // REMOVAL OF CLUSTERS THAT HAVE STOPPED BEING TRACKED
    opt->remover(cluster_list);

    if (terminal)
    {
        std::cout << "\n\n----- 🟣 ORIGINAL CLUSTERS ----\n";
    }

    // KALMAN FILTERING --->> improve clusters by precision
    if (!cluster_list.empty())
    {
        // SORT before using kalman filter
        sort(cluster_list.begin(), cluster_list.end());
        opt->kalmanfilter(cluster_list, kalman_centers);

        // Terminal View of kalman_centers
        if (terminal)
        {
            std::cout << "\n----- 🟠 KALMAN CENTERS ------\n";
            infoT->show_clusters(kalman_centers);
        }
    }
    else
    {
        kalman_centers.erase(kalman_centers.begin(), kalman_centers.end());
    }

    if (!kalman_centers.empty())
    {
        tracking->object_tracker(kalman_centers);
    }

    // TERMINAL VIEW of list created
    if (terminal)
    {
        std::cout << "\n----- 🟢 CLUSTERS LIST CREATED --\n";
        infoT->show_clusters(cluster_list);
    }

    // ---- VISUALIZATION OF CLUSTERS CREATED ----
    visualizer(kalman_centers, cluster_list, object_coordinates); // orange and green

    // USE THE NEW KALMAN CENTERS for next iteration (FEEDBACK)
    cluster_list.erase(cluster_list.begin(), cluster_list.end());
    for (const std::vector<double> aVector : kalman_centers)
    {
        cluster_list.push_back(aVector);
    }

    delete opt;
    delete tracking;
    delete infoT;

    return kalman_centers;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        int minN = eclustering->getMinN(); 
        cv::Mat im = cv_bridge::toCvShare(msg, "rgb8")->image; 
        for (auto &ClusterCenters : eclustering->clusters)
        {
            if (ClusterCenters.getN() >= minN)
            {
                Eigen::VectorXd cen(ClusterCenters.getClusterCentroid());
                cv::circle(im,cv::Point(cen[0],cen[1]), 1, cv::Scalar(0,255,0,255), -1);
            }
            // Assign cluster
            IDClustersObj.clusters_assign_process(cen, ClusterCenters, IDClustersObj.aClusterCentersVector);
        }
        // Remove clusters who stopped being tracked, apply Kalman Filter and visualize results
        removal_KF_visualize(cluster_centers);

        im_msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", im).toImageMsg();
        pubIm.publish(im_msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }

    delete IDClustersObj;
}

void eventCallback(const dvs_msgs::EventArray::ConstPtr &msg)
{
    std::deque<double> ev(4, 0.0);
    for (const auto e : msg->events)
    {
        ev[0] = e.ts.toSec();       
        ev[1] = e.x;
        ev[2] = e.y;
        ev[3] = e.polarity;

        // Exclude broken pixel(231,202) due to hardware flaw of davis used at the lab
        if ((e.x > 231 || e.x < 229) & (e.y > 203 || e.y < 200))
        { 
            eclustering->update(ev);
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "asyncrhonous_event_based_clustering_node");
    ros::NodeHandle nh_public;
    ros::NodeHandle nh("~");

    int szBuffer;
    double radius;
    double alpha;
    int minN;
    int kappa;

    nh.getParam("szBuffer", szBuffer);
    nh.getParam("radius", radius); 
    nh.getParam("kappa", kappa); 
    nh.getParam("alpha", alpha);
    nh.getParam("minN", minN);
    
    eclustering->init(szBuffer, radius, kappa, alpha, minN);

    ros::Subscriber subEv = nh.subscribe("/dvs/events", 1, eventCallback);

    image_transport::ImageTransport it(nh_public);
    pubIm = it.advertise("clusters_image", 1);
    image_transport::Subscriber subIm = it.subscribe("dvs/image_raw", 1, imageCallback);

    ros::spin();
}