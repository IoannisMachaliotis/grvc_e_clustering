#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <dvs_msgs/EventArray.h>
#include "grvc_e_clustering/AEClustering.h"

AEClustering *eclustering(new AEClustering);


image_transport::Publisher pubIm;
sensor_msgs::ImagePtr im_msg;


void imageCallback(const sensor_msgs::ImageConstPtr& msg){
    try{
        int minN = eclustering->getMinN(); 
        cv::Mat im = cv_bridge::toCvShare(msg, "rgb8")->image; 
        for (auto cc : eclustering->clusters){
            if (cc.getN() >= minN){
                Eigen::VectorXd cen(cc.getClusterCentroid());
                cv::circle(im,cv::Point(cen[0],cen[1]), 1, cv::Scalar(0,255,0,255), -1);
            }
        }
        im_msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", im).toImageMsg();
        pubIm.publish(im_msg);
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    } 
}


void eventCallback(const dvs_msgs::EventArray::ConstPtr &msg){
    std::deque<double> ev(4, 0.0);
    for (const auto e : msg->events){
        ev[0] = e.ts.toSec();       
        ev[1] = e.x;
        ev[2] = e.y;
        ev[3] = e.polarity;
        eclustering->update(ev);
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

