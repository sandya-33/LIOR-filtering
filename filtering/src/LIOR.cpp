#include <ros/ros.h>

// PCL includes
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

//Conversion
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

//pcl_ros
#include <pcl_ros/point_cloud.h>

//Custom filter requirements
#include <filtering/LIOR.h>
#include <filtering/LIOR.hpp>
#include <pcl/kdtree/kdtree_flann.h>

//For benchmarking
#include <chrono>


using namespace std;

class Main {
    typedef pcl::PointXYZI PointT;
    typedef pcl::PointCloud<PointT> PointCloud;

protected:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher pub;
private:
    pcl::LIOR<PointT> cust_filter;  
    pcl::KdTreeFLANN<PointT> kdFLANN;
public:    
    int public_var;
    double detection_range=71.235;
    double min_nbrs_=3;
    string incoming_topic;
    string outgoing_topic;

~Main () {

        nh = ros::NodeHandle("~");
        incoming_topic = "/input/to/PointCloud2";
        outgoing_topic = "/output/to/PointCloud2";

        cust_filter.setSearchMethod(kdFLANN);
        cust_filter.setPrivateVariable(public_var);
        cust_filter.setDetection_range(detection_range);
        cust_filter.setMinNeighborsInRadius(min_nbrs_);

        //Subscribing
        sub = nh.subscribe(incoming_topic, 1, &Main::cloud_cb, this);
        ROS_INFO ("Listening for incoming data on topic %s", nh.resolveName(incoming_topic).c_str());

        //Publishing
        pub = nh.advertise<PointCloud>(outgoing_topic, 1);
    }
~Main() {}

    void set_params() {
        /*
        * Basic set param Template
        * 
        * string check;
        * nh.getParam("param", check);
        * 
        * rosrun ... _param:=string_value
        * 
        */ 

        if(nh.hasParam("input")){
            nh.getParam("input", incoming_topic);
        }
        if(nh.hasParam("output")){
            nh.getParam("output", outgoing_topic);
        }
    }

void cloud_cb (const sensor_msgs::PointCloud2::ConstPtr& input) {
        if ((input->width * input->height) == 0)
            return;

        ROS_INFO ("Received %d data points in frame %s with the following fields: %s",
                  (int) input->width * input->height,
                  input->header.frame_id.c_str(),
                  pcl::getFieldsList(*input).c_str());

        PointCloud::Ptr unfilter_cloud (new PointCloud);
        PointCloud::Ptr filter_cloud (new PointCloud);
        pcl::fromROSMsg(*input,*unfilter_cloud);
        auto unfil_points = unfilter_cloud->width * unfilter_cloud->height;

        cust_filter.setInputCloud(unfilter_cloud);

        cust_filter.applyFilter(*filter_cloud);
        
        auto fil_points = (*filter_cloud).width * (*filter_cloud).height;

        filter_cloud->header = unfilter_cloud->header;
        filter_cloud->is_dense = false;
        pub.publish(*filter_cloud);
    }
};

int main (int argc, char** argv) {
    ros::init(argc, argv, "node_name");
    Main frm;

    ros::spin();

    return (0);
}


