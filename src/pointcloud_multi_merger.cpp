#include <ros/ros.h>
#include <string.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h> 
#include "pcl_ros/point_cloud.h"
#include <Eigen/Dense>
#include <dynamic_reconfigure/server.h>
#include <ira_laser_tools/pointcloud_multi_mergerConfig.h>

using namespace std;
using namespace pcl;
using namespace pointcloud_multi_merger;

class PointCloudMerger
{
public:
    PointCloudMerger();
    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& pcd, std::string topic);
    void reconfigureCallback(pointcloud_multi_mergerConfig &config, uint32_t level);

private:
    ros::NodeHandle node_;
    tf::TransformListener tfListener_;

    ros::Publisher point_cloud_publisher_;
    vector<ros::Subscriber> cloud_subscribers;
    vector<bool> clouds_modified;

    vector<pcl::PCLPointCloud2> clouds;
    vector<string> input_topics;

    void pointcloud_topic_parser();

    double angle_min;
    double angle_max;
    double angle_increment;
    double time_increment;
    double scan_time;
    double range_min;
    double range_max;

    string destination_frame;
    string cloud_destination_topic;
    string pointcloud_topics;
};

void PointCloudMerger::reconfigureCallback(pointcloud_multi_mergerConfig &config, uint32_t level)
{
	this->angle_min = config.angle_min;
	this->angle_max = config.angle_max;
	this->angle_increment = config.angle_increment;
	this->time_increment = config.time_increment;
	this->scan_time = config.scan_time;
	this->range_min = config.range_min;
	this->range_max = config.range_max;
}

void PointCloudMerger::pointcloud_topic_parser()
{
	// PointCloud topics to subscribe
	ros::master::V_TopicInfo topics;
	ros::master::getTopics(topics);

    istringstream iss(pointcloud_topics);
	vector<string> tokens;
	copy(istream_iterator<string>(iss), istream_iterator<string>(), back_inserter<vector<string> >(tokens));
	vector<string> tmp_input_topics;
	for(int i=0;i<tokens.size();++i)
	{
        for(int j=0;j<topics.size();++j)
		{
			if( (tokens[i].compare(topics[j].name) == 0) && (topics[j].datatype.compare("sensor_msgs/PointCloud2") == 0) )
			{
				tmp_input_topics.push_back(topics[j].name);
			}
		}
	}

	sort(tmp_input_topics.begin(),tmp_input_topics.end());
	std::vector<string>::iterator last = std::unique(tmp_input_topics.begin(), tmp_input_topics.end());
	tmp_input_topics.erase(last, tmp_input_topics.end());


	// Do not re-subscribe if the topics are the same
	if( (tmp_input_topics.size() != input_topics.size()) || !equal(tmp_input_topics.begin(),tmp_input_topics.end(),input_topics.begin()))
	{

		// Unsubscribe from previous topics
		for(int i=0; i<cloud_subscribers.size(); ++i)
			cloud_subscribers[i].shutdown();

		input_topics = tmp_input_topics;
		if(input_topics.size() > 0)
		{
            cloud_subscribers.resize(input_topics.size());
			clouds_modified.resize(input_topics.size());
			clouds.resize(input_topics.size());
            ROS_INFO("Subscribing to topics\t%ld", cloud_subscribers.size());
			for(int i=0; i<input_topics.size(); ++i)
			{
                cloud_subscribers[i] = node_.subscribe<sensor_msgs::PointCloud2> (input_topics[i].c_str(), 1, boost::bind(&PointCloudMerger::pointCloudCallback,this, _1, input_topics[i]));
				clouds_modified[i] = false;
				cout << input_topics[i] << " ";
			}
		}
		else
            ROS_INFO("Not subscribed to any topic.");
	}
}

PointCloudMerger::PointCloudMerger()
{
	ros::NodeHandle nh("~");

    nh.param<std::string>("destination_frame", destination_frame, "cart_frame");
    nh.param<std::string>("cloud_destination_topic", cloud_destination_topic, "/merged_cloud");
    nh.param<std::string>("pointcloud_topics", pointcloud_topics, "");
    nh.param("angle_min", angle_min, -2.36);
    nh.param("angle_max", angle_max, 2.36);
    nh.param("angle_increment", angle_increment, 0.0058);
    nh.param("scan_time", scan_time, 0.0333333);
    nh.param("range_min", range_min, 0.45);
    nh.param("range_max", range_max, 25.0);

    this->pointcloud_topic_parser();

	point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud2> (cloud_destination_topic.c_str(), 1, false);

}

void PointCloudMerger::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& pcd, std::string topic)
{
	sensor_msgs::PointCloud2 tmpCloud;
    tf::StampedTransform transform;

	tfListener_.waitForTransform(pcd->header.frame_id.c_str(), destination_frame.c_str(), pcd->header.stamp, ros::Duration(10));
	try
	{
	    pcl_ros::transformPointCloud(destination_frame.c_str(), *pcd, tmpCloud, tfListener_);
	}catch (tf::TransformException ex){ROS_ERROR("%s",ex.what());return;}

	for(int i=0; i<input_topics.size(); ++i)
	{
		if(topic.compare(input_topics[i]) == 0)
		{
			pcl_conversions::toPCL(tmpCloud, clouds[i]);
			clouds_modified[i] = true;
		}
	}	

    // Count how many clouds we have
	int totalClouds = 0;
	for(int i=0; i<clouds_modified.size(); ++i)
		if(clouds_modified[i])
			++totalClouds;

    // Go ahead only if all subscribed clouds have arrived
	if(totalClouds == clouds_modified.size())
	{
		pcl::PCLPointCloud2 merged_cloud = clouds[0];
		clouds_modified[0] = false;

		for(int i=1; i<clouds_modified.size(); ++i)
		{
			pcl::concatenatePointCloud(merged_cloud, clouds[i], merged_cloud);
			clouds_modified[i] = false;
		}
	
		point_cloud_publisher_.publish(merged_cloud);
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "pointcloud_multi_merger");

    PointCloudMerger _pcd_merger;

    dynamic_reconfigure::Server<pointcloud_multi_mergerConfig> server;
    dynamic_reconfigure::Server<pointcloud_multi_mergerConfig>::CallbackType f;

    f = boost::bind(&PointCloudMerger::reconfigureCallback,&_pcd_merger, _1, _2);
	server.setCallback(f);

	ros::spin();

	return 0;
}
