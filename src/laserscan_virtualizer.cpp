#include <ros/ros.h>
#include <string.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/point_cloud_conversion.h> 
#include "sensor_msgs/LaserScan.h"
#include <pcl_ros/point_cloud.h>
#include <Eigen/Dense>
#include <dynamic_reconfigure/server.h>
#include <ira_laser_tools/laserscan_virtualizerConfig.h>

typedef pcl::PointCloud<pcl::PointXYZ> myPointCloud;

using namespace std;
using namespace pcl;
using namespace laserscan_virtualizer;

class LaserscanVirtualizer
{
	public:
        LaserscanVirtualizer();
		void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan, std::string topic);
		void pointcloud_to_laserscan(Eigen::MatrixXf points, pcl::PCLHeader scan_header, int pub_index);
		void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& pcl_in);
        void reconfigureCallback(laserscan_virtualizerConfig &config, uint32_t level);

	private:
		ros::NodeHandle node_;
		tf::TransformListener tfListener_;
		vector<tf::StampedTransform> transform_;

		ros::Subscriber point_cloud_subscriber_;
		vector<ros::Publisher> virtual_scan_publishers;
		vector<string> output_frames;

        void virtual_laser_scan_parser();

		double angle_min;
		double angle_max;
		double angle_increment;
		double time_increment;
		double scan_time;
		double range_min;
		double range_max;

		string cloud_frame;
		string base_frame;
        string cloud_topic;
        string output_laser_topic;
        string virtual_laser_scan;
};

void LaserscanVirtualizer::reconfigureCallback(laserscan_virtualizerConfig &config, uint32_t level)
{
	this->angle_min = config.angle_min;
	this->angle_max = config.angle_max;
	this->angle_increment = config.angle_increment;
	this->time_increment = config.time_increment;
    this->scan_time = config.scan_time;
	this->range_min = config.range_min;
	this->range_max = config.range_max;
}

void LaserscanVirtualizer::virtual_laser_scan_parser()
{
	// LaserScan frames to use for virtualization
    istringstream iss(virtual_laser_scan);
	vector<string> tokens;
	copy(istream_iterator<string>(iss), istream_iterator<string>(), back_inserter<vector<string> >(tokens));

	vector<string> tmp_output_frames;

	for(int i=0;i<tokens.size();++i)
	{
		ros::Time beg = ros::Time::now();
        if(tfListener_.waitForTransform (base_frame, tokens[i] ,ros::Time(0),ros::Duration(1.0))) // Check if TF knows the transform from this frame reference to base_frame reference
		{
			cout << "Elapsed: " << ros::Time::now() - beg << endl;
            cout << "Adding: " << tokens[i]  << endl;
			tmp_output_frames.push_back(tokens[i]);
		}
		else
            cout << "Can't transform: '" << tokens[i] + "' to '" << base_frame << "'" << endl;
	}

	// Sort and remove duplicates
	sort(tmp_output_frames.begin(),tmp_output_frames.end());
	std::vector<string>::iterator last = std::unique(tmp_output_frames.begin(), tmp_output_frames.end());
	tmp_output_frames.erase(last, tmp_output_frames.end());

	// Do not re-advertize if the topics are the same
	if( (tmp_output_frames.size() != output_frames.size()) || !equal(tmp_output_frames.begin(),tmp_output_frames.end(),output_frames.begin()) )
	{
		// Shutdown previous publishers
		for(int i=0; i<virtual_scan_publishers.size(); ++i)
			virtual_scan_publishers[i].shutdown();

		cloud_frame = "";

		output_frames = tmp_output_frames;
		if(output_frames.size() > 0)
		{
            virtual_scan_publishers.resize(output_frames.size());
            ROS_INFO("Publishing: %ld virtual scans", virtual_scan_publishers.size() );
            cout << "Advertising topics: " << endl;
			for(int i=0; i<output_frames.size(); ++i)
			{
                if (output_laser_topic.empty())
                {
                    virtual_scan_publishers[i] = node_.advertise<sensor_msgs::LaserScan> (output_frames[i].c_str(), 1);
                    cout << "\t\t" << output_frames[i] << " on topic " << output_frames[i].c_str() << endl;
                }
                else
                {
                    virtual_scan_publishers[i] = node_.advertise<sensor_msgs::LaserScan> (output_laser_topic.c_str(), 1);
                    cout << "\t\t" << output_frames[i] << " on topic " << output_laser_topic.c_str() << endl;
                }
			}
		}
		else
            ROS_INFO("Not publishing to any topic.");
	}
}

LaserscanVirtualizer::LaserscanVirtualizer()
{
    ros::NodeHandle nh("~");

    //Setting class parameters
    if(!nh.getParam("/laserscan_virtualizer/base_frame", base_frame))                   base_frame = "/cart_frame";
    if(!nh.getParam("/laserscan_virtualizer/cloud_topic", cloud_topic))                 cloud_topic = "/cloud_pcd";
    nh.getParam("/laserscan_virtualizer/output_laser_topic", output_laser_topic);
    nh.getParam("/laserscan_virtualizer/virtual_laser_scan", virtual_laser_scan);

    this->virtual_laser_scan_parser();

    point_cloud_subscriber_ = node_.subscribe<sensor_msgs::PointCloud2> (cloud_topic.c_str(), 1, boost::bind(&LaserscanVirtualizer::pointCloudCallback,this, _1));
	cloud_frame = "";
}

void LaserscanVirtualizer::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& pcl_in)
{
	if(cloud_frame.empty())
	{
		cloud_frame = (*pcl_in).header.frame_id;
		transform_.resize(output_frames.size());
		for(int i=0; i<output_frames.size(); ++i)
            if(tfListener_.waitForTransform (output_frames[i] , cloud_frame, ros::Time(0),ros::Duration(2.0)))
                tfListener_.lookupTransform (output_frames[i] , cloud_frame, ros::Time(0), transform_[i]);
	}

	for(int i=0; i<output_frames.size(); ++i)
	{
        myPointCloud pcl_out, tmpPcl;
		pcl::PCLPointCloud2 tmpPcl2;

		pcl_conversions::toPCL(*pcl_in, tmpPcl2);
		pcl::fromPCLPointCloud2(tmpPcl2,tmpPcl);

		// Initialize the header of the temporary pointcloud, needed to rototranslate the three points that define our plane
		// It shall be equal to the input point cloud's one, changing only the 'frame_id'
        string tmpFrame = output_frames[i] ;
		pcl_out.header = tmpPcl.header;
	
		// Ask tf to rototranslate the velodyne pointcloud to base_frame reference
		pcl_ros::transformPointCloud(tmpPcl, pcl_out, transform_[i]);
		pcl_out.header.frame_id = tmpFrame;

		// Transform the pcl into eigen matrix
		Eigen::MatrixXf tmpEigenMatrix;
		pcl::toPCLPointCloud2(pcl_out, tmpPcl2);
		pcl::getPointCloudAsEigen(tmpPcl2,tmpEigenMatrix);

		// Extract the points close to the z=0 plane, convert them into a laser-scan message and publish it
		pointcloud_to_laserscan(tmpEigenMatrix, pcl_out.header, i);
	}
}


void LaserscanVirtualizer::pointcloud_to_laserscan(Eigen::MatrixXf points, pcl::PCLHeader scan_header, int pub_index) //pcl::PCLPointCloud2 *merged_cloud)
{
	sensor_msgs::LaserScanPtr output(new sensor_msgs::LaserScan());
    output->header = pcl_conversions::fromPCL(scan_header);
    output->angle_min = this->angle_min;
	output->angle_max = this->angle_max;
	output->angle_increment = this->angle_increment;
    output->time_increment = this->time_increment;
    output->scan_time = this->scan_time;
	output->range_min = this->range_min;
	output->range_max = this->range_max;

	uint32_t ranges_size = std::ceil((output->angle_max - output->angle_min) / output->angle_increment);
	output->ranges.assign(ranges_size, output->range_max + 1.0);

	for(int i=0; i<points.cols(); i++)
	{
		const float &x = points(0,i);
		const float &y = points(1,i);
		const float &z = points(2,i);

		if ( std::isnan(x) || std::isnan(y) || std::isnan(z) )
		{
			ROS_DEBUG("rejected for nan in point(%f, %f, %f)\n", x, y, z);
			continue;
		}

        if ( std::abs(z) > 0.01 )
			continue;

        pcl::PointXYZ p;
        p.x=x;
        p.y=y;
        p.z=z;

		double range_sq = y*y+x*x;
		double range_min_sq_ = output->range_min * output->range_min;
		if (range_sq < range_min_sq_) {
			ROS_DEBUG("rejected for range %f below minimum value %f. Point: (%f, %f, %f)", range_sq, range_min_sq_, x, y, z);
			continue;
		}

		double angle = atan2(y, x);
		if (angle < output->angle_min || angle > output->angle_max)
		{
			ROS_DEBUG("rejected for angle %f not in range (%f, %f)\n", angle, output->angle_min, output->angle_max);
			continue;
		}
		int index = (angle - output->angle_min) / output->angle_increment;


		if (output->ranges[index] * output->ranges[index] > range_sq)
			output->ranges[index] = sqrt(range_sq);
	}

    virtual_scan_publishers[pub_index].publish(output);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "laserscan_virtualizer");

    LaserscanVirtualizer _laser_merger;

    dynamic_reconfigure::Server<laserscan_virtualizerConfig> server;
    dynamic_reconfigure::Server<laserscan_virtualizerConfig>::CallbackType f;

    f = boost::bind(&LaserscanVirtualizer::reconfigureCallback,&_laser_merger, _1, _2);
	server.setCallback(f);

	ros::spin();

	return 0;
}
