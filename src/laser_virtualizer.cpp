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
#include <ira_laser_tools/laser_virtualizerConfig.h>

typedef pcl::PointCloud<pcl::PointXYZ> myPointCloud;

using namespace std;
using namespace pcl;
using namespace laser_virtualizer;

class LaserMerger
{
	public:
		LaserMerger();
		void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan, std::string topic);
		void pointcloud_to_laserscan(Eigen::MatrixXf points, pcl::PCLHeader scan_header, int pub_index);
		void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& pcl_in);
		void reconfigureCallback(laser_virtualizerConfig &config, uint32_t level);
	private:
		ros::NodeHandle node_;
		tf::TransformListener tfListener_;
		vector<tf::StampedTransform> transform_;

		ros::Subscriber point_cloud_subscriber_;
		vector<ros::Publisher> virtual_scan_publishers;
		vector<string> output_frames;

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
};

void LaserMerger::reconfigureCallback(laser_virtualizerConfig &config, uint32_t level)
{
	this->angle_min = config.angle_min;
	this->angle_max = config.angle_max;
	this->angle_increment = config.angle_increment;
	this->time_increment = config.time_increment;
	this->scan_time = config.scan_time;
	this->range_min = config.range_min;
	this->range_max = config.range_max;

	// LaserScan frames to use for virtualization
	istringstream iss(config.str_param);
	vector<string> tokens;
	copy(istream_iterator<string>(iss), istream_iterator<string>(), back_inserter<vector<string> >(tokens));

	vector<string> tmp_output_frames;

	for(int i=0;i<tokens.size();++i)
	{
		ros::Time beg = ros::Time::now();
		if(tfListener_.waitForTransform (base_frame, tokens[i] + "_frame",ros::Time(0),ros::Duration(1.0))) // Check if TF knows the transform from this frame reference to base_frame reference
		{
			cout << "Elapsed: " << ros::Time::now() - beg << endl;
			cout << "Adding: " << tokens[i] + "_frame" << endl;
			tmp_output_frames.push_back(tokens[i]);
		}
		else
			cout << "Can't transform: '" << tokens[i] + "_frame' to '" << base_frame << "'" << endl;
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
//			cout << "Publishing: " << virtual_scan_publishers.size() << " virtual scans" << endl;
			virtual_scan_publishers.resize(output_frames.size());
			cout << "Publishing: " << virtual_scan_publishers.size() << " virtual scans" << endl;
			cout << "Advertising topics: ";
			for(int i=0; i<output_frames.size(); ++i)
			{
				virtual_scan_publishers[i] = node_.advertise<sensor_msgs::LaserScan> (output_frames[i].c_str(), 1);
				cout << output_frames[i] << " ";

			}
			cout << endl;
		}
		else
			cout << "Not publishing to any topic." << endl;
	}
}

LaserMerger::LaserMerger()
{
	ros::NodeHandle nh("~");
	if(!nh.getParam("base_frame", base_frame))
		base_frame = "/cart_frame";
	if(!nh.getParam("cloud_topic", cloud_topic))
		cloud_topic = "/cloud_pcd";

	point_cloud_subscriber_ = node_.subscribe<sensor_msgs::PointCloud2> (cloud_topic.c_str(), 1, boost::bind(&LaserMerger::pointCloudCallback,this, _1));
	cloud_frame = "";
}

void LaserMerger::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& pcl_in)
{
	if(cloud_frame.empty())
	{
		cloud_frame = (*pcl_in).header.frame_id;
		transform_.resize(output_frames.size());
		for(int i=0; i<output_frames.size(); ++i)
			if(tfListener_.waitForTransform (output_frames[i] + "_frame", cloud_frame, ros::Time(0),ros::Duration(2.0)))
				tfListener_.lookupTransform (output_frames[i] + "_frame", cloud_frame, ros::Time(0), transform_[i]);
	}

	for(int i=0; i<output_frames.size(); ++i)
	{
		myPointCloud pcl_out, tmpPcl, tmpPclRt;
		pcl::PCLPointCloud2 tmpPcl2;

		pcl_conversions::toPCL(*pcl_in, tmpPcl2);
		pcl::fromPCLPointCloud2(tmpPcl2,tmpPcl);

		// Initialize the header of the temporary pointcloud, needed to rototranslate the three points that define our plane
		// It shall be equal to the input point cloud's one, changing only the 'frame_id'
		string tmpFrame = output_frames[i] + "_frame";
		pcl_out.header = tmpPcl.header;
	
		// Ask tf to rototranslate the velodyne pointcloud to base_frame reference
//		ros::Time beg = ros::Time::now();
		pcl_ros::transformPointCloud(tmpPcl, pcl_out, transform_[i]);
		pcl_out.header.frame_id = tmpFrame;
//		ros::Time end = ros::Time::now();
//		cout << "Elapsed: " << end - beg << endl;

//		beg = ros::Time::now();
		// Transform the pcl into eigen matrix
		Eigen::MatrixXf tmpEigenMatrix;
		pcl::toPCLPointCloud2(pcl_out, tmpPcl2);
		pcl::getPointCloudAsEigen(tmpPcl2,tmpEigenMatrix);
//		end = ros::Time::now();
//		cout << "Elapsed: " << end - beg << endl;

//		beg = ros::Time::now();
		// Extract the points close to the z=0 plane, convert them into a laser-scan message and publish it
		pointcloud_to_laserscan(tmpEigenMatrix, pcl_out.header, i);
//		end = ros::Time::now();
//		cout << "Elapsed: " << end - beg << endl;
	}
}

/*
	// Create the three points that define the plane in the virtual-laser-frame reference
	tmpPcl.points.push_back(pcl::PointXYZ(0.0,0.0,0.0));
	tmpPcl.points.push_back(pcl::PointXYZ(1.0,0.0,0.0));
	tmpPcl.points.push_back(pcl::PointXYZ(0.0,1.0,0.0));

	// Ask tf to rototranslate these points into the base_frame reference
	tfListener_.waitForTransform(tmpFrame, base_frame, ros::Time((*pcl_in).header.stamp), ros::Duration(1));
	pcl_ros::transformPointCloud(base_frame, tmpPcl, tmpPclRt, tfListener_);

	// Define the three points that define our plane in base_frame reference
	Eigen::Vector3f pt0, pt1, pt2;
	pt0.x() = tmpPclRt.points[0].x;
	pt0.y() = tmpPclRt.points[0].y;
	pt0.z() = tmpPclRt.points[0].z;

	pt1.x() = tmpPclRt.points[1].x;
	pt1.y() = tmpPclRt.points[1].y;
	pt1.z() = tmpPclRt.points[1].z;

	pt2.x() = tmpPclRt.points[2].x;
	pt2.y() = tmpPclRt.points[2].y;
	pt2.z() = tmpPclRt.points[2].z;

	// Calculate the plane normal versor
	Eigen::Vector3f n = (pt1-pt0).cross(pt2-pt0);
	n.normalize();

	// Convert the point cloud to an Eigen matrix
	Eigen::MatrixXf tmpEigenMatrix;
	pcl::toPCLPointCloud2(pcl_out, tmpPcl2);
	pcl::getPointCloudAsEigen(tmpPcl2,tmpEigenMatrix);

	tmpEigenMatrix = tmpEigenMatrix.transpose(); // Get the matrix as a 3xN

	tmpEigenMatrix.colwise() -= pt0; // Subtract pt0 to each column of the matrix

	Eigen::VectorXf ptsDists = n.transpose() * tmpEigenMatrix; // Get the dot product of n with every column of the matrix, which represents each point's distance from the plane

	ptsDists = ptsDists.cwiseAbs(); // Get the absolute value of the distance of each point from the plane

	Eigen::MatrixXi ptsIndexes = (ptsDists.array() <= 0.1).cast<int>(); // Get the indexes of only those points that are closer than 0.1

	Eigen::MatrixXf planePts;
	planePts =  Eigen::MatrixXf::Zero(3,ptsIndexes.sum()); // Pre-allocate the matrix that will contain the selected points

	// Populate the matrix with the selected points
	int colCounter = 0;
	for(int i=0;i<ptsIndexes.cols();++i)
		if(ptsIndexes(i))
			planePts.col(colCounter++) = tmpEigenMatrix.col(i);	
*/

void LaserMerger::pointcloud_to_laserscan(Eigen::MatrixXf points, pcl::PCLHeader scan_header, int pub_index) //pcl::PCLPointCloud2 *merged_cloud)
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

		if ( std::abs(z) > 0.1 )
			continue;

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
	ros::init(argc, argv, "laser_virtualizer");

	LaserMerger _laser_merger;

	dynamic_reconfigure::Server<laser_virtualizerConfig> server;
	dynamic_reconfigure::Server<laser_virtualizerConfig>::CallbackType f;

	f = boost::bind(&LaserMerger::reconfigureCallback,&_laser_merger, _1, _2);
	server.setCallback(f);

	ros::spin();

	return 0;
}
