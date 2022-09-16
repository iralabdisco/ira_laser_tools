#include <string.h>
#include <vector>
#include <Eigen/Dense>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include "rclcpp/rclcpp.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "pcl_ros/transforms.hpp"

typedef pcl::PointCloud<pcl::PointXYZ> myPointCloud;

using namespace std;
using namespace pcl;

class LaserscanVirtualizer : public rclcpp::Node
{
public:
	LaserscanVirtualizer();
	void pointcloud_to_laserscan(Eigen::MatrixXf points, pcl::PCLHeader scan_header, int pub_index);
	void pointCloudCallback(sensor_msgs::msg::PointCloud2::SharedPtr pcl_in);
	rcl_interfaces::msg::SetParametersResult reconfigureCallback(const std::vector<rclcpp::Parameter> &parameters);

private:
	std::shared_ptr<tf2_ros::TransformListener> tfListener_;
	std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
	std::vector<tf2::Stamped<tf2::Transform>> transform_;
	OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscription_;
	std::vector<rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr> virtual_scan_publishers;
	std::vector<string> output_frames;

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

rcl_interfaces::msg::SetParametersResult LaserscanVirtualizer::reconfigureCallback(const std::vector<rclcpp::Parameter> &parameters)
{
	rcl_interfaces::msg::SetParametersResult result;

	for (auto parameter : parameters)
	{
		const auto &type = parameter.get_type();
		const auto &name = parameter.get_name();

		// Make sure it is a double value
		if (type == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE)
		{
			double value = parameter.as_double();

			if (name == "angle_min")
			{
				this->angle_min = value;
			}
			else if (name == "angle_max")
			{
				this->angle_max = value;
			}
			else if (name == "angle_increment")
			{
				this->angle_increment = value;
			}
			else if (name == "time_increment")
			{
				this->time_increment = value;
			}
			else if (name == "scan_time")
			{
				this->scan_time = value;
			}
			else if (name == "range_min")
			{
				this->range_min = value;
			}
			else if (name == "range_max")
			{
				this->range_max = value;
			}
		}
	}

	result.successful = true;
	return result;
}

void LaserscanVirtualizer::virtual_laser_scan_parser()
{
	// LaserScan frames to use for virtualization
	istringstream iss(virtual_laser_scan);
	std::vector<string> tokens;
	copy(istream_iterator<string>(iss), istream_iterator<string>(), back_inserter<std::vector<string>>(tokens));

	std::vector<string> tmp_output_frames;

	for (std::vector<int>::size_type i = 0; i < tokens.size(); i++)
	{
		auto beg = this->get_clock()->now();
		if (tf_buffer_->canTransform(base_frame, tokens[i], rclcpp::Time(0), rclcpp::Duration(1, 0))) // Check if TF knows the transform from this frame reference to base_frame reference
		{
			cout << "Elapsed: " << (this->get_clock()->now() - beg).nanoseconds() / 1e9 << endl;
			cout << "Adding: " << tokens[i] << endl;
			tmp_output_frames.push_back(tokens[i]);
		}
		else
		{
			cout << "Can't transform: '" << tokens[i] + "' to '" << base_frame << "'" << endl;
		}
	}

	// Sort and remove duplicates
	sort(tmp_output_frames.begin(), tmp_output_frames.end());
	std::vector<string>::iterator last = std::unique(tmp_output_frames.begin(), tmp_output_frames.end());
	tmp_output_frames.erase(last, tmp_output_frames.end());

	// Do not re-advertize if the topics are the same
	if ((tmp_output_frames.size() != output_frames.size()) || !equal(tmp_output_frames.begin(), tmp_output_frames.end(), output_frames.begin()))
	{
		cloud_frame = "";

		output_frames = tmp_output_frames;
		if (output_frames.size() > 0)
		{
			virtual_scan_publishers.resize(output_frames.size());
			RCLCPP_INFO(this->get_logger(), "Publishing: %ld virtual scans", virtual_scan_publishers.size());
			cout << "Advertising topics: " << endl;
			for (std::vector<int>::size_type i = 0; i < output_frames.size(); ++i)
			{
				if (output_laser_topic.empty())
				{
					virtual_scan_publishers[i] = this->create_publisher<sensor_msgs::msg::LaserScan>(output_frames[i].c_str(), rclcpp::SensorDataQoS());
					cout << "\t\t" << output_frames[i] << " on topic " << output_frames[i].c_str() << endl;
				}
				else
				{
					virtual_scan_publishers[i] = this->create_publisher<sensor_msgs::msg::LaserScan>(output_laser_topic.c_str(), rclcpp::SensorDataQoS());
					cout << "\t\t" << output_frames[i] << " on topic " << output_laser_topic.c_str() << endl;
				}
			}
		}
		else
		{
			RCLCPP_INFO(this->get_logger(), "Not publishing to any topic.");
		}
	}
}

LaserscanVirtualizer::LaserscanVirtualizer() : Node("laserscan_virtualizer")
{
	this->declare_parameter<std::string>("base_frame", "base_link");
	this->declare_parameter<std::string>("cloud_topic", "/cloud_pcd");
	this->declare_parameter<std::string>("output_laser_topic", "/scan");
	this->declare_parameter<std::string>("virtual_laser_scan", "scansx scandx");
	this->declare_parameter("angle_min", -3.14);
	this->declare_parameter("angle_max", 3.14);
	this->declare_parameter("angle_increment", 0.0058);
	this->declare_parameter("scan_time", 0.0);
	this->declare_parameter("range_min", 0.0);
	this->declare_parameter("range_max", 25.0);

	this->get_parameter("base_frame", base_frame);
	this->get_parameter("cloud_topic", cloud_topic);
	this->get_parameter("output_laser_topic", output_laser_topic);
	this->get_parameter("virtual_laser_scan", virtual_laser_scan);
	this->get_parameter("angle_min", angle_min);
	this->get_parameter("angle_max", angle_max);
	this->get_parameter("angle_increment", angle_increment);
	this->get_parameter("scan_time", scan_time);
	this->get_parameter("range_min", range_min);
	this->get_parameter("range_max", range_max);

	param_callback_handle_ = this->add_on_set_parameters_callback(
			std::bind(&LaserscanVirtualizer::reconfigureCallback, this, std::placeholders::_1));

	tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
	tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

	this->virtual_laser_scan_parser();

	point_cloud_subscription_ =
			this->create_subscription<sensor_msgs::msg::PointCloud2>(cloud_topic.c_str(), rclcpp::SensorDataQoS(), std::bind(&LaserscanVirtualizer::pointCloudCallback, this, std::placeholders::_1));
	cloud_frame = "";
}

void LaserscanVirtualizer::pointCloudCallback(sensor_msgs::msg::PointCloud2::SharedPtr pcl_in)
{
	if (cloud_frame.empty())
	{
		cloud_frame = (*pcl_in).header.frame_id;
		transform_.resize(output_frames.size());
		for (std::vector<int>::size_type i = 0; i < output_frames.size(); i++)
		{
			if (tf_buffer_->canTransform(output_frames[i], cloud_frame, rclcpp::Time(0), rclcpp::Duration(2, 0)))
			{
				geometry_msgs::msg::TransformStamped tfGeom = tf_buffer_->lookupTransform(output_frames[i], cloud_frame, rclcpp::Time(0));

				tf2::convert(tfGeom, transform_[i]);
			}
		}
	}

	for (std::vector<int>::size_type i = 0; i < output_frames.size(); i++)
	{
		myPointCloud pcl_out, tmpPcl;
		pcl::PCLPointCloud2 tmpPcl2;

		pcl_conversions::toPCL(*pcl_in, tmpPcl2);
		pcl::fromPCLPointCloud2(tmpPcl2, tmpPcl);

		// Initialize the header of the temporary pointcloud, needed to rototranslate the three points that define our plane
		// It shall be equal to the input point cloud's one, changing only the 'frame_id'
		string tmpFrame = output_frames[i];
		pcl_out.header = tmpPcl.header;

		// Ask tf to rototranslate the velodyne pointcloud to base_frame reference
		pcl_ros::transformPointCloud(tmpPcl, pcl_out, transform_[i]);
		pcl_out.header.frame_id = tmpFrame;

		// Transform the pcl into eigen matrix
		Eigen::MatrixXf tmpEigenMatrix;
		pcl::toPCLPointCloud2(pcl_out, tmpPcl2);
		pcl::getPointCloudAsEigen(tmpPcl2, tmpEigenMatrix);

		// Extract the points close to the z=0 plane, convert them into a laser-scan message and publish it
		pointcloud_to_laserscan(tmpEigenMatrix, pcl_out.header, i);
	}
}

void LaserscanVirtualizer::pointcloud_to_laserscan(Eigen::MatrixXf points, pcl::PCLHeader scan_header, int pub_index) // pcl::PCLPointCloud2 *merged_cloud)
{
	sensor_msgs::msg::LaserScan output;
	output.header = pcl_conversions::fromPCL(scan_header);
	output.angle_min = this->angle_min;
	output.angle_max = this->angle_max;
	output.angle_increment = this->angle_increment;
	output.time_increment = this->time_increment;
	output.scan_time = this->scan_time;
	output.range_min = this->range_min;
	output.range_max = this->range_max;

	uint32_t ranges_size = std::ceil((output.angle_max - output.angle_min) / output.angle_increment);
	output.ranges.assign(ranges_size, output.range_max + 1.0);

	for (int i = 0; i < points.cols(); i++)
	{
		const float &x = points(0, i);
		const float &y = points(1, i);
		const float &z = points(2, i);

		if (std::isnan(x) || std::isnan(y) || std::isnan(z))
		{
			RCLCPP_DEBUG(this->get_logger(), "rejected for nan in point(%f, %f, %f)\n", x, y, z);
			continue;
		}

		double range_sq = pow(y, 2) + pow(x, 2);
		double range_min_sq_ = output.range_min * output.range_min;
		if (range_sq < range_min_sq_)
		{
			RCLCPP_DEBUG(this->get_logger(), "rejected for range %f below minimum value %f. Point: (%f, %f, %f)", range_sq, range_min_sq_, x, y, z);
			continue;
		}

		double angle = atan2(y, x);
		if (angle < output.angle_min || angle > output.angle_max)
		{
			RCLCPP_DEBUG(this->get_logger(), "rejected for angle %f not in range (%f, %f)\n", angle, output.angle_min, output.angle_max);
			continue;
		}

		int index = (angle - output.angle_min) / output.angle_increment;
		if (output.ranges[index] * output.ranges[index] > range_sq)
		{
			output.ranges[index] = sqrt(range_sq);
		}
	}

	virtual_scan_publishers[pub_index]->publish(output);
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);

	rclcpp::spin(std::make_shared<LaserscanVirtualizer>());

	rclcpp::shutdown();

	return 0;
}
