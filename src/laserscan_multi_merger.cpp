#include <string.h>
#include <vector>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <laser_geometry/laser_geometry.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "pcl_ros/transforms.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std;
using namespace pcl;

using std::placeholders::_1;

class LaserscanMerger : public rclcpp::Node
{
public:
	LaserscanMerger();
	void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan, std::string topic);
	void pointcloud_to_laserscan(Eigen::MatrixXf points, pcl::PCLPointCloud2 *merged_cloud);
	rcl_interfaces::msg::SetParametersResult reconfigureCallback(const std::vector<rclcpp::Parameter> &parameters);

private:
	rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
	laser_geometry::LaserProjection projector_;
	std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
	std::shared_ptr<tf2_ros::TransformListener> tfListener_;
	OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher_;
	rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_publisher_;
	std::vector<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr> scan_subscribers;
	std::vector<bool> clouds_modified;

	std::vector<pcl::PCLPointCloud2> clouds;
	std::vector<string> input_topics;

	void laserscan_topic_parser();

	double angle_min;
	double angle_max;
	double angle_increment;
	double time_increment;
	double scan_time;
	double range_min;
	double range_max;

	string destination_frame;
	string cloud_destination_topic;
	string scan_destination_topic;
	string laserscan_topics;
};

LaserscanMerger::LaserscanMerger() : Node("laserscan_multi_merger")
{
	this->declare_parameter<std::string>("destination_frame", "cart_frame");
	this->declare_parameter<std::string>("cloud_destination_topic", "/merged_cloud");
	this->declare_parameter<std::string>("scan_destination_topic", "/scan_multi");
	this->declare_parameter<std::string>("laserscan_topics", "");
	this->declare_parameter("angle_min", -3.14);
	this->declare_parameter("angle_max", 3.14);
	this->declare_parameter("angle_increment", 0.0058);
	this->declare_parameter("scan_time", 0.0);
	this->declare_parameter("range_min", 0.0);
	this->declare_parameter("range_max", 25.0);

	this->get_parameter("destination_frame", destination_frame);
	this->get_parameter("cloud_destination_topic", cloud_destination_topic);
	this->get_parameter("scan_destination_topic", scan_destination_topic);
	this->get_parameter("laserscan_topics", laserscan_topics);
	this->get_parameter("angle_min", angle_min);
	this->get_parameter("angle_max", angle_max);
	this->get_parameter("angle_increment", angle_increment);
	this->get_parameter("scan_time", scan_time);
	this->get_parameter("range_min", range_min);
	this->get_parameter("range_max", range_max);

	param_callback_handle_ = this->add_on_set_parameters_callback(
			std::bind(&LaserscanMerger::reconfigureCallback, this, _1));

	tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
	tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

	this->laserscan_topic_parser();

	point_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(cloud_destination_topic.c_str(), rclcpp::SensorDataQoS());
	laser_scan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(scan_destination_topic.c_str(), rclcpp::SensorDataQoS());
}

rcl_interfaces::msg::SetParametersResult LaserscanMerger::reconfigureCallback(const std::vector<rclcpp::Parameter> &parameters)
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

void LaserscanMerger::laserscan_topic_parser()
{
	// LaserScan topics to subscribe
	std::map<std::string, std::vector<std::string>> topics;

	istringstream iss(laserscan_topics);
	set<string> tokens;
	copy(istream_iterator<string>(iss), istream_iterator<string>(), inserter<set<string>>(tokens, tokens.begin()));
	std::vector<string> tmp_input_topics;

	while (!tokens.empty())
	{
		RCLCPP_INFO(this->get_logger(), "Waiting for topics ...");
		sleep(1);

		topics = this->get_topic_names_and_types();

		for (const auto &topic_it : topics)
		{
			std::vector<std::string> topic_types = topic_it.second;

			if (std::find(topic_types.begin(), topic_types.end(), "sensor_msgs/msg/LaserScan") != topic_types.end() && tokens.erase(topic_it.first) > 0)
			{
				tmp_input_topics.push_back(topic_it.first);
			}
		}
	}

	sort(tmp_input_topics.begin(), tmp_input_topics.end());
	std::vector<string>::iterator last = std::unique(tmp_input_topics.begin(), tmp_input_topics.end());
	tmp_input_topics.erase(last, tmp_input_topics.end());

	// Do not re-subscribe if the topics are the same
	if ((tmp_input_topics.size() != input_topics.size()) || !equal(tmp_input_topics.begin(), tmp_input_topics.end(), input_topics.begin()))
	{
		input_topics = tmp_input_topics;

		if (input_topics.size() > 0)
		{
			scan_subscribers.resize(input_topics.size());
			clouds_modified.resize(input_topics.size());
			clouds.resize(input_topics.size());
			RCLCPP_INFO(this->get_logger(), "Subscribing to topics\t%ld", scan_subscribers.size());
			for (std::vector<int>::size_type i = 0; i < input_topics.size(); ++i)
			{
				// workaround for std::bind https://github.com/ros2/rclcpp/issues/583
				std::function<void(const sensor_msgs::msg::LaserScan::SharedPtr)> callback =
						std::bind(
								&LaserscanMerger::scanCallback,
								this, std::placeholders::_1, input_topics[i]);
				scan_subscribers[i] = this->create_subscription<sensor_msgs::msg::LaserScan>(input_topics[i].c_str(), rclcpp::SensorDataQoS(), callback);
				clouds_modified[i] = false;
				cout << input_topics[i] << " ";
			}
		}
		else
		{
			RCLCPP_INFO(this->get_logger(), "Not subscribed to any topic.");
		}
	}
}

void LaserscanMerger::scanCallback(sensor_msgs::msg::LaserScan::SharedPtr scan, std::string topic)
{
	sensor_msgs::msg::PointCloud2 tmpCloud1, tmpCloud2;

	try
	{
		// Verify that TF knows how to transform from the received scan to the destination scan frame
		tf_buffer_->lookupTransform(scan->header.frame_id.c_str(), destination_frame.c_str(), scan->header.stamp, rclcpp::Duration(1, 0));
		projector_.transformLaserScanToPointCloud(scan->header.frame_id, *scan, tmpCloud1, *tf_buffer_, laser_geometry::channel_option::Distance);
		pcl_ros::transformPointCloud(destination_frame.c_str(), tmpCloud1, tmpCloud2, *tf_buffer_);
	}
	catch (tf2::TransformException &ex)
	{
		return;
	}

	for (std::vector<int>::size_type i = 0; i < input_topics.size(); i++)
	{
		if (topic.compare(input_topics[i]) == 0)
		{
			pcl_conversions::toPCL(tmpCloud2, clouds[i]);
			clouds_modified[i] = true;
		}
	}

	// Count how many scans we have
	std::vector<int>::size_type totalClouds = 0;
	for (std::vector<int>::size_type i = 0; i < clouds_modified.size(); i++)
	{
		if (clouds_modified[i])
		{
			totalClouds++;
		}
	}

	// Go ahead only if all subscribed scans have arrived
	if (totalClouds == clouds_modified.size())
	{
		pcl::PCLPointCloud2 merged_cloud = clouds[0];
		clouds_modified[0] = false;

		for (std::vector<int>::size_type i = 1; i < clouds_modified.size(); i++)
		{
#if PCL_VERSION_COMPARE(>=, 1, 10, 0)
			merged_cloud += clouds[i];
#else
			pcl::concatenatePointCloud(merged_cloud, clouds[i], merged_cloud);
#endif

			clouds_modified[i] = false;
		}

		Eigen::MatrixXf points;

		pcl::getPointCloudAsEigen(merged_cloud, points);

		pointcloud_to_laserscan(points, &merged_cloud);

		// Publish point cloud after publishing laser scan as for some reason moveFromPCL is causing getPointCloudAsEigen to
		// throw a segmentation fault crash
		std::shared_ptr<sensor_msgs::msg::PointCloud2> cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();

		pcl_conversions::moveFromPCL(merged_cloud, *cloud_msg);

		point_cloud_publisher_->publish(*cloud_msg);
	}
}

void LaserscanMerger::pointcloud_to_laserscan(Eigen::MatrixXf points, pcl::PCLPointCloud2 *merged_cloud)
{
	sensor_msgs::msg::LaserScan output;
	output.header = pcl_conversions::fromPCL(merged_cloud->header);
	output.angle_min = this->angle_min;
	output.angle_max = this->angle_max;
	output.angle_increment = this->angle_increment;
	output.time_increment = this->time_increment;
	output.scan_time = this->scan_time;
	output.range_min = this->range_min;
	output.range_max = this->range_max;

	uint32_t ranges_size = std::ceil((output.angle_max - output.angle_min) / output.angle_increment);
	output.ranges.assign(ranges_size, std::numeric_limits<double>::infinity());

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

	laser_scan_publisher_->publish(output);
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);

	rclcpp::spin(std::make_shared<LaserscanMerger>());

	rclcpp::shutdown();

	return 0;
}
