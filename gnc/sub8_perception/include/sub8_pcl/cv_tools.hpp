#pragma once

#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <limits>

#include <eigen_conversions/eigen_msg.h>
#include <Eigen/StdVector>

#include <boost/foreach.hpp>
#include "boost/format.hpp"

#include <opencv2/opencv.hpp>

#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include "ros/ros.h"

#define VISUALIZE
// #define SEGMENTATION_DEBUG

namespace sub {

typedef std::vector<cv::Point> Contour;

// Compute the centroid of an OpenCV contour (Not templated)
cv::Point contour_centroid(Contour& contour);

// Used as a comparison function for std::sort(), sorting in order of decreasing perimeters
// returns true if the contourArea(c1) > contourArea(c2)
bool larger_contour(const Contour &c1, const Contour &c2);

// Filter a histogram of type cv::MatND generated by cv::calcHist using a gaussian kernel
cv::MatND smooth_histogram(const cv::MatND &histogram, size_t filter_kernel_size = 3, float sigma = 1.0);

// Generate a one-dimensional gaussian kernel given a kernel size and it's standard deviation (sigma)
std::vector<float> generate_gaussian_kernel_1D(size_t kernel_size = 3, float sigma = 1.0);

// Finds positive local maxima greater than (global maximum * thresh_multiplier)
std::vector<cv::Point> find_local_maxima(const cv::MatND &histogram,  float thresh_multiplier);

// Finds negative local minima less than (global minimum * thresh_multiplier)
std::vector<cv::Point> find_local_maxima(const cv::MatND &histogram,  float thresh_multiplier);

// Selects the mode of a multi-modal distribution closest to a given target value
unsigned int select_hist_mode(std::vector<cv::Point> &histogram_modes, unsigned int target);

// Takes in a grayscale image and segments out a semi-homogenous foreground object with
// pixel intensities close to <target>. Tuning of last three parameters may imrove
// results but default values should work well in most cases.
void statistical_image_segmentation(const cv::Mat &src, cv::Mat &dest, 
		const int hist_size, const float** ranges, const int target, std::string image_name,
        const float sigma = 1.5, const float low_thresh_gain = 0.5, const float high_thresh_gain = 0.5);

Eigen::Vector3d triangulate_image_coordinates(const cv::Point &pt1, const cv::Point &pt2, 
											  const Eigen::Matrix3d &fundamental, const Eigen::Matrix3d &R);

struct ImageWithCameraInfo
{	
	/**
		Packages corresponding  sensor_msgs::ImageConstPtr and sensor_msgs::CameraInfoConstPtr info_msg
		into one object. Containers of these objects can be sorted by their image_time attribute
	*/
	public:
		ImageWithCameraInfo(){}
		ImageWithCameraInfo(sensor_msgs::ImageConstPtr _image_msg_ptr, sensor_msgs::CameraInfoConstPtr _info_msg_ptr);
		sensor_msgs::ImageConstPtr image_msg_ptr;
		sensor_msgs::CameraInfoConstPtr info_msg_ptr;
		ros::Time image_time;
		bool operator <(const ImageWithCameraInfo& right) const {
			return this->image_time < right.image_time;
		}
};


class FrameHistory
{
	/**
		Object that subscribes itself to an image topic and stores up to a user defined
		number of ImageWithCameraInfo objects. The frame history can then be retrieved 
		in whole or just a portion.
	*/
public:

	FrameHistory(std::string img_topic, unsigned int hist_size);
	~FrameHistory();
	void image_callback(const sensor_msgs::ImageConstPtr &image_msg, const sensor_msgs::CameraInfoConstPtr &info_msg);
	std::vector<ImageWithCameraInfo> get_frame_history(unsigned int frames_requested);
	int frames_available();

	const std::string topic_name;
	const size_t history_size;

private:
	ros::NodeHandle nh;
	image_transport::CameraSubscriber _image_sub;
	image_transport::ImageTransport _image_transport;
	std::vector<ImageWithCameraInfo> _frame_history_ring_buffer;
	size_t frame_count;

};

} // namespace sub