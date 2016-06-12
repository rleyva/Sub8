#include <stdio.h>
#include <iostream>

#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/nonfree.hpp"

#include <boost/bind.hpp>


class Sub8FeatureDetector
{
    private:
        // OpenCV Handlers
        cv::Mat final_img;
        cv::Mat received_img, template_img;
        cv::Mat received_desc, template_desc;

        cv::vector<cv::KeyPoint> received_kp, template_kp;
        cv::vector<cv::vector<cv::DMatch> > matches;
        cv::vector<cv::DMatch> filtered_matches;

        cv::Ptr<cv::FeatureDetector> feature_detector;
        cv::Ptr<cv::DescriptorExtractor> feature_extractor;
        cv::Ptr<cv::DescriptorMatcher> feature_matcher;

    public:
        Sub8FeatureDetector(cv::Mat template_image);
        void observe_image(const sensor_msgs::ImageConstPtr& msg);
        void filter_keypoints();
        void generate_final_img();
        void generate_homography();
        void run_pipeline(const sensor_msgs::ImageConstPtr& msg);
};

Sub8FeatureDetector::Sub8FeatureDetector(cv::Mat template_image)
{
    this->feature_detector = new cv::DynamicAdaptedFeatureDetector(new cv::StarAdjuster(30.0), 600, 3000, 10);
    this->feature_extractor = new cv::SiftDescriptorExtractor;
    this->feature_matcher = cv::DescriptorMatcher::create("FlannBased");

    this->template_img = template_image;
    this->feature_detector->detect(this->template_img, this->template_kp);
    this->feature_extractor->compute(this->template_img, this->template_kp, this->template_desc);

    cv::imshow("Template", this->template_img);
    cv::waitKey(0);
}

void Sub8FeatureDetector::observe_image(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr tmp = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    this->received_img = tmp->image;

    // TODO: Add some filtering...

    this->feature_detector->detect(this->received_img, this->received_kp);
    this->feature_extractor->compute(this->received_img, this->received_kp, this->received_desc);
    this->feature_matcher->knnMatch(this->template_desc, this->received_desc, this->matches, 500);
}

void Sub8FeatureDetector::filter_keypoints()
{
    this->filtered_matches.reserve(this->matches.size());
    double treshold_dist = 0.25 * sqrt(double(this->received_img.size().height * this->received_img.size().height +
                                              this->received_img.size().width * this->received_img.size().width));
    for (size_t i = 0; i < this->matches.size(); ++i)
    {
        for (int j = 0; j < this->matches[i].size(); j++)
        {
            cv::Point2f from = this->template_kp[this->matches[i][j].queryIdx].pt;
            cv::Point2f to = this->received_kp[this->matches[i][j].trainIdx].pt;

            //Calculate local distance for each possible match
            double dist = sqrt((from.x - to.x) * (from.x - to.x) + (from.y - to.y) * (from.y - to.y));

            //Save as best match if local distance is in specified area and on same height
            if (dist < treshold_dist && abs(from.y-to.y)<5)
            {
                this->filtered_matches.push_back(this->matches[i][j]);
                j = this->matches[i].size();
            }
        }
    }
}

void Sub8FeatureDetector::generate_final_img()
{
    cv::drawMatches(this->template_img, this->template_kp, this->received_img, this->received_kp,
                    this->filtered_matches, this->final_img);
}

void Sub8FeatureDetector::run_pipeline(const sensor_msgs::ImageConstPtr& msg)
{
    this->observe_image(msg);
    this->filter_keypoints();
    this->generate_final_img();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Feature_Detector");
    ros::NodeHandle nh;

    cv::Mat template_img;
    cv::Mat image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
    cv::cvtColor(image, template_img, CV_BGR2GRAY);

    if(!image.data)
    {
        std::cout <<  "Could not open or find the image" << std::endl ;
        return -1;
    }

    Sub8FeatureDetector sift_handler(template_img);
    ros::Subscriber img_sub = nh.subscribe<sensor_msgs::Image>("/down/left/image_raw", 1,
                                                               boost::bind(&Sub8FeatureDetector::run_pipeline,
                                                                           &sift_handler, _1));
    ros::spin();
    return 0;
}