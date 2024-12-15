#include "MonoNode.h"
#include <opencv2/imgproc/imgproc.hpp> // For Hough Lines and image processing

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc > 1) {
        ROS_WARN ("Arguments supplied via command line are neglected.");
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ros::NodeHandle node_handle;
    image_transport::ImageTransport image_transport (node_handle);

    MonoNode node (ORB_SLAM2::System::MONOCULAR, node_handle, image_transport);

    node.Init();

    ros::spin();

    ros::shutdown();

    return 0;
}

MonoNode::MonoNode (ORB_SLAM2::System::eSensor sensor, ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport) : Node (sensor, node_handle, image_transport) {
    // Parameterizing image topics and line detection parameters
    std::string image_topic, camera_info_topic;
    node_handle.param<std::string>("image_topic", image_topic, "/camera/image");
    node_handle.param<std::string>("camera_info_topic", camera_info_topic, "/camera/camera_info");

    node_handle.param<int>("canny_thresh1", canny_thresh1_, 50);
    node_handle.param<int>("canny_thresh2", canny_thresh2_, 150);
    node_handle.param<int>("hough_thresh", hough_thresh_, 50);

    image_subscriber = image_transport.subscribe(image_topic, 1, &MonoNode::ImageCallback, this);
    camera_info_topic_ = camera_info_topic;
}

MonoNode::~MonoNode () {
}

void MonoNode::ImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImageConstPtr cv_in_ptr;
    try {
        cv_in_ptr = cv_bridge::toCvShare(msg);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Original Image
    cv::Mat original_image = cv_in_ptr->image;

    // Step 1: Convert to grayscale (if not already)
    cv::Mat gray_image;
    if (original_image.channels() == 3) {
        cv::cvtColor(original_image, gray_image, cv::COLOR_BGR2GRAY);
    } else {
        gray_image = original_image.clone();
    }

    // Step 2: Apply Canny edge detection (can be tuned)
    cv::Mat edges;
    cv::Canny(gray_image, edges, canny_thresh1_, canny_thresh2_); // Adjust thresholds as necessary

    // Step 3: Use Hough Line Transform to detect lines
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(edges, lines, 1, CV_PI/180, hough_thresh_, 50, 10); // Tune parameters for better results

    // Step 4: Draw the detected lines on the image (for visualization purposes)
    for (size_t i = 0; i < lines.size(); i++) {
        cv::line(original_image, 
                 cv::Point(lines[i][0], lines[i][1]), 
                 cv::Point(lines[i][2], lines[i][3]), 
                 cv::Scalar(0, 255, 0), 2, cv::LINE_AA); // Draw green lines
    }

    // Optional: Visualize the line-detected image (uncomment for debugging)
    // cv::imshow("Line Detection", original_image);
    // cv::waitKey(1); // Wait for 1 ms to process visualization

    current_frame_time_ = msg->header.stamp;

    // Step 5: Send the original image to ORB-SLAM2 (even though it's now pre-processed)
    orb_slam_->TrackMonocular(original_image, cv_in_ptr->header.stamp.toSec());

    Update();
}

void MonoNode::SaveMap(const std::string& filename)
{
    if (orb_slam_)
    {
        ROS_INFO("Saving map to %s", filename.c_str());
        orb_slam_->SaveMap(filename); // Save the current map
    }
}
