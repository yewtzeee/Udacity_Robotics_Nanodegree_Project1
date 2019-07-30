#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include "std_msgs/String.h"
#include <sensor_msgs/Image.h>
#include <string>
#include <sstream>

// Define a global client that can request services
ros::ServiceClient client;
std::stringstream ss;
std::stringstream image_height_ss;
std::stringstream image_step_ss;
std::stringstream image_step_index_ss;
std_msgs::String msg;
std_msgs::String msg_image_height;
std_msgs::String msg_image_step;
std_msgs::String msg_image_step_index;
bool printed_image_height_and_step = false;
int image_step_index = 0;
int image_step_segmentation_division = 0;
int image_step_segmentation_remainder = 0;
int image_step_first_segment_upper_limit = 0;
int image_step_third_segment_lower_limit = 0;
bool found_white_pixel = false;



// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    // Call the safe_move service and pass the requested joint angles
    if (!client.call(srv))
    {
        ROS_ERROR("Failed to call service safe_move");
    }
    
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255;

    // http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
    // Convert int to string reference
    // std::stringstream ss;
    // std_msgs::String msg;
    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
    
    if (!printed_image_height_and_step)
    {
        image_height_ss << img.height;
        image_step_ss << img.step;
        msg_image_height.data = image_height_ss.str();
        msg_image_step.data = image_step_ss.str();
        ROS_INFO("Entering process_image_callback with img.height: %s and img.step: %s", msg_image_height.data.c_str(), msg_image_step.data.c_str());
        printed_image_height_and_step = true;
    }
    int total_pixel = img.height * img.step;

    image_step_segmentation_division = img.step / 3;
    image_step_segmentation_remainder = img.step % 3;

    image_step_first_segment_upper_limit = 0;
    image_step_third_segment_lower_limit = 0;
    
    switch (image_step_segmentation_remainder)
    {
        case 0:
            image_step_first_segment_upper_limit = image_step_segmentation_division - 1;
            image_step_third_segment_lower_limit = image_step_segmentation_division * 2;
            break;
        case 1:
            image_step_first_segment_upper_limit = image_step_segmentation_division - 1;
            image_step_third_segment_lower_limit = (image_step_segmentation_division * 2) + 1;
            break;
        case 2:
            image_step_first_segment_upper_limit = image_step_segmentation_division;
            image_step_third_segment_lower_limit = (image_step_segmentation_division * 2) + 1;
            break;
        default:
            ROS_INFO("Entering process_image_callback with");
    }

    found_white_pixel = false;
    for (int i = 0; i < total_pixel; i++)
    {
        if (img.data[i] == 255)
        {
            found_white_pixel = true;
            ss << i;
            msg.data = ss.str();
            
            image_step_index = i % img.step;
            image_step_index_ss << image_step_index;
            msg_image_step_index.data = image_step_index_ss.str();
            ROS_INFO("Found white pixel at %s with img.step: %s", msg.data.c_str(), msg_image_step_index.data.c_str());

            break;
        }
    }
    
    if (found_white_pixel)
    {
        if (image_step_index <= image_step_first_segment_upper_limit)
        {
            drive_robot(0.5, -0.6);
        }
        else if (image_step_index >= image_step_third_segment_lower_limit)
        {
            drive_robot(0.5, 0.6);
        }
        else
        {
            drive_robot(1.0, 0);
        }
    }
    else
    {
        drive_robot(0, 0);
    }
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
