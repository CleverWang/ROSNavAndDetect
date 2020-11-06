#include <ros/ros.h>
#include <iostream>
#include <cstdlib>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

static std::vector<move_base_msgs::MoveBaseGoal> arWayPoint;
static ros::Publisher marker_pub;
static visualization_msgs::Marker marker_waypoints;
static visualization_msgs::Marker text_marker;
static ros::Subscriber rgb_sub;
static ros::Subscriber depth_sub;

int nWPIndex = 0;

#define CNT_WAYPOINTS 3
#define DIM_WAYPOINT 6
#define IMAGE_PATH "/home/robot/CapturedImages"
#define RGB_SLEEP_DURATION 1
#define DEPTH_SLEEP_DURATION 10

static double waypoints[CNT_WAYPOINTS][DIM_WAYPOINT] = {
    // x,y,z,r,p,y
    0, 0, 0, 0, 0, 0,
    0.0, 1.0, 0, 0, 0, 0,
    0.0, 2.0, 0, 0, 0, 0,
};

static const std::string WIN_TITLE = "nav";
static const char rgbFlag[] = "rgb";
static const char depthFlag[] = "depth";
static const char poseFlag[] = "pose";
static bool isReadyRGB = false;
static bool isReadyDepth = false;
static bool isDoneRGB = false;
static bool isDoneDepth = false;
static char filename[128];

void Init_WayPoints()
{
    move_base_msgs::MoveBaseGoal newWayPoint;
    tf::Quaternion q;

    newWayPoint.target_pose.header.frame_id = "map";
    //for (int i = 0; i < 2; i++)
    for (int i = 0; i < CNT_WAYPOINTS; i++)
    {
        newWayPoint.target_pose.pose.position.x = waypoints[i][0];
        newWayPoint.target_pose.pose.position.y = waypoints[i][1];

        q.setRPY(waypoints[i][3], waypoints[i][4], waypoints[i][5]);
        newWayPoint.target_pose.pose.orientation.x = q.x();
        newWayPoint.target_pose.pose.orientation.y = q.y();
        newWayPoint.target_pose.pose.orientation.z = q.z();
        newWayPoint.target_pose.pose.orientation.w = q.w();

        arWayPoint.push_back(newWayPoint);
    }
}

void Init_Marker()
{
    marker_waypoints.header.frame_id = "map";
    marker_waypoints.ns = "marker_waypoints";
    marker_waypoints.action = visualization_msgs::Marker::ADD;
    marker_waypoints.id = 1;
    marker_waypoints.type = visualization_msgs::Marker::CUBE_LIST;
    marker_waypoints.scale.x = 0.2;
    marker_waypoints.scale.y = 0.2;
    marker_waypoints.scale.z = 0.3;
    marker_waypoints.color.r = 0;
    marker_waypoints.color.g = 0.5;
    marker_waypoints.color.b = 1.0;
    marker_waypoints.color.a = 1.0;

    geometry_msgs::Point point;
    point.z = 0.15;
    int nNumWP = arWayPoint.size();
    for (int i = 0; i < nNumWP; i++)
    {
        point.x = arWayPoint[i].target_pose.pose.position.x;
        point.y = arWayPoint[i].target_pose.pose.position.y;
        marker_waypoints.points.push_back(point);
    }
}

void DrawTextMarker(std::string inText, int inID, float inScale, float inX, float inY, float inZ, float inR, float inG, float inB)
{
    text_marker.header.frame_id = "map";
    text_marker.ns = "line_obj";
    text_marker.action = visualization_msgs::Marker::ADD;
    text_marker.id = inID;
    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.scale.z = inScale;
    text_marker.color.r = inR;
    text_marker.color.g = inG;
    text_marker.color.b = inB;
    text_marker.color.a = 1.0;

    text_marker.pose.position.x = inX;
    text_marker.pose.position.y = inY;
    text_marker.pose.position.z = inZ;

    text_marker.pose.orientation = tf::createQuaternionMsgFromYaw(1.0);

    text_marker.text = inText;

    marker_pub.publish(text_marker);
}

void PublishWaypointsMarker()
{
    int nNumWP = arWayPoint.size();
    for (int i = 0; i < nNumWP; i++)
    {
        float wp_x = arWayPoint[i].target_pose.pose.position.x;
        float wp_y = arWayPoint[i].target_pose.pose.position.y;

        std::ostringstream stringStream;
        stringStream << "nav_" << i;
        std::string id = stringStream.str();
        DrawTextMarker(id, i, 0.2, wp_x, wp_y, marker_waypoints.scale.z + 0.2, 0, 0.5, 1.0);
    }
    marker_pub.publish(marker_waypoints);
    ros::spinOnce();
}

void callbackRGB(const sensor_msgs::ImageConstPtr &msg)
{
    if (isReadyRGB == true)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        sprintf(filename, "%s/%s_%d.jpg", IMAGE_PATH, rgbFlag, nWPIndex);
        //if (access(filename, R_OK) != 0)
        //{
        //imshow(WIN_TITLE, cv_ptr->image);
        //cvWaitKey(2000);
        imwrite(filename, cv_ptr->image);
        ROS_WARN("captured %s", filename);
        //}

        isDoneRGB = true;
        isReadyRGB = false;
    }
}

void callbackDepth(const sensor_msgs::ImageConstPtr &msg)
{
    // ROS_INFO("callbackDepth");
    if (isReadyDepth == true)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        sprintf(filename, "%s/%s_%d.jpg", IMAGE_PATH, depthFlag, nWPIndex);
        //if (access(filename, R_OK) != 0)
        //{
        imwrite(filename, cv_ptr->image);
        ROS_WARN("captured %s", filename);
        //}

        isDoneDepth = true;
        isReadyDepth = false;
    }
}

bool getTFTransform(tf::StampedTransform &transform, std::string parent, std::string child)
{
    tf::TransformListener listener;
    try
    {
        bool res = listener.waitForTransform(parent, child, ros::Time(0), ros::Duration(3.0));
        if (!res)
        {
            return res;
        }
        listener.lookupTransform(parent, child, ros::Time(0), transform);
        ROS_INFO("Quaternion(x=%f, y=%f, z=%f, w=%f)", transform.getRotation()[0], transform.getRotation()[1],
                 transform.getRotation()[2], transform.getRotation()[3]);
        ROS_INFO("Vector(x=%f, y=%f, z=%f)", transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
        return true;
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
    }
    return false;
}

void saveTFTransform()
{
    tf::StampedTransform map2footTrans;
    getTFTransform(map2footTrans, "map", "base_footprint");
    sprintf(filename, "%s/%s_%d.txt", IMAGE_PATH, poseFlag, nWPIndex);
    std::ofstream fout(filename, std::ios::out);
    fout << map2footTrans.getOrigin().x() << " " << map2footTrans.getOrigin().y() << " " << map2footTrans.getOrigin().z();
    for (int i = 0; i < 4; ++i)
    {
        fout << " " << map2footTrans.getRotation()[i];
    }
    fout.close();
}

void captureRGBAndDepth()
{
    // ROS_INFO("captureRGBAndDepth");

    // ros::Duration(RGB_SLEEP_DURATION).sleep();
    isReadyRGB = true;
    while (isDoneRGB == false)
    {
        ros::spinOnce();
    }
    isDoneRGB = false;

    // ros::Duration(DEPTH_SLEEP_DURATION).sleep();
    isReadyDepth = true;
    while (isDoneDepth == false)
    {
        ros::spinOnce();
    }
    isDoneDepth = false;

    saveTFTransform();
}

int main(int argc, char **argv)
{
    cv::namedWindow(WIN_TITLE);
    ros::init(argc, argv, "buaa_demos_nav");

    ros::NodeHandle nh;
    marker_pub = nh.advertise<visualization_msgs::Marker>("waypoints_marker", 100);
    rgb_sub = nh.subscribe("/kinect2/hd/image_color", 1, callbackRGB);
    depth_sub = nh.subscribe("/kinect2/hd/image_depth_rect", 1, callbackDepth);

    // ros::AsyncSpinner spinner(3);
    // spinner.start();

    Init_WayPoints();
    Init_Marker();

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);
    //wait for the action server to come up
    while (!ac.waitForServer(ros::Duration(5.0)))
    {
        if (!ros::ok())
            break;
        ROS_INFO("Waiting for the move_base action server to come up");

        PublishWaypointsMarker();
    }

    while (ros::ok())
    {
        PublishWaypointsMarker();
        if (nWPIndex >= arWayPoint.size())
        {
            nWPIndex = 0;
            break;
        }

        ROS_INFO("Go to the WayPoint[%d]", nWPIndex);
        ac.sendGoal(arWayPoint[nWPIndex]);
        ac.waitForResult();
        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("Arrived at WayPoint[%d] !", nWPIndex);

            captureRGBAndDepth();

            nWPIndex++;
        }
        else
            ROS_INFO("Failed to get to WayPoint[%d] ...", nWPIndex);
    }

    cv::destroyWindow(WIN_TITLE);

    // ROS_INFO("detecting...");
    // system("python /home/robot/catkin_ws/src/buaa_demos/src/detect.py");

    // ros::waitForShutdown();
    // ros::shutdown();

    return 0;
}