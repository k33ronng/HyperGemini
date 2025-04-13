#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

class ParkingSpotDetector {
public:
    ParkingSpotDetector(ros::NodeHandle& nh) : it_(nh) {
        sub_ = nh.subscribe("/velodyne_points", 1, &ParkingSpotDetector::pointCloudCallback, this);
        image_pub_ = it_.advertise("/parking_spot/image", 1);
    }

private:
    ros::Subscriber sub_;
    image_transport::ImageTransport it_;
    image_transport::Publisher image_pub_;

    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        // Downsample
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(cloud);
        vg.setLeafSize(0.2f, 0.2f, 0.2f);
        vg.filter(*filtered);

        // Remove ground
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.2);
        seg.setInputCloud(filtered);
        seg.segment(*inliers, *coeff);

        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(filtered);
        extract.setIndices(inliers);
        extract.setNegative(true); // remove ground
        pcl::PointCloud<pcl::PointXYZ>::Ptr no_ground(new pcl::PointCloud<pcl::PointXYZ>);
        extract.filter(*no_ground);

        // Create 2D occupancy image
        const int map_size = 500;
        const float resolution = 0.1f;  // meters/pixel
        cv::Mat bird_eye = cv::Mat::zeros(map_size, map_size, CV_8UC1);

        for (const auto& pt : no_ground->points) {
            int x = static_cast<int>(pt.x / resolution) + map_size / 2;
            int y = static_cast<int>(pt.y / resolution) + map_size / 2;
            if (x >= 0 && x < map_size && y >= 0 && y < map_size) {
                bird_eye.at<uchar>(map_size - y - 1, x) = 255;
            }
        }

        for (uint x = map_size / 2 - 2; x <= map_size / 2 + 2; x++) {
            for (uint y = map_size / 2 - 2; y <= map_size / 2 + 2; y++) {
                bird_eye.at<uchar>(x, y) = 255;
            }
        }
        

        // Morphology + Contour Detection
        cv::Mat morph;
        cv::dilate(bird_eye, morph, cv::Mat(), cv::Point(-1,-1), 2);
        cv::erode(morph, morph, cv::Mat(), cv::Point(-1,-1), 2);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(255 - morph, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for (const auto& contour : contours) {
            cv::RotatedRect rect = cv::minAreaRect(contour);
            if (rect.size.width > 10 && rect.size.height > 5) {
                cv::Point2f box[4];
                rect.points(box);
                for (int i = 0; i < 4; i++) {
                    cv::line(bird_eye, box[i], box[(i+1)%4], 128, 2);
                }
                ROS_INFO("Potential parking spot at center: (%f, %f)", rect.center.x, rect.center.y);
            }
        }

        // Debug view (optional): save image to file
        // cv::imwrite("/tmp/bird_eye_view.png", bird_eye);

        std_msgs::Header header;
        header.stamp = ros::Time::now();
        header.frame_id = "map";  // Or your preferred fixed frame

        sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(header, "mono8", bird_eye).toImageMsg();
        image_pub_.publish(image_msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "parking_spot_detector");
    ros::NodeHandle nh;
    ParkingSpotDetector detector(nh);
    ros::spin();
    return 0;
}
