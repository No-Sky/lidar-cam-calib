#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <pcl_conversions/pcl_conversions.h>
#include <time.h>

// #include "CustomMsg.h"
#include "common.h"

using namespace std;

struct pointData{
    float x;
    float y;
    float z;
    int i;
};
vector<pointData> vector_data;
sensor_msgs::PointCloud2 point_cloud;
pcl::PointCloud<pcl::PointXYZI> cloud;
string input_bag_path, output_path;
int threshold_lidar, data_num;

void loadAndSavePointcloud(int index);
void writeTitle(const string filename, unsigned long point_num);
void writePointCloud(const string filename, const vector<pointData> singlePCD);
void dataSave(int index);

void loadAndSavePointcloud(int index) {
    string path = input_bag_path + int2str(index) + ".bag";
    fstream file_;
    file_.open(path, ios::in);
    if (!file_) {
        cout << "File " << path << " does not exit" << endl;
        return;
    }
    ROS_INFO("Start to load the rosbag %s", path.c_str());
    rosbag::Bag bag;
    try {
        bag.open(path, rosbag::bagmode::Read);
    } catch (rosbag::BagException e) {
        ROS_ERROR_STREAM("LOADING BAG FAILED: " << e.what());
        return;
    }

    vector<string> types;
    types.push_back(string("/rslidar_mid_points")); 
    rosbag::View view(bag, rosbag::TypeQuery(types));

    int cloudCount = 0;
    for (const rosbag::MessageInstance& m : view) {
        std::string topic   = m.getTopic();
        std::cout<<"topic:"<<topic<<std::endl;

        point_cloud = *(m.instantiate<sensor_msgs::PointCloud2>()); // message type

        // sensor_msgs::PointCloud2::ConstPtr input = m.instantiate<sensor_msgs::PointCloud2>();
        // cloud (new pcl::PointCloud<pcl::PointXYZI>);

        pcl::fromROSMsg(point_cloud, cloud);

        for(uint i = 0; i < cloud.size(); ++i) {
            pointData myPoint;
            myPoint.x = cloud.points[i].x;
            myPoint.y = cloud.points[i].y;
            myPoint.z = cloud.points[i].z;
            myPoint.i = cloud.points[i].intensity;

            vector_data.push_back(myPoint);
        }
        ++cloudCount;
        if (cloudCount >= threshold_lidar) {
            break;
        }
    }
    dataSave(index);
    vector_data.clear();
}

void writeTitle(const string filename, unsigned long point_num) {
    ofstream outfile(filename.c_str(), ios_base::app);
    if (!outfile) {
        cout << "Can not open the file: " << filename << endl;
        exit(0);
    }
    else {
        outfile << "# .PCD v.7 - Point Cloud Data file format" << endl;
        outfile << "VERSION .7" << endl;
        outfile << "FIELDS x y z intensity" << endl;
        outfile << "SIZE 4 4 4 4" << endl;
        outfile << "TYPE F F F F" << endl;
        outfile << "COUNT 1 1 1 1" << endl;
        outfile << "WIDTH " << long2str(point_num) << endl;
        outfile << "HEIGHT 1" << endl;
        outfile << "VIEWPOINT 0 0 0 1 0 0 0" << endl;
        outfile << "POINTS " << long2str(point_num) << endl;
        outfile << "DATA ascii" << endl;
    }
    ROS_INFO("Save file %s", filename.c_str());
}

void writePointCloud(const string filename, const vector<pointData> singlePCD) {
    ofstream outfile(filename.c_str(), ios_base::app);
    if (!outfile) {
        cout << "Can not open the file: " << filename << endl;
        exit(0);
    }
    else {
        for (unsigned long i = 0; i < singlePCD.size(); ++i) {
            outfile << float2str(singlePCD[i].x) << " " << float2str(singlePCD[i].y) << " " << float2str(singlePCD[i].z) << " " << int2str(singlePCD[i].i) << endl;
        }
    }
}

void dataSave(int index) {
    string outputName = output_path + int2str(index) + ".pcd";
    writeTitle(outputName, vector_data.size());
    writePointCloud(outputName, vector_data);
}

void getParameters() {
    cout << "Get the parameters from the launch file" << endl;

    if (!ros::param::get("input_bag_path", input_bag_path)) {
        cout << "Can not get the value of input_bag_path" << endl;
        exit(1);
    }
    else {
        cout << input_bag_path << endl;
    }
    if (!ros::param::get("output_pcd_path", output_path)) {
        cout << "Can not get the value of output_path" << endl;
        exit(1);
    }
    if (!ros::param::get("threshold_lidar", threshold_lidar)) {
        cout << "Can not get the value of threshold_lidar" << endl;
        exit(1);
    }
    if (!ros::param::get("data_num", data_num)) {
        cout << "Can not get the value of data_num" << endl;
        exit(1);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pcdTransfer");
    getParameters();

    for (int i = 1; i <= data_num; ++i) {
        loadAndSavePointcloud(i);
    }
    ROS_INFO("Finish all!");
    return 0;
}

