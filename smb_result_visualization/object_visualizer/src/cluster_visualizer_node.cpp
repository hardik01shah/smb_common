#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <fstream>
#include <sstream>
#include <vector>

struct ObjectData {
    std::string class_id;
    double confidence;
    double x, y, z;
};

std::vector<ObjectData> readCSV(const std::string& filename) {
    std::vector<ObjectData> objects;
    std::ifstream file(filename);
    std::string line, class_id;
    double confidence, x, y, z;

    // Skip the header line
    std::getline(file, line);

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::getline(ss, class_id, ',');
        ss >> x;
        std::cout << "x: " << x << std::endl;
        ss.ignore();
        ss >> y;
        std::cout << "y: " << y << std::endl;
        ss.ignore();
        ss >> z;
        std::cout << "z: " << z << std::endl;
        objects.push_back({class_id, confidence, x, y, z});
    }
    return objects;
}

void createBoundingBoxMarker(const ObjectData& obj, double length, double width, double height, int id, visualization_msgs::Marker& marker) {
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "bounding_boxes";
    marker.id = id;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    
    marker.scale.x = 0.1; // Line width
    
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;

    double x = obj.x;
    double y = obj.y;
    double z = obj.z;

    std::vector<geometry_msgs::Point> points(8);
    points[0].x = x - length / 2; points[0].y = y - width / 2; points[0].z = z - height / 2;
    points[1].x = x + length / 2; points[1].y = y - width / 2; points[1].z = z - height / 2;
    points[2].x = x + length / 2; points[2].y = y + width / 2; points[2].z = z - height / 2;
    points[3].x = x - length / 2; points[3].y = y + width / 2; points[3].z = z - height / 2;
    points[4].x = x - length / 2; points[4].y = y - width / 2; points[4].z = z + height / 2;
    points[5].x = x + length / 2; points[5].y = y - width / 2; points[5].z = z + height / 2;
    points[6].x = x + length / 2; points[6].y = y + width / 2; points[6].z = z + height / 2;
    points[7].x = x - length / 2; points[7].y = y + width / 2; points[7].z = z + height / 2;

    // Bottom square
    marker.points.push_back(points[0]);
    marker.points.push_back(points[1]);
    
    marker.points.push_back(points[1]);
    marker.points.push_back(points[2]);

    marker.points.push_back(points[2]);
    marker.points.push_back(points[3]);

    marker.points.push_back(points[3]);
    marker.points.push_back(points[0]);

    // Top square
    marker.points.push_back(points[4]);
    marker.points.push_back(points[5]);

    marker.points.push_back(points[5]);
    marker.points.push_back(points[6]);

    marker.points.push_back(points[6]);
    marker.points.push_back(points[7]);

    marker.points.push_back(points[7]);
    marker.points.push_back(points[4]);

    // Vertical lines
    marker.points.push_back(points[0]);
    marker.points.push_back(points[4]);

    marker.points.push_back(points[1]);
    marker.points.push_back(points[5]);

    marker.points.push_back(points[2]);
    marker.points.push_back(points[6]);

    marker.points.push_back(points[3]);
    marker.points.push_back(points[7]);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "bounding_box_publisher");
    ros::NodeHandle nh;
    ros::NodeHandle nh_;

    std::string cluster_file_path="/home/scarlett/Workspaces/smb_ws/src/object_visualizer/data/cluster_centers.csv";
    std::string map_frame="map";

    // ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("cluster_markers", 1);
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("cluster_markers", 1);



    nh_.getParam("cluster_file_path", cluster_file_path);
    nh_.getParam("map_frame", map_frame);

    std::cout << "cluster_file_path: " << cluster_file_path << std::endl;
    if (cluster_file_path.empty())
    {
        ROS_ERROR("No cluster file path specified. Please set the parameter 'cluster_file_path'.");
        return -1;
    }

    std::vector<ObjectData> objects = readCSV(cluster_file_path);

    ros::Rate r(1);
    while (ros::ok()) {
        visualization_msgs::MarkerArray marker_array;
        int id = 0;
        for (const auto& obj : objects) {
            visualization_msgs::Marker marker;
            createBoundingBoxMarker(obj, 1.0, 1.0, 1.0, id++, marker); // Set the dimensions as needed
            marker_array.markers.push_back(marker);
        }
        marker_pub.publish(marker_array);
        r.sleep();
    }

    return 0;
}
