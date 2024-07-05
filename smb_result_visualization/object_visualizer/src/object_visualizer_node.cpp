#include "ros/ros.h"
#include "visualization_msgs/MarkerArray.h"
#include <fstream>
#include <sstream>
#include <string>

struct ObjectData {
    std::string class_id;
    double confidence;
    double x;
    double y;
    double z;
};

std::vector<ObjectData> readCSV(const std::string& filename) {
    // std::cout << "executing?" << std::endl;
    std::vector<ObjectData> objects;
    std::ifstream file(filename);
    std::string line, class_id;
    double confidence, x, y, z;

    // Skip the header line
    std::getline(file, line);

    while (std::getline(file, line)) {
        // std::cout << "in the loop?" << std::endl;
        std::stringstream ss(line);
        std::getline(ss, class_id, ',');
        // std::cout << "class_id: " << class_id << std::endl;
        ss >> confidence;
        // std::cout << "confidence: " << confidence << std::endl;
        ss.ignore();
        ss >> x;
        // std::cout << "x: " << x << std::endl;
        ss.ignore();
        ss >> y;
        // std::cout << "y: " << y << std::endl;
        ss.ignore();
        ss >> z;
        // std::cout << "z: " << z << std::endl;
        objects.push_back({class_id, confidence, x, y, z});
    }
    return objects;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "object_visualizer");
    ros::NodeHandle nh;
    ros::NodeHandle nh_("~");

    ros::Rate loop_rate(1);

    std::string object_file_path;
    std::string map_frame;
    double text_scale;
    double marker_scale;

    nh_.getParam("object_file_path", object_file_path);
    nh_.getParam("map_frame", map_frame);
    nh_.getParam("text_scale", text_scale);
    nh_.getParam("marker_scale", marker_scale);

    ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("object_markers", 1);


    // std::cout << "here?" << std::endl;

    if (object_file_path.empty())
    {
        ROS_ERROR("No object file path specified. Please set the parameter 'object_file_path'.");
        return -1;
    }

    std::vector<ObjectData> objects = readCSV(object_file_path);

    while (ros::ok()) {
        visualization_msgs::MarkerArray marker_array;

        for (size_t i = 0; i < objects.size(); ++i) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = map_frame;
            marker.header.stamp = ros::Time::now();
            marker.ns = "objects";
            marker.id = i;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = objects[i].x;
            marker.pose.position.y = objects[i].y;
            marker.pose.position.z = objects[i].z;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = marker_scale; // Adjust the size as needed
            marker.scale.y = marker_scale;
            marker.scale.z = marker_scale;
            marker.color.a = objects[i].confidence; // Alpha value set to confidence
            // marker.color.a = 1;

            std::string text = objects[i].class_id;

            if(text == "stop sign"){ //red
                marker.color.r = 1.0;
                marker.color.g = 0.0;
                marker.color.b = 0.0;
            }else if(text == "bottle"){ //blue
                marker.color.r = 0.0;
                marker.color.g = 0.0;
                marker.color.b = 1.0;
            }else if(text == "backpack"){ //green
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
            }else if(text == "umbrella"){ //purple?
                marker.color.r = 0.5;
                marker.color.g = 0.0;
                marker.color.b = 0.5;
            }else if(text == "clock"){ //cyan?
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 1.0;
            }
            // std::cout << "marker.color.r: " << marker.color.r << std::endl;
            // std::cout << "marker.color.g: " << marker.color.g << std::endl;
            // std::cout << "marker.color.b: " << marker.color.b <<std::endl;

            marker_array.markers.push_back(marker);

            // Create a text marker for the class name
            visualization_msgs::Marker text_marker;
            text_marker.header.frame_id = map_frame;
            text_marker.header.stamp = ros::Time::now();
            text_marker.ns = "object_names";
            text_marker.id = i + objects.size(); // Ensure unique ID for text marker
            text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::Marker::ADD;
            text_marker.pose.position.x = objects[i].x;
            text_marker.pose.position.y = objects[i].y;
            text_marker.pose.position.z = objects[i].z + 0.5; // Slightly above the marker
            text_marker.scale.z = text_scale; // Text height
            text_marker.text = objects[i].class_id;
            text_marker.color.a = 1.0;

            if(text == "stop sign"){ //red
                text_marker.color.r = 1.0;
                text_marker.color.g = 0.0;
                text_marker.color.b = 0.0;
            }else if(text == "bottle"){ //blue
                text_marker.color.r = 0.0;
                text_marker.color.g = 0.0;
                text_marker.color.b = 1.0;
            }else if(text == "backpack"){ //green
                text_marker.color.r = 0.0;
                text_marker.color.g = 1.0;
                text_marker.color.b = 0.0;
            }else if(text == "umbrella"){ //purple?
                text_marker.color.r = 0.5;
                text_marker.color.g = 0.0;
                text_marker.color.b = 0.5;
            }else if(text == "clock"){ //cyan?
                text_marker.color.r = 0.0;
                text_marker.color.g = 1.0;
                text_marker.color.b = 1.0;
            }

            marker_array.markers.push_back(text_marker);
        }

        marker_pub.publish(marker_array);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
