#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <fstream>
#include <sstream>
#include <cmath>
#include <vector>
#include <string>
#include <algorithm>

struct ObjectData {
    std::string class_id;
    double confidence;
    double x, y, z;
};

class ObjectClustering {
public:
    ObjectClustering(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
        : nh_(nh), private_nh_(private_nh){
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("cluster_cubes", 1);
        private_nh_.getParam("input_csv", input_csv_);
        private_nh_.getParam("output_csv", output_csv_);
        private_nh_.getParam("cluster_distance_threshold",cluster_distance_threshold_);
        private_nh_.getParam("marker_scale",marker_scale_);
        objects_ = readCSV(input_csv_);
        clusterObjects();
        publishMarkers();
    }

private:
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
            ss >> confidence;
            ss.ignore();
            ss >> x;
            ss.ignore();
            ss >> y;
            ss.ignore();
            ss >> z;
            objects.push_back({class_id, confidence, x, y, z});
        }
        return objects;
    }

    void clusterObjects() {
        // const double cluster_distance_threshold = 1.0; // Adjust as needed
        std::vector<bool> object_used(objects_.size(), false);
        std::vector<ObjectData> clustered_objects;

        for (size_t i = 0; i < objects_.size(); ++i) {
            if (object_used[i])
                continue;

            std::vector<ObjectData> cluster;
            double total_confidence = 0.0;

            // Start a new cluster with the current object
            cluster.push_back(objects_[i]);
            total_confidence += objects_[i].confidence;
            object_used[i] = true;

            for (size_t j = i + 1; j < objects_.size(); ++j) {
                if (object_used[j])
                    continue;

                double distance = std::sqrt(std::pow(objects_[j].x - objects_[i].x, 2) +
                                            std::pow(objects_[j].y - objects_[i].y, 2) +
                                            std::pow(objects_[j].z - objects_[i].z, 2));

                if (distance <= cluster_distance_threshold_) {
                    cluster.push_back(objects_[j]);
                    total_confidence += objects_[j].confidence;
                    object_used[j] = true;
                }
            }

            // Calculate the weighted centroid of the cluster
            double weighted_x = 0.0, weighted_y = 0.0, weighted_z = 0.0;
            for (const auto& obj : cluster) {
                weighted_x += obj.x * obj.confidence;
                weighted_y += obj.y * obj.confidence;
                weighted_z += obj.z * obj.confidence;
            }
            ObjectData centroid = cluster[0];
            centroid.x = weighted_x / total_confidence;
            centroid.y = weighted_y / total_confidence;
            centroid.z = weighted_z / total_confidence;
            clustered_objects.push_back(centroid);
        }

        objects_ = clustered_objects;
        writeCSV(output_csv_);
    }

    void writeCSV(const std::string& filename) {
        std::ofstream file(filename);
        file << "class,x,y,z\n";
        for (const auto& obj : objects_) {
            file << obj.class_id  << ","
                 << obj.x << "," << obj.y << "," << obj.z << "\n";
        }
    }

    void publishMarkers() {
        ros::Rate rate(1);
        while (ros::ok()) {
            visualization_msgs::MarkerArray marker_array;
            int id = 0;
            for (const auto& obj : objects_) {
                std::string text = obj.class_id;
                visualization_msgs::Marker marker;
                marker.header.frame_id = "map";
                marker.header.stamp = ros::Time::now();
                marker.ns = "object_markers";
                marker.id = id++;
                marker.type = visualization_msgs::Marker::CUBE;
                marker.action = visualization_msgs::Marker::ADD;
                marker.pose.position.x = obj.x;
                marker.pose.position.y = obj.y;
                marker.pose.position.z = obj.z;
                marker.scale.x = marker_scale_; // Adjust as needed
                marker.scale.y = marker_scale_; // Adjust as needed
                marker.scale.z = marker_scale_; // Adjust as needed
                marker.color.a = obj.confidence; // Full opacity
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
                marker.text = text;
                
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

                visualization_msgs::Marker text_marker = marker;
                text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                text_marker.id = marker.id + objects_.size(); // Ensure unique ID for text marker
                text_marker.pose.position.z -= marker_scale_; // Offset text above the object
                text_marker.scale.z = 0.3; // Text height
                text_marker.color.r = 1.0;
                text_marker.color.g = 1.0;
                text_marker.color.b = 1.0;
                text_marker.color.a = 1.0;
                text_marker.text = text + ": " + "(" + std::to_string(obj.x) + ", "
                                                     + std::to_string(obj.y) + ", "
                                                     + std::to_string(obj.z) + ")";
                marker_array.markers.push_back(marker);
                marker_array.markers.push_back(text_marker);
            }
            marker_pub_.publish(marker_array);
            rate.sleep();
        }
    }

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Publisher marker_pub_;
    std::vector<ObjectData> objects_;
    std::string input_csv_; // Replace with your CSV file path
    std::string output_csv_; // Replace with your output CSV file path
    double cluster_distance_threshold_;
    double marker_scale_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "object_clustering_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // std::string input_csv = "/home/scarlett/Workspaces/smb_ws/src/object_visualizer/data/object_detection.csv";  // Replace with your CSV file path
    // std::string output_csv = "/home/scarlett/Workspaces/smb_ws/src/object_visualizer/data/clustered.csv"; // Replace with your output CSV file path

    ObjectClustering object_clustering(nh, private_nh);

    ros::spin();
    return 0;
}