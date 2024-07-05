#include "ros/ros.h"
#include "std_msgs/String.h"
#include "object_detection_msgs/ObjectDetectionInfoArray.h"
#include "object_detection_msgs/ObjectDetectionInfo.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include <tf/transform_listener.h>
#include <fstream>

static const double kThreshold = 1.5; // temp
static const std::string detectionInfoTopic = "object_detector/detection_info";

struct DetectedObject {
    std::string class_id{};
    geometry_msgs::Point position_world{};
};

struct Parameters {
    // Define parameters if necessary
};

class RecordDetection {
public:
    RecordDetection(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh) 
        : nh_(nh), private_nh_(private_nh) {
        detected_objects_sub_ = nh_.subscribe<object_detection_msgs::ObjectDetectionInfoArray>(
            detectionInfoTopic, 10, &RecordDetection::detectionInfoCallback, this);
        current_detected_objects_pub_ = nh_.advertise<object_detection_msgs::ObjectDetectionInfoArray>(
        "current_detected_objects",1);
        current_detected_objects_pose_pub_ = nh_.advertise<geometry_msgs::PoseArray>(
        "pose_array_current_detected_objects",1);
        current_optical_link_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("optical_link_pose_in_world",1);
        timer_ = nh_.createTimer(ros::Duration(1), &RecordDetection::TimedCommandCallback, this,
                                  false, true);
        detected_object_pose_array_.header.frame_id = "world_graph_msf";
        current_detected_object_pub_ = nh_.advertise<geometry_msgs::Pose>("current_detected_object_pose_in_world",1);
    }

    double calDistance(const DetectedObject& it, const geometry_msgs::Point& temp_position_world);
    void detectionInfoCallback(const object_detection_msgs::ObjectDetectionInfoArrayConstPtr& detection_info_array);
    void TimedCommandCallback(const ros::TimerEvent& e);
private:
    // Node handles
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // Subscriber
    ros::Subscriber detected_objects_sub_;

    // Publisher
    ros::Publisher current_detected_objects_pub_;
    ros::Publisher current_detected_objects_pose_pub_;
    ros::Publisher current_optical_link_pose_pub_; // in world frame
    ros::Publisher current_detected_object_pub_;
    // Timer
    ros::Timer timer_;

    // TF listener
    tf::TransformListener listener_;

    // Vector to store detected objects
    std::vector<DetectedObject> detected_object_vector_{};
    object_detection_msgs::ObjectDetectionInfoArray detected_object_array_;
    geometry_msgs::PoseArray detected_object_pose_array_;
    object_detection_msgs::ObjectDetectionInfo detected_object_temp_;
    geometry_msgs::Pose detected_object_pose_temp_;
    geometry_msgs::PoseStamped optical_link_pose_in_world_temp_; 

};


void RecordDetection::TimedCommandCallback(const ros::TimerEvent& e){

}

// euclidean distance
double RecordDetection::calDistance(const DetectedObject& it, const geometry_msgs::Point& temp_position_world) {
    double distance = ((it.position_world.x - temp_position_world.x) * (it.position_world.x - temp_position_world.x) + 
                      (it.position_world.y - temp_position_world.y) * (it.position_world.y - temp_position_world.y) + 
                      (it.position_world.z - temp_position_world.z) * (it.position_world.z - temp_position_world.z));
    return std::sqrt(distance);
}

void RecordDetection::detectionInfoCallback(const object_detection_msgs::ObjectDetectionInfoArrayConstPtr& detection_info_array) {
    // std::cout << "test point A"<<std::endl;
    int object_num = detection_info_array->info.size();
    // std::cout << "info size: " << object_num << std::endl;

    geometry_msgs::Point temp_position;
    geometry_msgs::Point temp_position_world;
    DetectedObject temp_detected_object;
    bool flag_for_new_object = false;

    tf::StampedTransform stamped_tf;
    listener_.lookupTransform("world_graph_msf", "rgb_camera_optical_link", ros::Time(0), stamped_tf);
    optical_link_pose_in_world_temp_.pose.position.x = stamped_tf.getOrigin().x();
    optical_link_pose_in_world_temp_.pose.position.y = stamped_tf.getOrigin().y();
    optical_link_pose_in_world_temp_.pose.position.z = stamped_tf.getOrigin().z();
    optical_link_pose_in_world_temp_.header.frame_id = "world_graph_msf";
    current_optical_link_pose_pub_.publish(optical_link_pose_in_world_temp_);

    detected_object_pose_array_.poses.clear();

    if (object_num != 0) {
        for (int i = 0; i < object_num; i++) {
            temp_position = detection_info_array->info.at(i).position;
            // std::cout << "temp_position: "<<std::endl;
            // std::cout << temp_position.x << " " << temp_position.y << " "<< temp_position.z <<std::endl;

            // Transform the position to the global frame
            tf::Vector3 position_in_optical(temp_position.x, temp_position.y, temp_position.z);
                        tf::Vector3 position_in_global = stamped_tf * position_in_optical;

            temp_position_world.x = position_in_global.x();
            temp_position_world.y = position_in_global.y();
            temp_position_world.z = position_in_global.z();

            // std::cout << "temp_position_world: "<<std::endl;
            // std::cout << temp_position_world.x << " " << temp_position_world.y << " "<< temp_position_world.z <<std::endl;

            if(detected_object_vector_.size()==0){
                temp_detected_object.class_id = detection_info_array->info.at(i).class_id;
                temp_detected_object.position_world = temp_position_world;
                flag_for_new_object = true;
            } else{
                // Check if the detected object is new
                for (const auto& it : detected_object_vector_) {
                    double distance = calDistance(it, temp_position_world);
                    if (distance < kThreshold) {
                        flag_for_new_object = false;
                        break;
                    } else {
                        temp_detected_object.class_id = detection_info_array->info.at(i).class_id;
                        temp_detected_object.position_world = temp_position_world;
                        flag_for_new_object = true;
                    }
                }
            }

            detected_object_pose_temp_.position = temp_detected_object.position_world;
            detected_object_pose_array_.poses.push_back(detected_object_pose_temp_);

            // If it's a new object, add it to the vector and write to file
            if (flag_for_new_object) {
                // // Open a file in append mode
                // std::ofstream outfile;
                // outfile.open("/home/scarlett/Workspaces/smb_ws/src/object_detection/record_detection/data/example.txt", std::ios_base::app);
                // if (!outfile.is_open()) {
                //     std::cerr << "Cannot open the file" << std::endl;
                //     return;
                // }
                // outfile << "Detected Object: [" << temp_detected_object.class_id << "], position in the world: ["
                //         << temp_detected_object.position_world.x << ", "
                //         << temp_detected_object.position_world.y << ", "
                //         << temp_detected_object.position_world.z << "]\n";
                // outfile.close();
                
                detected_object_vector_.push_back(temp_detected_object);
                detected_object_temp_.position = temp_detected_object.position_world;
                detected_object_temp_.class_id = temp_detected_object.class_id;
                detected_object_array_.info.push_back(detected_object_temp_);
            }
        }
    current_detected_objects_pub_.publish(detected_object_array_);// content to save in txt or csv
    current_detected_objects_pose_pub_.publish(detected_object_pose_array_);

    }
}

int main(int argc, char* argv[]) {
    setlocale(LC_ALL, "");

    ros::init(argc, argv, "record_detection");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    RecordDetection record_detection_node(nh, private_nh);

    ros::spin();

    return 0;
}