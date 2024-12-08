#ifndef WHYCON_ROS_CWHYCONROS2NODE_H
#define WHYCON_ROS_CWHYCONROS2NODE_H

#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>

// #include <dynamic_reconfigure/server.h>
// #include <tf2_ros/transform_broadcaster.h>
// #include "whycon/whyconConfig.h"

#include <sensor_msgs/msg/camera_info.hpp>

#include <whycon/srv/select_marker.hpp>
#include <whycon/srv/set_calib_method.hpp>
#include <whycon/srv/set_calib_path.hpp>
#include <whycon/srv/set_coords.hpp>
#include <whycon/srv/set_drawing.hpp>
#include <whycon/srv/get_gui_settings.hpp>

#include <whycon/msg/marker_array.hpp>
#include <whycon/msg/marker.hpp>

#include "whycon/whycon.h"


namespace whycon_ros2
{

class CWhyconROSNode
{

    public:
        void getGuiSettingsCallback(const std::shared_ptr<whycon::srv::GetGuiSettings::Request> req,
                                          std::shared_ptr<whycon::srv::GetGuiSettings::Response> res);

        void setDrawingCallback(const std::shared_ptr<whycon::srv::SetDrawing::Request> req,
                                      std::shared_ptr<whycon::srv::SetDrawing::Response> res);

        void setCoordsCallback(const std::shared_ptr<whycon::srv::SetCoords::Request> req,
                                     std::shared_ptr<whycon::srv::SetCoords::Response> res);

        void setCalibMethodCallback(const std::shared_ptr<whycon::srv::SetCalibMethod::Request> req,
                                          std::shared_ptr<whycon::srv::SetCalibMethod::Response> res);

        void setCalibPathCallback(const std::shared_ptr<whycon::srv::SetCalibPath::Request> req,
                                        std::shared_ptr<whycon::srv::SetCalibPath::Response> res);

        void selectMarkerCallback(const std::shared_ptr<whycon::srv::SelectMarker::Request> req,
                                        std::shared_ptr<whycon::srv::SelectMarker::Response> res);

        // void reconfigureCallback(whycon::whyconConfig& config, uint32_t level);

        void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

        void imageCallback(const sensor_msgs::msg::Image::ConstPtr &msg);

        void start();

        CWhyconROSNode();

        ~CWhyconROSNode();

    private:
        std::shared_ptr<rclcpp::Node> node;

        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
        image_transport::Subscriber img_sub_;

        image_transport::Publisher img_pub_;
        rclcpp::Publisher<whycon::msg::MarkerArray>::SharedPtr markers_pub_;
        // ros::Publisher visual_pub_;

        bool draw_coords_;
        rclcpp::Service<whycon::srv::GetGuiSettings>::SharedPtr gui_settings_srv_;
        rclcpp::Service<whycon::srv::SetDrawing>::SharedPtr drawing_srv_;
        rclcpp::Service<whycon::srv::SetCoords>::SharedPtr coord_system_srv_;
        rclcpp::Service<whycon::srv::SetCalibMethod>::SharedPtr calib_method_srv_;
        rclcpp::Service<whycon::srv::SetCalibPath>::SharedPtr calib_path_srv_;
        rclcpp::Service<whycon::srv::SelectMarker>::SharedPtr select_marker_srv_;
        
        bool publish_visual_;   // whether to publish visualization msgs
        bool use_gui_;          // generate images for graphic interface?
        whycon::CWhycon whycon_;        // WhyCon instance
        whycon::CRawImage *image_;      // image wrapper for WhyCon
        double circle_diameter_;  // marker diameter [m]

        std::vector<whycon::SMarker> whycon_detections_;    // array of detected markers
        
        std::vector<float> intrinsic_mat_;        // intrinsic matrix from camera_info topic
        std::vector<float> distortion_coeffs_;    // distortion parameters from camera_info topic

        // dynamic_reconfigure::Server<whycon::whyconConfig> dyn_srv_;
        // dynamic_reconfigure::Server<whycon::whyconConfig>::CallbackType dyn_srv_cb_;

        bool identify_;
        bool publish_tf_;
        // tf2_ros::TransformBroadcaster tf_broad_;
};

}  // namespace whycon_ros2


#endif  // WHYCON_ROS_CWHYCONROS2NODE_H
