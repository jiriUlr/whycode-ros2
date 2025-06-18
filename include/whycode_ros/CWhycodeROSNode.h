#ifndef WHYCODEROS2_CWHYCODEROSNODE_H
#define WHYCODEROS2_CWHYCODEROSNODE_H

#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

// #include <rcl_interfaces/msg/SetParametersResult.hpp>

#include <whycode_interfaces/srv/select_marker.hpp>
#include <whycode_interfaces/srv/set_calib_method.hpp>
#include <whycode_interfaces/srv/set_calib_path.hpp>
#include <whycode_interfaces/srv/set_coords.hpp>
#include <whycode_interfaces/srv/set_drawing.hpp>
#include <whycode_interfaces/srv/get_gui_settings.hpp>

#include <whycode_interfaces/msg/marker_array.hpp>
#include <whycode_interfaces/msg/marker.hpp>

#include "whycode/whycode.h"


namespace whycode_ros2
{

class CWhycodeROSNode : public rclcpp::Node
{

public:

    CWhycodeROSNode();

    void getGuiSettingsCallback(const std::shared_ptr<whycode_interfaces::srv::GetGuiSettings::Request> req,
                                      std::shared_ptr<whycode_interfaces::srv::GetGuiSettings::Response> res);

    void setDrawingCallback(const std::shared_ptr<whycode_interfaces::srv::SetDrawing::Request> req,
                                  std::shared_ptr<whycode_interfaces::srv::SetDrawing::Response> res);

    void setCoordsCallback(const std::shared_ptr<whycode_interfaces::srv::SetCoords::Request> req,
                                 std::shared_ptr<whycode_interfaces::srv::SetCoords::Response> res);

    void setCalibMethodCallback(const std::shared_ptr<whycode_interfaces::srv::SetCalibMethod::Request> req,
                                      std::shared_ptr<whycode_interfaces::srv::SetCalibMethod::Response> res);

    void setCalibPathCallback(const std::shared_ptr<whycode_interfaces::srv::SetCalibPath::Request> req,
                                    std::shared_ptr<whycode_interfaces::srv::SetCalibPath::Response> res);

    void selectMarkerCallback(const std::shared_ptr<whycode_interfaces::srv::SelectMarker::Request> req,
                                    std::shared_ptr<whycode_interfaces::srv::SelectMarker::Response> res);

    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);

    void saveCalibration(const std::string &str);
    void loadCalibration(const std::string &str);

    rcl_interfaces::msg::SetParametersResult validate_upcoming_parameters_callback(const std::vector<rclcpp::Parameter> & parameters);
    void react_to_updated_parameters_callback(const std::vector<rclcpp::Parameter> & parameters);
    void process_node_parameters();
    void process_lib_parameters();

private:

    whycode::Parameters why_params_;

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr   on_set_parameters_callback_handle_;
    rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr post_set_parameters_callback_handle_;

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr      cam_info_sub_;
    rclcpp::Publisher<whycode_interfaces::msg::MarkerArray>::SharedPtr markers_pub_;

    image_transport::Subscriber img_sub_;
    image_transport::Publisher  img_pub_;
    
    rclcpp::Service<whycode_interfaces::srv::GetGuiSettings>::SharedPtr gui_settings_srv_;
    rclcpp::Service<whycode_interfaces::srv::SetDrawing>::SharedPtr     drawing_srv_;
    rclcpp::Service<whycode_interfaces::srv::SetCoords>::SharedPtr      coord_system_srv_;
    rclcpp::Service<whycode_interfaces::srv::SetCalibMethod>::SharedPtr calib_method_srv_;
    rclcpp::Service<whycode_interfaces::srv::SetCalibPath>::SharedPtr   calib_path_srv_;
    rclcpp::Service<whycode_interfaces::srv::SelectMarker>::SharedPtr   select_marker_srv_;

    bool draw_coords_;
    bool use_gui_;                  // generate images for graphic interface?
    std::unique_ptr<whycode::CWhycode> whycode_;     // WhyCon instance
    whycode::CRawImage image_;      // image wrapper for WhyCon
    double circle_diameter_;        // marker diameter [m]

    std::vector<whycode::SMarker> whycode_detections_;    // array of detected markers
    
    bool identify_;
    int num_markers_;
    int min_size_;
};

}  // namespace whycode_ros2


#endif  // WHYCODEROS2_CWHYCODEROSNODE_H
