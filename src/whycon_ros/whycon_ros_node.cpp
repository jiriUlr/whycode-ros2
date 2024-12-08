#include "whycon_ros/whycon_ros_node.h"

#include <functional>
#include <memory>
#include <string>
#include <algorithm>

#include <geometry_msgs/msg/transform_stamped.hpp>
// #include <visualization_msgs/msg/marker_array.hpp>


namespace whycon_ros2
{

void CWhyconROSNode::getGuiSettingsCallback(const std::shared_ptr<whycon::srv::GetGuiSettings::Request> req,
                                                  std::shared_ptr<whycon::srv::GetGuiSettings::Response> res)
{
    RCLCPP_INFO(node->get_logger(), "getGuiSettingsCallback");
    res->draw_coords = whycon_.getDrawCoords();
    res->draw_segments = whycon_.getDrawSegments();
    res->coords = whycon_.getCoordinates();
}

void CWhyconROSNode::setDrawingCallback(const std::shared_ptr<whycon::srv::SetDrawing::Request> req,
                                              std::shared_ptr<whycon::srv::SetDrawing::Response> res)
{
    RCLCPP_INFO(node->get_logger(), "setDrawingCallback coords %d segs %d", req->draw_coords, req->draw_segments);
    whycon_.setDrawing(req->draw_coords, req->draw_segments);
    res->success = true;
}

void CWhyconROSNode::setCoordsCallback(const std::shared_ptr<whycon::srv::SetCoords::Request> req,
                                             std::shared_ptr<whycon::srv::SetCoords::Response> res)
{
    RCLCPP_INFO(node->get_logger(), "setCoordsCallback %d", req->coords);
    try
    {
        whycon_.setCoordinates(static_cast<whycon::ETransformType>(req->coords));
        res->success = true;
    }
    catch(const std::exception& e)
    {
        res->success = false;
        res->msg = e.what();
    }
}

void CWhyconROSNode::setCalibMethodCallback(const std::shared_ptr<whycon::srv::SetCalibMethod::Request> req,
                                                  std::shared_ptr<whycon::srv::SetCalibMethod::Response> res)
{
    RCLCPP_INFO(node->get_logger(), "setCalibMethodCallback %d", req->method);
    try
    {
        if(req->method == 0)
        {
            whycon_.autocalibration();
            res->success = true;
        }
        else if(req->method == 1)
        {
            whycon_.manualcalibration();
            res->success = true;
        }
        else
        {
            res->success = false;
            res->msg = "ERROR in setting calibration method : unkown method '" + std::to_string(req->method) + "'";
        }
    }
    catch(const std::exception& e)
    {
        res->success = false;
        res->msg = e.what();
    }
}

void CWhyconROSNode::setCalibPathCallback(const std::shared_ptr<whycon::srv::SetCalibPath::Request> req,
                                                std::shared_ptr<whycon::srv::SetCalibPath::Response> res)
{
    RCLCPP_INFO(node->get_logger(), "setCalibPathCallback action %s path %s", req->action.c_str(), req->path.c_str());
    try
    {
        if(req->action == "load")
        {
            whycon_.loadCalibration(req->path);
            res->success = true;
        }
        else if(req->action == "save")
        {
            whycon_.saveCalibration(req->path);
            res->success = true;
        }
        else
        {
            res->success = false;
            res->msg = "ERROR in setting calibration path : unkown action '" + req->action + "'";
        }
    }
    catch(const std::exception& e)
    {
        res->success = false;
        res->msg = e.what();
    }
}

void CWhyconROSNode::selectMarkerCallback(const std::shared_ptr<whycon::srv::SelectMarker::Request> req,
                                                std::shared_ptr<whycon::srv::SelectMarker::Response> res)
{
    RCLCPP_INFO(node->get_logger(), "selectMarkerCallback x %f y %f", req->point.x, req->point.y);
    whycon_.selectMarker(req->point.x, req->point.y);
}

/* void CWhyconROSNode::reconfigureCallback(whycon::whyconConfig& config, uint32_t level)
{
    ROS_INFO("[Reconfigure Request]\n"
        "identify %s circle_diameter %lf num_markers %d\n"
        "min_size %d field_length %lf field_width %lf\n"
        "initial_circularity_tolerance %lf final_circularity_tolerance %lf\n"
        "area_ratio_tolerance %lf\n"
        "center_distance_tolerance_ratio %lf center_distance_tolerance_abs %lf\n",
        (config.identify) ? "True" : "False",
        config.circle_diameter, config.num_markers, config.min_size, config.field_length,
        config.field_width, config.initial_circularity_tolerance,
        config.final_circularity_tolerance, config.area_ratio_tolerance,
        config.center_distance_tolerance_ratio, config.center_distance_tolerance_abs
        );

    whycon_.updateConfiguration(
        config.identify, config.circle_diameter, config.num_markers, config.min_size,
        config.field_length, config.field_width, config.initial_circularity_tolerance,
        config.final_circularity_tolerance, config.area_ratio_tolerance,
        config.center_distance_tolerance_ratio, config.center_distance_tolerance_abs
        );

    identify_ = config.identify;
} */

void CWhyconROSNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    if(msg->K[0] == 0)
    {
        RCLCPP_ERROR_ONCE(node->get_logger(), "ERROR: Camera is not calibrated!");
        return;
    }

    if(!std::equal(intrinsic_mat_.begin(), intrinsic_mat_.end(), msg->K.begin()))
        intrinsic_mat_.assign(msg->K.begin(), msg->K.end());

    if(!std::equal(distortion_coeffs_.begin(), distortion_coeffs_.end(), msg->D.begin()))
        distortion_coeffs_.assign(msg->D.begin(), msg->D.end());

    whycon_.updateCameraInfo(intrinsic_mat_, distortion_coeffs_);
}

void CWhyconROSNode::imageCallback(const sensor_msgs::msg::Image::ConstPtr &msg)
{
    // convert sensor_msgs::Image msg to whycon CRawImage
    image_->updateImage((unsigned char*)&msg->data[0], msg->width, msg->height, msg->step / msg->width);

    whycon_.processImage(image_, whycon_detections_);

    // generate information about markers into msgs
    whycon::msg::MarkerArray marker_array;
    marker_array.header.stamp = node->header.stamp;
    marker_array.header.frame_id = msg->header.frame_id;
    
    // tf vector
    // std::vector<geometry_msgs::TransformStamped> transform_array;

    // Generate RVIZ visualization marker
    // visualization_msgs::MarkerArray visual_array;

    for(const whycon::SMarker &detection : whycon_detections_)
    {
        whycon::msg::Marker marker;

        marker.id = detection.seg.ID;
        marker.size = detection.seg.size;
        marker.u = detection.seg.x;
        marker.v = detection.seg.y;
        marker.angle = detection.obj.angle;

        // Convert to ROS standard Coordinate System
        marker.position.position.x = detection.obj.x;
        marker.position.position.y = detection.obj.y;
        marker.position.position.z = detection.obj.z;
        marker.position.orientation.x = detection.obj.qx;
        marker.position.orientation.y = detection.obj.qy;
        marker.position.orientation.z = detection.obj.qz;
        marker.position.orientation.w = detection.obj.qw;
        marker.rotation.x = detection.obj.roll;
        marker.rotation.y = detection.obj.pitch;
        marker.rotation.z = detection.obj.yaw;
        marker_array.markers.push_back(marker);

        /* if(identify_ && publish_tf_)
        {
            geometry_msgs::TransformStamped transform_stamped;

            transform_stamped.header.stamp = ros::Time::now();
            transform_stamped.header.frame_id = msg->header.frame_id;
            transform_stamped.child_frame_id = "marker_" + std::to_string(detection.seg.ID);
            transform_stamped.transform.translation.x = detection.obj.x;
            transform_stamped.transform.translation.y = detection.obj.y;
            transform_stamped.transform.translation.z = detection.obj.z;
            transform_stamped.transform.rotation.x = detection.obj.qx;
            transform_stamped.transform.rotation.y = detection.obj.qy;
            transform_stamped.transform.rotation.z = detection.obj.qz;
            transform_stamped.transform.rotation.w = detection.obj.qw;

            transform_array.push_back(transform_stamped);
        } */

        /* if(publish_visual_)
        {
            visualization_msgs::Marker visual_marker;
            visual_marker.header.stamp = ros::Time::now();
            visual_marker.header.frame_id = msg->header.frame_id;
            visual_marker.ns = "whycon";
            visual_marker.id = detection.seg.ID;
            visual_marker.type = visualization_msgs::Marker::SPHERE;
            visual_marker.action = visualization_msgs::Marker::MODIFY;

            visual_marker.pose.position.x = detection.obj.x;
            visual_marker.pose.position.y = detection.obj.y;
            visual_marker.pose.position.z = detection.obj.z;
            visual_marker.pose.orientation.x = detection.obj.qx;
            visual_marker.pose.orientation.y = detection.obj.qy;
            visual_marker.pose.orientation.z = detection.obj.qz;
            visual_marker.pose.orientation.w = detection.obj.qw;

            visual_marker.scale.x = 0.01;//circleDiameter;  // meters
            visual_marker.scale.y = circle_diameter_;//circleDiameter;
            visual_marker.scale.z = circle_diameter_;
            visual_marker.color.r = 0.0;
            visual_marker.color.g = 1.0;
            visual_marker.color.b = 0.0;
            visual_marker.color.a = 1.0;
            visual_marker.lifetime = ros::Duration(0.1);  // secs
            visual_marker.frame_locked = true;

            visual_array.markers.push_back(visual_marker);
        } */
    }

    // publishing detected markers
    if(marker_array.markers.size() > 0)
    {
        markers_pub_->publish(marker_array);

        /* if(publish_visual_)
            visual_pub_.publish(visual_array);

        for(int i = 0; i < transform_array.size(); i++)
            tf_broad_.sendTransform(transform_array[i]); */
    }

    if(use_gui_)
    {
        std::memcpy((void*)&msg->data[0], image_->data_, msg->step * msg->height);
        img_pub_.publish(msg);
    }

    whycon_detections_.clear();
}

void CWhyconROSNode::start()
{
    while(rclcpp::ok())
    {
        rclcpp::spin_some(node);
        usleep(10000);
    }
}

CWhyconROSNode::CWhyconROSNode() :
    intrinsic_mat_(9),
    distortion_coeffs_(5),
    node(rclcpp::Node::make_shared("~"))
{
    // ros::NodeHandle nh("~");
    
    int id_bits;
    int id_samples;
    int hamming_dist;
    int num_markers;
    std::string calib_path;
    int coords_method;

    node->declare_parameter("use_gui", true);
    node->declare_parameter("pub_visual", false);
    node->declare_parameter("pub_tf", false);
    node->declare_parameter("circle_diameter", 0.122);
    node->declare_parameter("id_bits", 6);
    node->declare_parameter("id_samples", 360);
    node->declare_parameter("hamming_dist", 1);
    node->declare_parameter("num_markers", 10);
    node->declare_parameter("calib_file", std::string(""));
    node->declare_parameter("coords_method", 0);

    use_gui_ = node->get_parameter("use_gui").as_bool();
    publish_visual_ = node->get_parameter("pub_visual").as_bool();
    publish_tf_ = node->get_parameter("pub_tf").as_bool();
    circle_diameter_ = node->get_parameter("circle_diameter").as_double();
    id_bits = node->get_parameter("id_bits").as_int();
    id_samples = node->get_parameter("id_samples").as_int();
    hamming_dist = node->get_parameter("hamming_dist").as_int();
    num_markers = node->get_parameter("num_markers").as_int();
    calib_path = node->get_parameter("calib_file").as_string();
    coords_method = node->get_parameter("coords_method").as_int();

    int default_width = 640;
    int default_height = 480;
    image_ = new whycon::CRawImage(default_width, default_height, 3);
    whycon_.init(circle_diameter_, use_gui_, id_bits, id_samples, hamming_dist, num_markers, default_width, default_height);
    
    // cam_info_sub_ = nh.subscribe("/camera/camera_info", 1, &CWhyconROSNode::cameraInfoCallback, this);
    cam_info_sub_ = node->create_subscription<sensor_msgs::msg::CameraInfo>("/camera/camera_info", 1, std::bind(&CWhyconROSNode::cameraInfoCallback, this, _1));
    image_transport::ImageTransport it(nh);
    img_sub_ = it.subscribe("/camera/image_raw", 1, std::bind(&CWhyconROSNode::imageCallback, this, _1));
    
    img_pub_ = it.advertise("processed_image", 1);
    // markers_pub_ = nh.advertise<whycon::MarkerArray>("markers", 1);
    markers_pub_ = node->create_publisher<whycon::msg::MarkerArray>("markers", 1);
    // visual_pub_ = nh.advertise<visualization_msgs::MarkerArray>("visualisation", 1);

    gui_settings_srv_ = node->create_service<whycon::srv::GetGuiSettings>("get_gui_settings", std::bind(&CWhyconROSNode::getGuiSettingsCallback, this, _1));
    drawing_srv_ = node->create_service<whycon::srv::SetDrawing>("set_drawing", std::bind(&CWhyconROSNode::setDrawingCallback, this, _1));
    coord_system_srv_ = node->create_service<whycon::srv::SetCoords>("set_coords", std::bind(&CWhyconROSNode::setCoordsCallback, this, _1));
    calib_method_srv_ = node->create_service<whycon::srv::SetCalibMethod>("set_calib_method", std::bind(&CWhyconROSNode::setCalibMethodCallback, this, _1));
    calib_path_srv_ = node->create_service<whycon::srv::SetCalibPath>("set_calib_path", std::bind(&CWhyconROSNode::setCalibPathCallback, this, _1));
    select_marker_srv_ = node->create_service<whycon::srv::SelectMarker>("select_marker", std::bind(&CWhyconROSNode::selectMarkerCallback, this, _1));
    
    // // create dynamic reconfigure server
    // dyn_srv_cb_ = boost::bind(&CWhyconROSNode::reconfigureCallback, this, _1, _2);
    // dyn_srv_.setCallback(dyn_srv_cb_);

    if(calib_path.size() > 0)
    {
        try
        {
            whycon_.loadCalibration(calib_path);
            whycon_.setCoordinates(static_cast<whycon::ETransformType>(coords_method));
        }
        catch(const std::exception& e)
        {
            RCLCPP_WARN(node->get_logger(), "Calibration file '%s' could not be loaded. Using camera centric coordinates.", calib_path.c_str());
        }
    }
    else
    {
        RCLCPP_INFO(node->get_logger(), "Calibration file path empty. Using camera centric coordinates.");
    }
}

CWhyconROSNode::~CWhyconROSNode()
{
    delete image_;
}

}

int main(int argc, char *argv[])
{
    // ros::init(argc, argv, "whycon");
    rclcpp::init(argc, argv);

    whycon_ros::CWhyconROSNode whycon_ros_node;
    whycon_ros_node.start();

    return 0;
}
