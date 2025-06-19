#include "whycode_ros/CWhycodeROSNode.h"

#include <functional>
#include <memory>
#include <string>
#include <algorithm>

using std::placeholders::_1;
using std::placeholders::_2;

namespace whycode_ros2
{

void CWhycodeROSNode::setCalibMethodCallback(const std::shared_ptr<whycode_interfaces::srv::SetCalibMethod::Request> req,
                                                  std::shared_ptr<whycode_interfaces::srv::SetCalibMethod::Response> res)
{
    RCLCPP_INFO(this->get_logger(), "setCalibMethodCallback %d", req->method);
    try
    {
        if(req->method == 0)
        {
            whycode_->autocalibration();
            res->success = true;
        }
        else if(req->method == 1)
        {
            whycode_->manualcalibration();
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

void CWhycodeROSNode::setCalibPathCallback(const std::shared_ptr<whycode_interfaces::srv::SetCalibPath::Request> req,
                                                std::shared_ptr<whycode_interfaces::srv::SetCalibPath::Response> res)
{
    RCLCPP_INFO(this->get_logger(), "setCalibPathCallback action %s path %s", req->action.c_str(), req->path.c_str());
    try
    {
        if(req->action == "load")
        {
            loadCalibration(req->path);
            res->success = true;
        }
        else if(req->action == "save")
        {
            saveCalibration(req->path);
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

void CWhycodeROSNode::selectMarkerCallback(const std::shared_ptr<whycode_interfaces::srv::SelectMarker::Request> req,
                                                std::shared_ptr<whycode_interfaces::srv::SelectMarker::Response> res)
{
    RCLCPP_INFO(this->get_logger(), "selectMarkerCallback x %f y %f", req->point.x, req->point.y);
    whycode_->selectMarker(req->point.x, req->point.y);
}

void CWhycodeROSNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    if(msg->k[0] == 0)
    {
        RCLCPP_ERROR_ONCE(this->get_logger(), "ERROR: Camera is not calibrated!");
        return;
    }

    whycode_->updateCameraInfo(msg->k, msg->d);
}

void CWhycodeROSNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
{
    image_.updateImage((unsigned char*)&msg->data[0], msg->width, msg->height, msg->step / msg->width);

    whycode_->processImage(image_, whycode_detections_);

    whycode_interfaces::msg::MarkerArray marker_array;
    marker_array.header.stamp = msg->header.stamp;
    marker_array.header.frame_id = msg->header.frame_id;

    for(const whycode::SMarker &detection : whycode_detections_)
    {
        whycode_interfaces::msg::Marker marker;

        marker.id = detection.seg.ID;
        marker.size = detection.seg.size;
        marker.u = detection.seg.x;
        marker.v = detection.seg.y;
        marker.angle = detection.obj.angle;

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
    }

    if(marker_array.markers.size() > 0)
    {
        markers_pub_->publish(marker_array);
    }

    if(why_params_.use_gui && img_pub_.getNumSubscribers() > 0)
    {
        sensor_msgs::msg::Image out_msg;
        out_msg.header = msg->header;
        out_msg.height = msg->height;
        out_msg.width = msg->width;
        out_msg.encoding = msg->encoding;
        out_msg.step = msg->step;
        out_msg.data.resize(msg->step * msg->height);
        std::memcpy((void*)&out_msg.data[0], image_.data_, msg->step * msg->height);
        img_pub_.publish(out_msg);
    }

    whycode_detections_.clear();
}

CWhycodeROSNode::CWhycodeROSNode() :
    Node("whycode_node")
{
    // node parameters
    std::string img_transport = this->declare_parameter("img_transport", std::string("raw"));
    std::string img_base_topic = this->declare_parameter("img_base_topic", std::string("/camera/image_raw"));
    std::string info_topic = this->declare_parameter("info_topic", std::string("/camera/camera_info"));
    std::string calib_path = this->declare_parameter("calib_file", std::string("./calib.yaml"));

    process_lib_parameters();

    whycode_ = std::make_unique<whycode::CWhycode>();
    whycode_->init(why_params_.circle_diameter, why_params_.id_bits, why_params_.id_samples, why_params_.hamming_dist);
    whycode_->set_parameters(why_params_);

    on_set_parameters_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&CWhycodeROSNode::validate_upcoming_parameters_callback, this, _1));
    post_set_parameters_callback_handle_ = this->add_post_set_parameters_callback(std::bind(&CWhycodeROSNode::react_to_updated_parameters_callback, this, _1));

    // "img_transport" parameter can chagne the transport during startup. default is "raw" (see image_transport::TransportHints)
    img_sub_ = image_transport::create_subscription(this, img_base_topic, std::bind(&CWhycodeROSNode::imageCallback, this, _1), img_transport);
    cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(info_topic, 1, std::bind(&CWhycodeROSNode::cameraInfoCallback, this, _1));
    img_pub_ = image_transport::create_publisher(this, "~/processed_image");
    markers_pub_ = this->create_publisher<whycode_interfaces::msg::MarkerArray>("~/markers", 1);

    calib_method_srv_ = this->create_service<whycode_interfaces::srv::SetCalibMethod>("~/set_calib_method", std::bind(&CWhycodeROSNode::setCalibMethodCallback, this, _1, _2));
    calib_path_srv_ = this->create_service<whycode_interfaces::srv::SetCalibPath>("~/set_calib_path", std::bind(&CWhycodeROSNode::setCalibPathCallback, this, _1, _2));
    select_marker_srv_ = this->create_service<whycode_interfaces::srv::SelectMarker>("~/select_marker", std::bind(&CWhycodeROSNode::selectMarkerCallback, this, _1, _2));

    if(calib_path.size() > 0){
        try{
            loadCalibration(calib_path);
            whycode_->setCoordinates(static_cast<whycode::ETransformType>(why_params_.coords_method));
        }catch(const std::exception& e){
            RCLCPP_WARN(this->get_logger(), "Calibration file '%s' could not be loaded. Using camera centric coordinates.", calib_path.c_str());
        }
    }else{
        RCLCPP_INFO(this->get_logger(), "Calibration file path empty. Using camera centric coordinates.");
    }
}

void CWhycodeROSNode::process_node_parameters()
{
    
}

void CWhycodeROSNode::process_lib_parameters()
{
    // static lib parameters
    why_params_.id_bits = this->declare_parameter("id_bits", 6);
    why_params_.id_samples = this->declare_parameter("id_samples", 360);
    why_params_.hamming_dist = this->declare_parameter("hamming_dist", 1);

    // dynamic lib parameters
    why_params_.draw_coords = this->declare_parameter("draw_coords", true);
    why_params_.draw_segments = this->declare_parameter("draw_segments", true);
    why_params_.use_gui = this->declare_parameter("use_gui", true);
    why_params_.identify = this->declare_parameter("identify", true);
    why_params_.coords_method = this->declare_parameter("coords_method", 0);
    why_params_.num_markers = this->declare_parameter("num_markers", 1);
    why_params_.min_size = this->declare_parameter("min_size", 100);
    why_params_.circle_diameter = this->declare_parameter("circle_diameter", 0.122);
    why_params_.field_length = this->declare_parameter("field_length", 1.0);
    why_params_.field_width = this->declare_parameter("field_width", 1.0);
    why_params_.initial_circularity_tolerance = this->declare_parameter("initial_circularity_tolerance", 100.0);
    why_params_.final_circularity_tolerance = this->declare_parameter("final_circularity_tolerance", 2.0);
    why_params_.area_ratio_tolerance = this->declare_parameter("area_ratio_tolerance", 40.0);
    why_params_.center_distance_tolerance_ratio = this->declare_parameter("center_distance_tolerance_ratio", 10.0);
    why_params_.center_distance_tolerance_abs = this->declare_parameter("center_distance_tolerance_abs", 5.0);
}

rcl_interfaces::msg::SetParametersResult CWhycodeROSNode::validate_upcoming_parameters_callback(const std::vector<rclcpp::Parameter> & parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    
    for(const auto & parameter : parameters)
    {
        if(param.get_name() == "coords_method")
        {
            if(param.as_int() != 0 && !whycode_.is_calibrated())
            {
                result.successful = false;
                result.reason = "The coordinates calibration is not available. Load or perform calibration.";
            }
        }
    }

    return result;
}

void CWhycodeROSNode::react_to_updated_parameters_callback(const std::vector<rclcpp::Parameter> & parameters)
{
    auto why_params = whycode_->get_parameters();

    for(const auto & param : parameters)
    {
        if(param.get_name() == "circle_diameter")
        {
            why_params.circle_diameter = param.as_double();
        }
        else if(param.get_name() == "num_markers")
        {
            why_params.num_markers = param.as_int();
        }
        else if(param.get_name() == "identify")
        {
            why_params.identify = param.as_bool();
        }
        else if(param.get_name() == "min_size")
        {
            why_params.min_size = param.as_int();
        }
        else if(param.get_name() == "field_length")
        {
            why_params.field_length = param.as_double();
        }
        else if(param.get_name() == "field_width")
        {
            why_params.field_width = param.as_double();
        }
        else if(param.get_name() == "initial_circularity_tolerance")
        {
            why_params.initial_circularity_tolerance = param.as_double();
        }
        else if(param.get_name() == "final_circularity_tolerance")
        {
            why_params.final_circularity_tolerance = param.as_double();
        }
        else if(param.get_name() == "area_ratio_tolerance")
        {
            why_params.area_ratio_tolerance = param.as_double();
        }
        else if(param.get_name() == "center_distance_tolerance_ratio")
        {
            why_params.center_distance_tolerance_ratio = param.as_double();
        }
        else if(param.get_name() == "center_distance_tolerance_abs")
        {
            why_params.center_distance_tolerance_abs = param.as_double();
        }
        else if(param.get_name() == "draw_coords")
        {
            why_params.draw_coords = param.as_bool();
        }
        else if(param.get_name() == "draw_segments")
        {
            why_params.draw_segments = param.as_bool();
        }
        else if(param.get_name() == "coords_method")
        {
            why_params.coords_method = param.as_int();
        }
    }

    whycode_->set_parameters(why_params);
}

void CWhycodeROSNode::loadCalibration(const std::string &str)
{
    whycode::CalibrationConfig config;

    cv::FileStorage fs(str, cv::FileStorage::READ);
    if(!fs.isOpened())
        throw std::runtime_error("Could not open/load calibration file. " + str);

    fs["dim_x"] >> config.grid_dim_x_;
    fs["dim_y"] >> config.grid_dim_y_;

    cv::Mat hom_tmp(3, 3, CV_32FC1);
    fs["hom"] >> hom_tmp;
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            config.hom_[3 * i + j] = hom_tmp.at<float>(i, j);
        }
    }

    for(int k = 0; k < 4; k++){
        cv::Mat offset_tmp(3, 1, CV_32FC1);
        fs["offset_" + std::to_string(k)] >> offset_tmp;
        for(int i = 0; i < 3; i++){
            config.D3transform_[k].orig[i] = offset_tmp.at<float>(i);
        }

        cv::Mat simlar_tmp(3, 3, CV_32FC1);
        fs["simlar_" + std::to_string(k)] >> simlar_tmp;
        for(int i = 0; i < 3; i++){
            for(int j = 0; j < 3; j++){
                config.D3transform_[k].simlar[3 * i + j] = simlar_tmp.at<float>(i, j);
            }
        }
    }

    fs.release();

    whycode_->setCalibrationConfig(config);
}

void CWhycodeROSNode::saveCalibration(const std::string &str)
{
    whycode::CalibrationConfig config = whycode_->getCalibrationConfig();

    cv::FileStorage fs(str, cv::FileStorage::WRITE);
    if(!fs.isOpened())
        throw std::runtime_error("Could not open/create calibration file. " + str);

    fs.writeComment("Dimensions");
    fs << "dim_x" << config.grid_dim_x_;
    fs << "dim_y" << config.grid_dim_y_;
    fs.writeComment("2D calibration");
    fs << "hom" << cv::Mat(3, 3, CV_32FC1, config.hom_);
    fs.writeComment("3D calibration");

    for (int k = 0; k < 4; k++)
    {
        fs.writeComment("D3transform " + std::to_string(k));
        fs << "offset_" + std::to_string(k) << cv::Mat(3, 1, CV_32FC1, config.D3transform_[k].orig);
        fs << "simlar_" + std::to_string(k) << cv::Mat(3, 3, CV_32FC1, config.D3transform_[k].simlar);
    }

    fs.release();
}

}  // namespace whycode_ros2