#include "whycode_ros/CWhycodeROSNode.h"

#include <functional>
#include <memory>
#include <string>
#include <algorithm>

using std::placeholders::_1;
using std::placeholders::_2;

namespace whycode_ros2
{

void CWhycodeROSNode::getGuiSettingsCallback(const std::shared_ptr<whycode_interfaces::srv::GetGuiSettings::Request> req,
                                                  std::shared_ptr<whycode_interfaces::srv::GetGuiSettings::Response> res)
{
    RCLCPP_INFO(this->get_logger(), "getGuiSettingsCallback");
    res->draw_coords = whycode_.getDrawCoords();
    res->draw_segments = whycode_.getDrawSegments();
    res->coords = whycode_.getCoordinates();
}

void CWhycodeROSNode::setDrawingCallback(const std::shared_ptr<whycode_interfaces::srv::SetDrawing::Request> req,
                                              std::shared_ptr<whycode_interfaces::srv::SetDrawing::Response> res)
{
    RCLCPP_INFO(this->get_logger(), "setDrawingCallback coords %d segs %d", req->draw_coords, req->draw_segments);
    whycode_.setDrawing(req->draw_coords, req->draw_segments);
    res->success = true;
}

void CWhycodeROSNode::setCoordsCallback(const std::shared_ptr<whycode_interfaces::srv::SetCoords::Request> req,
                                             std::shared_ptr<whycode_interfaces::srv::SetCoords::Response> res)
{
    RCLCPP_INFO(this->get_logger(), "setCoordsCallback %d", req->coords);
    try
    {
        whycode_.setCoordinates(static_cast<whycode::ETransformType>(req->coords));
        res->success = true;
    }
    catch(const std::exception& e)
    {
        res->success = false;
        res->msg = e.what();
    }
}

void CWhycodeROSNode::setCalibMethodCallback(const std::shared_ptr<whycode_interfaces::srv::SetCalibMethod::Request> req,
                                                  std::shared_ptr<whycode_interfaces::srv::SetCalibMethod::Response> res)
{
    RCLCPP_INFO(this->get_logger(), "setCalibMethodCallback %d", req->method);
    try
    {
        if(req->method == 0)
        {
            whycode_.autocalibration();
            res->success = true;
        }
        else if(req->method == 1)
        {
            whycode_.manualcalibration();
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
    whycode_.selectMarker(req->point.x, req->point.y);
}

/* void CWhycodeROSNode::reconfigureCallback(whycon::whyconConfig& config, uint32_t level)
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

void CWhycodeROSNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    if(msg->k[0] == 0)
    {
        RCLCPP_ERROR_ONCE(this->get_logger(), "ERROR: Camera is not calibrated!");
        return;
    }

    if(!std::equal(intrinsic_mat_.begin(), intrinsic_mat_.end(), msg->k.begin()))
        intrinsic_mat_.assign(msg->k.begin(), msg->k.end());

    if(!std::equal(distortion_coeffs_.begin(), distortion_coeffs_.end(), msg->d.begin()))
        distortion_coeffs_.assign(msg->d.begin(), msg->d.end());

    whycode_.updateCameraInfo(intrinsic_mat_, distortion_coeffs_);
}

void CWhycodeROSNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
{
    image_->updateImage((unsigned char*)&msg->data[0], msg->width, msg->height, msg->step / msg->width);

    whycode_.processImage(image_, whycode_detections_);

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

    if(use_gui_)
    {
        sensor_msgs::msg::Image out_msg;
        out_msg.header = msg->header;
        out_msg.height = msg->height;
        out_msg.width = msg->width;
        out_msg.encoding = msg->encoding;
        out_msg.step = msg->step;
        out_msg.data.resize(msg->step * msg->height);
        std::memcpy((void*)&out_msg.data[0], image_->data_.data(), msg->step * msg->height);
        img_pub_.publish(out_msg);
    }

    whycode_detections_.clear();
}

void CWhycodeROSNode::declareParameters()
{
    // rcl_interfaces::msg::FloatingPointRange range;
    // range.from_value = std::numeric_limits<double>::min();
    // range.to_value = std::numeric_limits<double>::max();
    // rcl_interfaces::msg::ParameterDescriptor descriptor;
    // descriptor.name = "circle_diameter";
    // descriptor.description = "Marker diameter [m]";
    // descriptor.integer_range.push_back(range);
    // circle_diameter_ = this->declare_parameter("circle_diameter", 0.122, descriptor);

    // rcl_interfaces::msg::IntegerRange range;
    // range.from_value = 1;
    // range.to_value = std::numeric_limits<int64_t>::max();
    // rcl_interfaces::msg::ParameterDescriptor descriptor;
    // descriptor.name = "min_size";
    // descriptor.description = "Minimal marker size [px]";
    // descriptor.integer_range.push_back(range);
    // min_size_ = this->declare_parameter("min_size", 100, descriptor);
}

CWhycodeROSNode::CWhycodeROSNode() :
    intrinsic_mat_(9),
    distortion_coeffs_(5),
    Node("whycode_node")
{
    int id_bits;
    int id_samples;
    int hamming_dist;
    std::string calib_path;
    int coords_method;
    std::string img_base_topic;
    std::string img_transport;
    std::string info_topic;

    // declareParameters();

    use_gui_ = this->declare_parameter("use_gui", true);
    id_bits = this->declare_parameter("id_bits", 6);
    id_samples = this->declare_parameter("id_samples", 360);
    hamming_dist = this->declare_parameter("hamming_dist", 1);
    num_markers_ = this->declare_parameter("num_markers", 1);
    calib_path = this->declare_parameter("calib_file", std::string(""));
    coords_method = this->declare_parameter("coords_method", 0);

    img_base_topic = this->declare_parameter("img_base_topic", std::string(""));
    img_transport = this->declare_parameter("img_transport", std::string(""));
    info_topic = this->declare_parameter("info_topic", std::string(""));

    image_ = new whycode::CRawImage();
    whycode_.init(circle_diameter_, use_gui_, id_bits, id_samples, hamming_dist, num_markers_);

    identify_ = this->declare_parameter("identify", true);
    double field_length = this->declare_parameter("field_length", 1.0);
    double field_width = this->declare_parameter("field_width", 1.0);
    double initial_circularity_tolerance = this->declare_parameter("initial_circularity_tolerance", 100.0);
    double final_circularity_tolerance = this->declare_parameter("final_circularity_tolerance", 2.0);
    double area_ratio_tolerance = this->declare_parameter("area_ratio_tolerance", 40.0);
    double center_distance_tolerance_ratio = this->declare_parameter("center_distance_tolerance_ratio", 10.0);
    double center_distance_tolerance_abs = this->declare_parameter("center_distance_tolerance_abs", 5.0);

    // whycon_.updateConfiguration(
    //     identify_, circle_diameter_, num_markers_, min_size_,
    //     field_length, field_width, initial_circularity_tolerance,
    //     final_circularity_tolerance, area_ratio_tolerance,
    //     center_distance_tolerance_ratio, center_distance_tolerance_abs
    //     );

    on_set_parameters_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&CWhycodeROSNode::validate_upcoming_parameters_callback, this, _1));
    post_set_parameters_callback_handle_ = this->add_post_set_parameters_callback(std::bind(&CWhycodeROSNode::react_to_updated_parameters_callback, this, _1));

    cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(info_topic, 1, std::bind(&CWhycodeROSNode::cameraInfoCallback, this, _1));
    markers_pub_ = this->create_publisher<whycode_interfaces::msg::MarkerArray>("~/markers", 1);

    // "img_transport" parameter can chagne the transport during startup. default is "raw" (see image_transport::TransportHints)
    img_sub_ = image_transport::create_subscription(this, img_base_topic, std::bind(&CWhycodeROSNode::imageCallback, this, _1), img_transport);
    img_pub_ = image_transport::create_publisher(this, "~/processed_image");

    gui_settings_srv_ = this->create_service<whycode_interfaces::srv::GetGuiSettings>("~/get_gui_settings", std::bind(&CWhycodeROSNode::getGuiSettingsCallback, this, _1, _2));
    drawing_srv_ = this->create_service<whycode_interfaces::srv::SetDrawing>("~/set_drawing", std::bind(&CWhycodeROSNode::setDrawingCallback, this, _1, _2));
    coord_system_srv_ = this->create_service<whycode_interfaces::srv::SetCoords>("~/set_coords", std::bind(&CWhycodeROSNode::setCoordsCallback, this, _1, _2));
    calib_method_srv_ = this->create_service<whycode_interfaces::srv::SetCalibMethod>("~/set_calib_method", std::bind(&CWhycodeROSNode::setCalibMethodCallback, this, _1, _2));
    calib_path_srv_ = this->create_service<whycode_interfaces::srv::SetCalibPath>("~/set_calib_path", std::bind(&CWhycodeROSNode::setCalibPathCallback, this, _1, _2));
    select_marker_srv_ = this->create_service<whycode_interfaces::srv::SelectMarker>("~/select_marker", std::bind(&CWhycodeROSNode::selectMarkerCallback, this, _1, _2));

    if(calib_path.size() > 0)
    {
        try
        {
            loadCalibration(calib_path);
            whycode_.setCoordinates(static_cast<whycode::ETransformType>(coords_method));
        }
        catch(const std::exception& e)
        {
            RCLCPP_WARN(this->get_logger(), "Calibration file '%s' could not be loaded. Using camera centric coordinates.", calib_path.c_str());
        }
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Calibration file path empty. Using camera centric coordinates.");
    }
}

rcl_interfaces::msg::SetParametersResult CWhycodeROSNode::validate_upcoming_parameters_callback(const std::vector<rclcpp::Parameter> & parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    // for(const auto & parameter : parameters)
    // {
    //     if(!some_condition)
    //     {
    //         result.successful = false;
    //         result.reason = "the reason it could not be allowed";
    //     }
    // }
    return result;
}

void CWhycodeROSNode::react_to_updated_parameters_callback(const std::vector<rclcpp::Parameter> & parameters)
{
    for(const auto & param:parameters)
    {
        // the internal class member can be changed after
        // successful change to param1 or param2
        if(param.get_name() == "param1")
        {
            // internal_tracked_class_parameter_1_ = param.get_value<double>();
        }
        else if(param.get_name() == "param2")
        {
            // internal_tracked_class_parameter_2_ = param.get_value<double>();
        }
        else if(param.get_name() == "use_gui")
        {

        }
        else if(param.get_name() == "circle_diameter")
        {

        }
        else if(param.get_name() == "id_bits")
        {

        }
        else if(param.get_name() == "id_samples")
        {

        }
        else if(param.get_name() == "hamming_dist")
        {

        }
        else if(param.get_name() == "num_markers")
        {

        }
        else if(param.get_name() == "calib_file")
        {

        }
        else if(param.get_name() == "coords_method")
        {

        }
        else if(param.get_name() == "min_size")
        {

        }
    }
}

CWhycodeROSNode::~CWhycodeROSNode()
{
    delete image_;
}

void CWhycodeROSNode::loadCalibration(const std::string &str)
{
    whycode::CalibrationConfig config;

    try
    {
        cv::FileStorage fs(str, cv::FileStorage::READ);
        if(!fs.isOpened())
            throw std::runtime_error("Could not open/load calibration file. " + str);

        fs["dim_x"] >> config.grid_dim_x_;
        fs["dim_y"] >> config.grid_dim_y_;

        cv::Mat hom_tmp(3, 3, CV_32FC1);
        fs["hom"] >> hom_tmp;
        for(int i = 0; i < 3; i++)
        {
            for(int j = 0; j < 3; j++)
                config.hom_[3 * i + j] = hom_tmp.at<float>(i, j);
        }

        for(int k = 0; k < 4; k++)
        {
            cv::Mat offset_tmp(3, 1, CV_32FC1);
            fs["offset_" + std::to_string(k)] >> offset_tmp;
            for(int i = 0; i < 3; i++)
                config.D3transform_[k].orig[i] = offset_tmp.at<float>(i);

            cv::Mat simlar_tmp(3, 3, CV_32FC1);
            fs["simlar_" + std::to_string(k)] >> simlar_tmp;
            for(int i = 0; i < 3; i++)
            {
                for(int j = 0; j < 3; j++)
                    config.D3transform_[k].simlar[3 * i + j] = simlar_tmp.at<float>(i, j);
            }
        }

        fs.release();
    }
    catch(const std::exception& e)
    {
        throw;
    }

    whycode_.setCalibrationConfig(config);
}

void CWhycodeROSNode::saveCalibration(const std::string &str)
{
    whycode::CalibrationConfig config = whycode_.getCalibrationConfig();

    try
    {
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
    catch(const std::exception& e)
    {
        throw;
    }
}

}  // namespace whycode_ros2