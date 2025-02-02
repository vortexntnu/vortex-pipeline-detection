#include <pipeline_filters/pipeline_filtering_ros.hpp>
#include <rclcpp_components/register_node_macro.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

namespace vortex::pipeline_processing {


PipelineFilteringNode::PipelineFilteringNode(const rclcpp::NodeOptions & options) : Node("pipeline_filtering_node", options) 
{
    this->declare_parameter<std::string>("sub_topic", "/flir_camera/image_raw");
    this->declare_parameter<std::string>("pub_topic", "/filtered_image");
    this->declare_parameter<std::string>("filter_params.filter_type", "ebus");
    this->declare_parameter<bool>("filter_params.otsu.gamma_auto_correction", true);
    this->declare_parameter<double>("filter_params.otsu.gamma_auto_correction_weight", 1.0);
    this->declare_parameter<bool>("filter_params.otsu.otsu_segmentation", true);
    this->declare_parameter<double>("filter_params.otsu.gsc_weight_r", 0.5);
    this->declare_parameter<double>("filter_params.otsu.gsc_weight_g", 0.5);
    this->declare_parameter<double>("filter_params.otsu.gsc_weight_b", 0.0);

    // Set up the QoS profile for the image subscriber
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos_sensor_data = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);

    check_and_subscribe_to_image_topic();
    set_filter_params();

    initialize_parameter_handler();

    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/filtered_image", qos_sensor_data);
    optimal_threshold_publisher_ = this->create_publisher<std_msgs::msg::Int32>("/optimal_threshold", 10);
    auto_gamma_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/auto_gamma", 10.0);
}

void PipelineFilteringNode::set_filter_params(){
    FilterParams params;
    std::string filter = this->get_parameter("filter_params.filter_type").as_string();
    if(!filter_functions.contains(filter)){
        RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Invalid filter type: " << filter << " Setting to no_filter.");
        filter_ = "no_filter";
    } else{
        filter_ = filter;
    }
    params.otsu.gamma_auto_correction = this->get_parameter("filter_params.otsu.gamma_auto_correction").as_bool();
    params.otsu.gamma_auto_correction_weight = this->get_parameter("filter_params.otsu.gamma_auto_correction_weight").as_double();
    params.otsu.otsu_segmentation = this->get_parameter("filter_params.otsu.otsu_segmentation").as_bool();
    params.otsu.gsc_weight_r = this->get_parameter("filter_params.otsu.gsc_weight_r").as_double();
    params.otsu.gsc_weight_g = this->get_parameter("filter_params.otsu.gsc_weight_g").as_double();
    params.otsu.gsc_weight_b = this->get_parameter("filter_params.otsu.gsc_weight_b").as_double();
    filter_params_ = params;
    RCLCPP_INFO(this->get_logger(), "Filter parameters updated.");
}

void PipelineFilteringNode::check_and_subscribe_to_image_topic() {
    std::string image_topic = this->get_parameter("sub_topic").as_string();
    if (image_topic_ != image_topic) {
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            image_topic, 10, std::bind(&PipelineFilteringNode::image_callback, this, _1));
        image_topic_ = image_topic;
        RCLCPP_INFO(this->get_logger(), "Subscribed to image topic: %s", image_topic.c_str());
    }
}

void PipelineFilteringNode::initialize_parameter_handler() {
    param_handler_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
    
    // Register the parameter event callback with the correct signature
    auto parameter_event_callback =
        [this](const rcl_interfaces::msg::ParameterEvent & event) -> void {
            this->on_parameter_event(event);
        };

    // Register the callback with the parameter event handler
    param_cb_handle_ = param_handler_->add_parameter_event_callback(parameter_event_callback);
}

void PipelineFilteringNode::on_parameter_event(const rcl_interfaces::msg::ParameterEvent & event) {
     // Get the fully qualified name of the current node
    auto node_name = this->get_fully_qualified_name();

    // Filter out events not related to this node
    if (event.node != node_name) {
        return; // Early return if the event is not from this node
    }
    RCLCPP_INFO(this->get_logger(), "Received parameter event");
    for (const auto& changed_parameter : event.changed_parameters) {
        if (changed_parameter.name.find("sub_topic") == 0) check_and_subscribe_to_image_topic();
        if (changed_parameter.name.find("filter_params") == 0) set_filter_params(); 
    } 
}

void PipelineFilteringNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    RCLCPP_INFO_ONCE(this->get_logger(), "Received image message.");
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        if (cv_ptr->image.empty()) {
            RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Empty image received, skipping processing.");
            return;
        }

    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "cv_bridge exception: " << e.what());
        return;
    }
    
    cv::Mat input_image = cv_ptr->image;
    cv::Mat filtered_image;

    // Calculate optimal threshold and auto gamma as an example (you'll need to replace these with your actual calculations)
    int optimalThreshold = 123; //calculateOptimalThreshold(input_image); // Replace with actual calculation logic
    float autoGamma = 1.23; //calculateAutoGamma(input_image);             // Replace with actual calculation logic

    int channels = input_image.channels();
    RCLCPP_INFO_ONCE(rclcpp::get_logger("image_info"), "Number of channels: %d", channels);

    apply_filter(filter_, filter_params_, input_image, filtered_image);

    // Create a unique pointer for the message
    auto message = std::make_unique<sensor_msgs::msg::Image>();
    cv_bridge::CvImage(msg->header, "mono8", filtered_image).toImageMsg(*message);

    // Publish the message using a unique pointer
    image_pub_->publish(std::move(message));

    // Publish the calculated optimalThreshold
    auto threshold_msg = std_msgs::msg::Int32();
    threshold_msg.data = optimalThreshold;
    optimal_threshold_publisher_->publish(threshold_msg);

    // Publish the calculated autoGamma
    auto gamma_msg = std_msgs::msg::Float32();
    gamma_msg.data = autoGamma;
    auto_gamma_publisher_->publish(gamma_msg);
}

RCLCPP_COMPONENTS_REGISTER_NODE(vortex::pipeline_processing::PipelineFilteringNode)

} // namespace vortex::pipeline_processing
