#include <pipeline_filters/pipeline_filtering_ros.hpp>
#include <rclcpp_components/register_node_macro.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

namespace vortex::pipeline_processing {

PipelineFilteringNode::PipelineFilteringNode(const rclcpp::NodeOptions &options)
    : Node("pipeline_filtering_node", options) {
  this->declare_parameter<std::string>("sub_topic", "/flir_camera/image_raw");
  this->declare_parameter<std::string>("pub_topic", "/filtered_image");
  this->declare_parameter<std::string>("filter_params.filter_type", "ebus");
  this->declare_parameter<bool>("filter_params.otsu.gamma_auto_correction",
                                true);
  this->declare_parameter<double>(
      "filter_params.otsu.gamma_auto_correction_weight", 1.0);
  this->declare_parameter<bool>("filter_params.otsu.otsu_segmentation", true);
  this->declare_parameter<double>("filter_params.otsu.gsc_weight_r", 0.5);
  this->declare_parameter<double>("filter_params.otsu.gsc_weight_g", 0.5);
  this->declare_parameter<double>("filter_params.otsu.gsc_weight_b", 0.0);

  check_and_subscribe_to_image_topic();
  create_image_publisher();
  set_filter_params();

  initialize_parameter_handler();
}

void PipelineFilteringNode::set_filter_params() {
  FilterParams params;
  std::string filter =
      this->get_parameter("filter_params.filter_type").as_string();
  if (!filter_functions.contains(filter)) {
    RCLCPP_ERROR_STREAM_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Invalid filter type: " << filter << " Setting to no_filter.");
    filter_ = "no_filter";
  } else {
    filter_ = filter;
  }
  params.otsu.gamma_auto_correction =
      this->get_parameter("filter_params.otsu.gamma_auto_correction").as_bool();
  params.otsu.gamma_auto_correction_weight =
      this->get_parameter("filter_params.otsu.gamma_auto_correction_weight")
          .as_double();
  params.otsu.otsu_segmentation =
      this->get_parameter("filter_params.otsu.otsu_segmentation").as_bool();
  params.otsu.gsc_weight_r =
      this->get_parameter("filter_params.otsu.gsc_weight_r").as_double();
  params.otsu.gsc_weight_g =
      this->get_parameter("filter_params.otsu.gsc_weight_g").as_double();
  params.otsu.gsc_weight_b =
      this->get_parameter("filter_params.otsu.gsc_weight_b").as_double();
  filter_params_ = params;
  RCLCPP_INFO(this->get_logger(), "Filter parameters updated.");
}

void PipelineFilteringNode::check_and_subscribe_to_image_topic() {
  auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(1))
                         .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  std::string image_topic = this->get_parameter("sub_topic").as_string();
  if (image_topic_ != image_topic) {
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        image_topic, qos_profile,
        std::bind(&PipelineFilteringNode::image_callback, this, _1));
    image_topic_ = image_topic;
    RCLCPP_INFO(this->get_logger(), "Subscribed to image topic: %s",
                image_topic.c_str());
  }
}

void PipelineFilteringNode::create_image_publisher() {
  auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(1))
                         .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  std::string pub_topic = this->get_parameter("pub_topic").as_string();
  if (image_pub_topic_ != pub_topic) {
    image_pub_ =
        this->create_publisher<sensor_msgs::msg::Image>(pub_topic, qos_profile);
    image_pub_topic_ = pub_topic;
    RCLCPP_INFO(this->get_logger(), "Created publisher for filtered image: %s",
                pub_topic.c_str());
  }
}

void PipelineFilteringNode::initialize_parameter_handler() {
  param_handler_ = std::make_shared<rclcpp::ParameterEventHandler>(this);

  auto parameter_event_callback =
      [this](const rcl_interfaces::msg::ParameterEvent &event) -> void {
    this->on_parameter_event(event);
  };

  param_cb_handle_ =
      param_handler_->add_parameter_event_callback(parameter_event_callback);
}

void PipelineFilteringNode::on_parameter_event(
    const rcl_interfaces::msg::ParameterEvent &event) {
  auto node_name = this->get_fully_qualified_name();

  if (event.node != node_name) {
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Received parameter event");
  for (const auto &changed_parameter : event.changed_parameters) {
    if (changed_parameter.name.find("image_sub_topic") == 0)
      check_and_subscribe_to_image_topic();
    if (changed_parameter.name.find("filter_params") == 0)
      set_filter_params();
    if (changed_parameter.name.find("image_pub_topic") == 0)
      create_image_publisher();
  }
}

void PipelineFilteringNode::image_callback(
    const sensor_msgs::msg::Image::SharedPtr msg) {
  RCLCPP_INFO_ONCE(this->get_logger(), "Received image message.");
  cv_bridge::CvImagePtr cv_ptr;

  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    if (cv_ptr->image.empty()) {
      RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                  "Empty image received, skipping processing.");
      return;
    }

  } catch (cv_bridge::Exception &e) {
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "cv_bridge exception: " << e.what());
    return;
  }

  cv::Mat input_image = cv_ptr->image;
  cv::Mat filtered_image;

  apply_filter(filter_, filter_params_, input_image, filtered_image);

  auto message = std::make_unique<sensor_msgs::msg::Image>();
  cv_bridge::CvImage(msg->header, "mono8", filtered_image).toImageMsg(*message);

  image_pub_->publish(std::move(message));
}

RCLCPP_COMPONENTS_REGISTER_NODE(
    vortex::pipeline_processing::PipelineFilteringNode)

} // namespace vortex::pipeline_processing
