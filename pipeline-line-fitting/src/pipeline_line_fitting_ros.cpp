#include <pipeline_line_fitting/pipeline_line_fitting_ros.hpp>

using std::placeholders::_1;

RandsacParams PipelineLineFittingNode::fetchParams() {
    this->declare_parameter("n", 5);
    this->declare_parameter("k", 500);
    this->declare_parameter("t", 50.0);
    this->declare_parameter("fracOfPoints", 0.001);
    this->declare_parameter("removeT", 1000.0);
    this->declare_parameter("finalScorethresh", 65.0);
    this->declare_parameter("minTurnAngle", 1.5);
    this->declare_parameter("size", 200);
    RandsacParams params;
    params.n = this->get_parameter("n").as_int();
    params.k = this->get_parameter("k").as_int();
    params.t = this->get_parameter("t").as_double();
    params.fracOfPoints = this->get_parameter("fracOfPoints").as_double();
    params.removeT = this->get_parameter("removeT").as_double();
    params.finalScorethresh =
        this->get_parameter("finalScorethresh").as_double();
    params.minTurnAngle = this->get_parameter("minTurnAngle").as_double();
    params.size = this->get_parameter("size").as_int();
    return params;
}

PipelineLineFittingNode::PipelineLineFittingNode(
    const rclcpp::NodeOptions& options)
    : Node("pipeline_line_fitting_node", options) {
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(1))
                           .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    auto imageSubTopic = this->declare_parameter<std::string>(
        "image_sub_topic", "/cam_down/image_color");
    auto image_visualization_pub_topic = this->declare_parameter<std::string>(
        "image_visualization_pub_topic", "/linefitting/visualization");
    auto pose_array_pub_topic = this->declare_parameter<std::string>(
        "pose_array_pub_topic", "/linefitting/pose_array");

    imageSub_ = this->create_subscription<sensor_msgs::msg::Image>(
        imageSubTopic, qos_profile,
        std::bind(&PipelineLineFittingNode::imageCallback, this, _1));

    publishVisualization_ =
        this->declare_parameter("publish_visualization", true);
    if (publishVisualization_) {
        imageVisualizationPub_ =
            this->create_publisher<sensor_msgs::msg::Image>(
                image_visualization_pub_topic, qos_profile);
    }
    poseArrayPub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
        pose_array_pub_topic, qos_profile);

    pipeline_ = LinedetectorPipe(fetchParams());
}

geometry_msgs::msg::Pose getPose(const cv::Point& point) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = point.x;
    pose.position.y = point.y;
    pose.position.z = 0.0;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;
    return pose;
}

void PipelineLineFittingNode::imageCallback(
    const sensor_msgs::msg::Image::SharedPtr msg) {
    cv::Mat img;
    try {
        img = cv_bridge::toCvCopy(msg, "mono8")->image;
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    std::vector<Line> lines = pipeline_.detect(img, 2);
    auto message = geometry_msgs::msg::PoseArray();
    message.header = msg->header;

    for (const auto& line : lines) {
        message.poses.push_back(getPose(line.start));
        message.poses.push_back(getPose(line.end));
    }

    poseArrayPub_->publish(message);

    if (publishVisualization_) {
        auto img_color = drawLines(img, lines);
        auto output_image =
            cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img_color)
                .toImageMsg();
        output_image->header = msg->header;
        imageVisualizationPub_->publish(*output_image);
    }
}

cv::Mat PipelineLineFittingNode::drawLines(cv::Mat& image,
                                           const std::vector<Line>& lines) {
    cv::Mat img_color;
    // pipeline_.preprocess(image);
    cv::cvtColor(image, img_color, cv::COLOR_GRAY2BGR);

    cv::Mat img_with_lines = img_color.clone();

    for (const auto& line : lines) {
        cv::Point start(line.start.x, line.start.y);
        cv::Point end(line.end.x, line.end.y);
        cv::line(img_color, start, end, cv::Scalar(255, 0, 255), 10);
    }
    cv::addWeighted(img_with_lines, 0.5, img_color, 0.5, 0, img_color);

    return img_color;
}

RCLCPP_COMPONENTS_REGISTER_NODE(PipelineLineFittingNode)
