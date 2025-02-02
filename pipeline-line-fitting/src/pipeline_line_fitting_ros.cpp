#include <pipeline_line_fitting/pipeline_line_fitting_ros.hpp>

using std::placeholders::_1;

PipelineLineFittingNode::PipelineLineFittingNode(
    const rclcpp::NodeOptions& options)
    : Node("pipeline_line_fitting_node", options) {
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(1))
                           .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    auto image_sub_topic = this->declare_parameter<std::string>(
        "image_sub_topic", "/cam_down/image_color");
    auto image_visualization_pub_topic = this->declare_parameter<std::string>(
        "image_visualization_pub_topic", "/linefitting/visualization");
    auto pose_array_pub_topic = this->declare_parameter<std::string>(
        "pose_array_pub_topic", "/linefitting/pose_array");

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        image_sub_topic, qos_profile,
        std::bind(&PipelineLineFittingNode::image_callback, this, _1));

    publish_visualization_ =
        this->declare_parameter("publish_visualization", true);
    if (publish_visualization_) {
        image_visualization_pub_ =
            this->create_publisher<sensor_msgs::msg::Image>(
                image_visualization_pub_topic, qos_profile);
    }
    pose_array_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
        pose_array_pub_topic, qos_profile);

    pipeline = LinedetectorPipe();
}

void PipelineLineFittingNode::image_callback(
    const sensor_msgs::msg::Image::SharedPtr msg) {
    cv::Mat img;
    try {
        img = cv_bridge::toCvCopy(msg, "mono8")->image;
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    vector<Line> lines = pipeline(img, 2);
    auto message = geometry_msgs::msg::PoseArray();
    message.header = msg->header;

    for (const auto& line : lines) {
        geometry_msgs::msg::Pose start_pose;
        start_pose.position.x = line.start.x;
        start_pose.position.y = line.start.y;
        start_pose.position.z = 0.0;
        start_pose.orientation.x = 0.0;
        start_pose.orientation.y = 0.0;
        start_pose.orientation.z = 0.0;
        start_pose.orientation.w = 1.0;
        message.poses.push_back(start_pose);

        geometry_msgs::msg::Pose end_pose;
        end_pose.position.x = line.end.x;
        end_pose.position.y = line.end.y;
        end_pose.position.z = 0.0;
        end_pose.orientation.x = 0.0;
        end_pose.orientation.y = 0.0;
        end_pose.orientation.z = 0.0;
        end_pose.orientation.w = 1.0;
        message.poses.push_back(end_pose);
    }

    pose_array_pub_->publish(message);

    if (publish_visualization_) {
        auto img_color = draw_lines(img, lines);
        auto output_image =
            cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img_color)
                .toImageMsg();
        output_image->header = msg->header;
        image_visualization_pub_->publish(*output_image);
    }
}

cv::Mat PipelineLineFittingNode::draw_lines(cv::Mat& image,
                                            const vector<Line>& lines) {
    cv::Mat img_color;
    // integrate with the size parameter
    // pipeline._preprocess(image);
    cv::cvtColor(image, img_color, cv::COLOR_GRAY2BGR);

    cv::Mat img_with_lines = img_color.clone();

    for (const auto& line : lines) {
        cv::Point start(line.start.x, line.start.y);
        cv::Point end(line.end.x, line.end.y);
        cv::line(img_color, start, end, cv::Scalar(255, 0, 255), 4);
    }
    cv::addWeighted(img_with_lines, 0.5, img_color, 0.5, 0, img_color);

    return img_color;
}

RCLCPP_COMPONENTS_REGISTER_NODE(PipelineLineFittingNode)
