#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "usv_interfaces/msg/zbbox_array.hpp"
#include "usv_interfaces/msg/zbbox.hpp"
#include "usv_interfaces/msg/object.hpp"
#include "usv_interfaces/msg/object_list.hpp"

#include <cmath>
#include <iomanip>
#include <sstream>
#include <vector>
#include <string>

using PointCloud2   = sensor_msgs::msg::PointCloud2;
using ZbboxArray    = usv_interfaces::msg::ZbboxArray;
using PointStamped  = geometry_msgs::msg::PointStamped;
using Object        = usv_interfaces::msg::Object;
using ObjectList    = usv_interfaces::msg::ObjectList;
using Marker        = visualization_msgs::msg::Marker;
using MarkerArray   = visualization_msgs::msg::MarkerArray;

// ── label helpers ─────────────────────────────────────────────────────────────

static void map_yolo_label(int label, Object & obj)
{
    switch (label) {
        case 0:  obj.color = 4; obj.type = "round";  break; // black_buoy
        case 1:  obj.color = 2; obj.type = "round";  break; // blue_buoy
        case 2:  obj.color = 4; obj.type = "marker"; break; // course_marker
        case 3:  obj.color = 1; obj.type = "round";  break; // green_buoy
        case 4:  obj.color = 0; obj.type = "marker"; break; // port_marker
        case 5:  obj.color = 0; obj.type = "round";  break; // red_buoy
        case 6:  obj.color = 1; obj.type = "marker"; break; // starboard_marker
        case 7:  obj.color = 3; obj.type = "round";  break; // yellow_marker
        default: obj.color =-1; obj.type = "ignore"; break;
    }
}

// color name for the marker label text
static std::string color_name(int8_t c)
{
    switch (c) {
        case 0:  return "red";
        case 1:  return "green";
        case 2:  return "blue";
        case 3:  return "yellow";
        case 4:  return "black";
        default: return "?";
    }
}

// RViz color per object color index  (r, g, b)
struct RGB { float r, g, b; };
static RGB rviz_color(int8_t c)
{
    switch (c) {
        case 0:  return {1.0f, 0.1f, 0.1f}; // red
        case 1:  return {0.1f, 0.9f, 0.1f}; // green
        case 2:  return {0.2f, 0.4f, 1.0f}; // blue
        case 3:  return {1.0f, 0.9f, 0.0f}; // yellow
        case 4:  return {0.2f, 0.2f, 0.2f}; // black
        default: return {0.6f, 0.6f, 0.6f}; // grey (ignore)
    }
}

static void pad_to(ObjectList & list, size_t size)
{
    while (list.obj_list.size() < size) {
        Object pad;
        pad.color = -1; pad.x = 0.0; pad.y = 0.0;
        pad.v_x = 0.0; pad.v_y = 0.0;
        pad.type = "ignore"; pad.uuid = "";
        list.obj_list.push_back(pad);
    }
}

// ─── Node ─────────────────────────────────────────────────────────────────────
class GzSimZedNode : public rclcpp::Node
{
public:
    GzSimZedNode()
    : Node("gz_sim_zed_node"),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_)
    {
        target_frame_  = declare_parameter<std::string>("target_frame",  "usv");
        sample_radius_ = declare_parameter<int>("sample_radius", 3);
        cloud_topic_   = declare_parameter<std::string>("cloud_topic",   "/velodyne_points");
        det_topic_     = declare_parameter<std::string>("det_topic",     "/yolo/detections");
        output_topic_  = declare_parameter<std::string>("output_topic",  "/bebblebrox/objects/yolo");

        cloud_sub_ = create_subscription<PointCloud2>(cloud_topic_, 10,
            [this](PointCloud2::ConstSharedPtr msg) { latest_cloud_ = msg; });

        det_sub_ = create_subscription<ZbboxArray>(det_topic_, 10,
            std::bind(&GzSimZedNode::det_callback, this, std::placeholders::_1));

        obj_pub_    = create_publisher<ObjectList>(output_topic_, 10);
        marker_pub_ = create_publisher<MarkerArray>("/gz_sim_zed/markers", 10);

        RCLCPP_INFO(get_logger(),
            "gz_sim_zed_node ready | '%s' + '%s' → '%s' (frame='%s')",
            cloud_topic_.c_str(), det_topic_.c_str(),
            output_topic_.c_str(), target_frame_.c_str());
    }

private:
    // ── helpers ───────────────────────────────────────────────────────────────
    static float median(std::vector<float> & v)
    {
        std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
        return v[v.size() / 2];
    }

    struct FieldOffsets { int x, y, z; bool valid; };

    FieldOffsets get_offsets(const PointCloud2 & cloud)
    {
        FieldOffsets o{-1, -1, -1, false};
        for (const auto & f : cloud.fields) {
            if (f.name == "x") o.x = static_cast<int>(f.offset);
            if (f.name == "y") o.y = static_cast<int>(f.offset);
            if (f.name == "z") o.z = static_cast<int>(f.offset);
        }
        o.valid = (o.x >= 0 && o.y >= 0 && o.z >= 0);
        return o;
    }

    // Build a TEXT_VIEW_FACING + SPHERE marker pair for one detection
    void make_markers(int id,
                      const Object & obj,
                      const geometry_msgs::msg::Point & pos_3d,
                      const std_msgs::msg::Header & header,
                      MarkerArray & out)
    {
        const auto col = rviz_color(obj.color);
        const float dist = std::sqrt(obj.x * obj.x + obj.y * obj.y);

        // ── sphere at 3-D position ──────────────────────────────────────────
        Marker sphere;
        sphere.header        = header;
        sphere.ns            = "gz_sim_zed_sphere";
        sphere.id            = id * 2;
        sphere.type          = Marker::SPHERE;
        sphere.action        = Marker::ADD;
        sphere.pose.position = pos_3d;
        sphere.pose.orientation.w = 1.0;
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.3;
        sphere.color.r = col.r; sphere.color.g = col.g;
        sphere.color.b = col.b; sphere.color.a = 0.85f;
        sphere.lifetime = rclcpp::Duration::from_seconds(0.5);
        out.markers.push_back(sphere);

        // ── text label floating above the sphere ───────────────────────────
        // format: "green round\n1.23 m"
        std::ostringstream label;
        label << color_name(obj.color) << " " << obj.type << "\n"
              << std::fixed << std::setprecision(2) << dist << " m";

        Marker text;
        text.header        = header;
        text.ns            = "gz_sim_zed_text";
        text.id            = id * 2 + 1;
        text.type          = Marker::TEXT_VIEW_FACING;
        text.action        = Marker::ADD;
        text.pose.position = pos_3d;
        text.pose.position.z += 0.4;   // float a bit above the sphere
        text.pose.orientation.w = 1.0;
        text.scale.z = 0.25;           // text height in metres
        text.color.r = col.r; text.color.g = col.g;
        text.color.b = col.b; text.color.a = 1.0f;
        text.text    = label.str();
        text.lifetime = rclcpp::Duration::from_seconds(0.5);
        out.markers.push_back(text);
    }

    // ── detection callback ────────────────────────────────────────────────────
    void det_callback(const ZbboxArray::ConstSharedPtr & dets)
    {
        ObjectList  obj_list;
        MarkerArray marker_array;

        if (!latest_cloud_) {
            RCLCPP_WARN_ONCE(get_logger(), "No cloud received yet — waiting...");
            pad_to(obj_list, 10);
            obj_pub_->publish(obj_list);
            return;
        }

        const uint32_t W          = latest_cloud_->width;
        const uint32_t H          = latest_cloud_->height;
        const uint32_t point_step = latest_cloud_->point_step;
        const uint8_t* data       = latest_cloud_->data.data();

        if (H <= 1) {
            RCLCPP_WARN_ONCE(get_logger(),
                "PointCloud2 is unorganized (height=%u) — pixel indexing unavailable", H);
            pad_to(obj_list, 10);
            obj_pub_->publish(obj_list);
            return;
        }

        const auto off = get_offsets(*latest_cloud_);
        if (!off.valid) {
            RCLCPP_WARN_ONCE(get_logger(), "PointCloud2 missing x/y/z fields");
            pad_to(obj_list, 10);
            obj_pub_->publish(obj_list);
            return;
        }

        int marker_id = 0;

        for (const auto & box : dets->boxes)
        {
            const int u_c = (box.x0 + box.x1) / 2;
            const int v_c = (box.y0 + box.y1) / 2;

            std::vector<float> xs, ys, zs;
            const int cap = (2 * sample_radius_ + 1) * (2 * sample_radius_ + 1);
            xs.reserve(cap); ys.reserve(cap); zs.reserve(cap);

            for (int dv = -sample_radius_; dv <= sample_radius_; ++dv) {
                for (int du = -sample_radius_; du <= sample_radius_; ++du) {
                    const int u = u_c + du;
                    const int v = v_c + dv;
                    if (u < 0 || v < 0 || u >= (int)W || v >= (int)H) continue;

                    const uint8_t* p =
                        data + (static_cast<size_t>(v) * W + static_cast<size_t>(u)) * point_step;

                    float x, y, z;
                    std::memcpy(&x, p + off.x, sizeof(float));
                    std::memcpy(&y, p + off.y, sizeof(float));
                    std::memcpy(&z, p + off.z, sizeof(float));

                    if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
                        xs.push_back(x); ys.push_back(y); zs.push_back(z);
                    }
                }
            }

            Object obj;

            if (xs.empty()) {
                RCLCPP_WARN(get_logger(),
                    "No valid depth at (%d,%d) label=%d — marking ignore", u_c, v_c, box.label);
                obj.color = -1; obj.x = 0.0; obj.y = 0.0;
                obj.v_x = 0.0; obj.v_y = 0.0;
                obj.type = "ignore"; obj.uuid = box.uuid;
                obj_list.obj_list.push_back(obj);
                continue;
            }

            PointStamped pt_cam;
            pt_cam.header       = latest_cloud_->header;
            pt_cam.point.x      = median(xs);
            pt_cam.point.y      = median(ys);
            pt_cam.point.z      = median(zs);

            try {
                PointStamped pt_base = tf_buffer_.transform(pt_cam, target_frame_);

                map_yolo_label(box.label, obj);
                obj.x   = pt_base.point.x;  // ← pt_base, not pt_cam
                obj.y   = pt_base.point.y;  // ← pt_base, not pt_cam
                obj.v_x = 0.0;
                obj.v_y = 0.0;
                obj.uuid = box.uuid;

                std_msgs::msg::Header marker_header;
                marker_header.stamp    = latest_cloud_->header.stamp;
                marker_header.frame_id = target_frame_;
                make_markers(marker_id++, obj, pt_base.point, marker_header, marker_array);

                RCLCPP_INFO(get_logger(),
                    "[%s %s | prob=%.2f] → x=%.3f  y=%.3f  dist=%.3f m",
                    color_name(obj.color).c_str(), obj.type.c_str(), box.prob,
                    obj.x, obj.y, std::sqrt(obj.x * obj.x + obj.y * obj.y));
            }
            catch (const tf2::TransformException & ex) {
                RCLCPP_WARN(get_logger(), "TF failed uuid=%s: %s", box.uuid.c_str(), ex.what());
                obj.color = -1; obj.x = 0.0; obj.y = 0.0;
                obj.v_x = 0.0; obj.v_y = 0.0;
                obj.type = "ignore"; obj.uuid = box.uuid;
            }

            obj_list.obj_list.push_back(obj);
        }

        pad_to(obj_list, 10);
        obj_pub_->publish(obj_list);

        if (!marker_array.markers.empty())
            marker_pub_->publish(marker_array);
    }

    // ── members ───────────────────────────────────────────────────────────────
    tf2_ros::Buffer            tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    PointCloud2::ConstSharedPtr                  latest_cloud_;
    rclcpp::Subscription<PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<ZbboxArray>::SharedPtr  det_sub_;
    rclcpp::Publisher<ObjectList>::SharedPtr     obj_pub_;
    rclcpp::Publisher<MarkerArray>::SharedPtr    marker_pub_;

    std::string target_frame_, cloud_topic_, det_topic_, output_topic_;
    int         sample_radius_;
};

// ─── Main ─────────────────────────────────────────────────────────────────────
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GzSimZedNode>());
    rclcpp::shutdown();
    return 0;
}