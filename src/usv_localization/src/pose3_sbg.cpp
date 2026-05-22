#include <rclcpp/rclcpp.hpp>
#include <sbg_driver/msg/sbg_imu_data.hpp>
#include <usv_interfaces/msg/pose3.hpp>
#include <cmath>

class pose3_sbg : public rclcpp::Node
{
public:
    pose3_sbg() : Node("pose3_sbg")
    {
        imu_sub_ = this->create_subscription<sbg_driver::msg::SbgImuData>(
            "/sbg/imu_data", 10,
            std::bind(&pose3_sbg::imu_callback, this, std::placeholders::_1));

        pose_pub_ = this->create_publisher<usv_interfaces::msg::Pose3>("usv/imu/pose3", 10);

        last_time_ = this->now().nanoseconds;
        RCLCPP_INFO(this->get_logger(), "IMU Pose3 node iniciado");
    }

private:
    void imu_callback(const sbg_driver::msg::SbgImuData::SharedPtr msg)
    {
        rclcpp::Time current_time = this->now().nanoseconds();
        double dt = (current_time - last_time_).nanoseconds();
        last_time_ = current_time;

        if (dt <= 0.0 || dt > 1.0) return;

        // Leer gyro [rad/s]
        double gyro_x = msg->gyro.x;
        double gyro_y = msg->gyro.y;
        double gyro_z = msg->gyro.z;

        // Leer accel [m/s²]
        double accel_x = msg->accel.x;
        double accel_y = msg->accel.y;
        double accel_z = msg->accel.z;

        //orientación: integrar gyro
        roll_  += gyro_x * dt;
        pitch_ += gyro_y * dt;
        yaw_   += gyro_z * dt;

        //remover gravedad e integrar accel
        // Matriz de rotación R = Ry(pitch) · Rx(roll)
        double R[3][3] = {
            { cos(pitch_),  sin(pitch_)*sin(roll_),  sin(pitch_)*cos(roll_) },
            { 0.0,          cos(roll_),             -sin(roll_)             },
            {-sin(pitch_),  cos(pitch_)*sin(roll_),  cos(pitch_)*cos(roll_) }
        };

        // g en frame mundo NED = [0, 0, 9.81]
        double g[3] = {0.0, 0.0, 9.81};

        // g_body = R · g_world
        double g_body[3] = {0.0, 0.0, 0.0};
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                g_body[i] += R[i][j] * g[j];
            }
        }

        vx_ += (accel_x - g_body[0]) * dt;
        vy_ += (accel_y - g_body[1]) * dt;
        vz_ += (accel_z - g_body[2]) * dt;

        //integrar velocidad
        x_ += vx_ * dt;
        y_ += vy_ * dt;
        z_ += vz_ * dt;

        // Publicar
        auto pose_msg = usv_interfaces::msg::Pose3();
        pose_msg.roll  = roll_;
        pose_msg.pitch = pitch_;
        pose_msg.yaw   = yaw_;
        pose_msg.vx    = vx_;
        pose_msg.vy    = vy_;
        pose_msg.vz    = vz_;
        pose_msg.x     = x_;
        pose_msg.y     = y_;
        pose_msg.z     = z_;
        pose_pub_->publish(pose_msg);
    }

    rclcpp::Subscription<sbg_driver::msg::SbgImuData>::SharedPtr imu_sub_;
    rclcpp::Publisher<usv_interfaces::msg::Pose3>::SharedPtr pose_pub_;
    rclcpp::Time last_time_;

    double roll_  = 0.0;
    double pitch_ = 0.0;
    double yaw_   = 0.0;

    double vx_ = 0.0;
    double vy_ = 0.0;
    double vz_ = 0.0;

    double x_ = 0.0;
    double y_ = 0.0;
    double z_ = 0.0;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<pose3_sbg>());
    rclcpp::shutdown();
    return 0;
}