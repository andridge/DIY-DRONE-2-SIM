#include <geometry_msgs/Twist.h>
#include "sensor_msgs/JointState.h"
#include <math.h>
#include <mutex>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>

// ====================== YOUR ORIGINAL CLASSES — 100% UNTOUCHED
// ======================
class KalmanFilter {
public:
  KalmanFilter()
      : q_angle(0.001), q_bias(0.003), r_measure(0.03), angle(0.0), bias(0.0) {
    P[0][0] = P[0][1] = P[1][0] = P[1][1] = 0.0;
  }
  double update(double newAngle, double newRate, double dt) {
    angle += dt * (newRate - bias);
    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += q_bias * dt;
    double S = P[0][0] + r_measure;
    double K0 = P[0][0] / S;
    double K1 = P[1][0] / S;
    double y = newAngle - angle;
    angle += K0 * y;
    bias += K1 * y;
    double P00 = P[0][0], P01 = P[0][1], P10 = P[1][0], P11 = P[1][1];
    P[0][0] = P00 - K0 * P00;
    P[0][1] = P01 - K0 * P01;
    P[1][0] = P10 - K1 * P00;
    P[1][1] = P11 - K1 * P01;
    return angle;
  }
  void setNoise(double q_ang, double q_b, double r_meas) {
    q_angle = q_ang;
    q_bias = q_b;
    r_measure = r_meas;
  }

private:
  double q_angle, q_bias, r_measure;
  double angle, bias;
  double P[2][2];
};

class PIDController {
public:
  PIDController(double kp = 0.0, double ki = 0.0, double kd = 0.0,
                double i_limit = 1.0)
      : Kp(kp), Ki(ki), Kd(kd), integral(0.0), prev_error(0.0),
        integral_limit(i_limit) {}
  double update(double error, double dt) {
    if (dt <= 0.0)
      return 0.0;
    integral += error * dt;
    if (integral > integral_limit)
      integral = integral_limit;
    if (integral < -integral_limit)
      integral = -integral_limit;
    double derivative = (error - prev_error) / dt;
    double out = (Kp * error) + (Ki * integral) + (Kd * derivative);
    prev_error = error;
    return out;
  }
  void reset() {
    integral = 0.0;
    prev_error = 0.0;
  }

private:
  double Kp, Ki, Kd, integral, prev_error, integral_limit;
};

class IMUProcessor {
public:
  IMUProcessor() : have_data(false) {}
  void init(ros::NodeHandle &nh,
            const std::string &topic = "/august/imu/data") {
    imu_sub = nh.subscribe(topic, 50, &IMUProcessor::imuCb, this);
    last_time = ros::Time::now();
  }
  bool getAngles(double &roll_deg, double &pitch_deg) {
    std::lock_guard<std::mutex> lock(mtx);
    if (!have_data)
      return false;
    roll_deg = roll_kal;
    pitch_deg = pitch_kal;
    return true;
  }
  void setKalmanNoise(double q_ang, double q_bias, double r_meas) {
    std::lock_guard<std::mutex> lock(mtx);
    kf_roll.setNoise(q_ang, q_bias, r_meas);
    kf_pitch.setNoise(q_ang, q_bias, r_meas);
  }

private:
  ros::Subscriber imu_sub;
  KalmanFilter kf_roll, kf_pitch;
  double roll_kal = 0.0, pitch_kal = 0.0;
  bool have_data = false;
  ros::Time last_time;
  std::mutex mtx;
  void imuCb(const sensor_msgs::Imu::ConstPtr &msg) {
    ros::Time now = msg->header.stamp;
    if (now.toSec() == 0.0)
      now = ros::Time::now();
    double dt = (now - last_time).toSec();
    if (dt <= 0.0 || dt > 0.5)
      dt = 1.0 / 50.0;
    last_time = now;
    double ax = msg->linear_acceleration.x;
    double ay = msg->linear_acceleration.y;
    double az = msg->linear_acceleration.z;
    double gx = msg->angular_velocity.x * 180.0 / M_PI;
    double gy = msg->angular_velocity.y * 180.0 / M_PI;
    double accel_roll = atan2(ay, az) * 180.0 / M_PI;
    double accel_pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / M_PI;
    std::lock_guard<std::mutex> lock(mtx);
    roll_kal = kf_roll.update(accel_roll, gx, dt);
    pitch_kal = kf_pitch.update(accel_pitch, gy, dt);
    have_data = true;
  }
};

class DroneMixer {
public:
  static constexpr double KP = 3.55;
  static constexpr double KI = 0.005;
  static constexpr double KD = 2.05;

  DroneMixer()
      : pid_pitch(KP, KI, KD, 50.0), pid_roll(KP, KI, KD, 50.0),
        attitude_gain(1.0), thrust_gain(1.0), max_motor(100.0) {}

  void init(ros::NodeHandle &nh) {
    nh_ = nh;
    std::string imu_topic;
    nh_.param("imu_topic", imu_topic, std::string("/august/imu/data"));
    imu_processor.init(nh_, imu_topic);
    double q_ang = 0.001, q_bias = 0.003, r_meas = 0.03;
    nh_.param("imu_q_angle", q_ang, q_ang);
    nh_.param("imu_q_bias", q_bias, q_bias);
    nh_.param("imu_r_meas", r_meas, r_meas);
    imu_processor.setKalmanNoise(q_ang, q_bias, r_meas);
    nh_.param("gain/attitude", attitude_gain, 1.0);
    nh_.param("gain/thrust", thrust_gain, 1.0);
    nh_.param("max_motor", max_motor, 100.0);
    last_loop = ros::Time::now();
  }

  void setDesiredFromTwist(const geometry_msgs::Twist::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(mtx);
    desired_thrust = msg->linear.z * thrust_gain;
    desired_pitch_cmd = msg->linear.x;
    desired_roll_cmd = msg->linear.y;
    desired_yaw_cmd = msg->angular.z;
    last_cmd_time = ros::Time::now();
  }

  void getCorrections(double &pitch_corr, double &roll_corr) {
    double dt = (ros::Time::now() - last_loop).toSec();
    if (dt <= 0.0 || dt > 0.5)
      dt = 1.0 / 50.0;
    last_loop = ros::Time::now();
    double measured_roll = 0.0, measured_pitch = 0.0;
    bool have_imu = imu_processor.getAngles(measured_roll, measured_pitch);
    std::lock_guard<std::mutex> lock(mtx);
    double pitch_error = desired_pitch_cmd - (have_imu ? measured_pitch : 0.0);
    double roll_error = desired_roll_cmd - (have_imu ? measured_roll : 0.0);
    pitch_corr = have_imu ? pid_pitch.update(pitch_error, dt) : 0.0;
    roll_corr = have_imu ? pid_roll.update(roll_error, dt) : 0.0;
  }

  double clampMotor(double v) {
    if (v < -max_motor)
      return -max_motor;
    if (v > max_motor)
      return max_motor;
    return v;
  }

private:
  ros::NodeHandle nh_;
  IMUProcessor imu_processor;
  PIDController pid_pitch, pid_roll;
  double attitude_gain, thrust_gain, max_motor;
  double desired_thrust = 0, desired_pitch_cmd = 0, desired_roll_cmd = 0,
         desired_yaw_cmd = 0;
  ros::Time last_cmd_time, last_loop;
  std::mutex mtx;
};

ros::Publisher pub47, pub48, pub49, pub50;

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg,
                    DroneMixer *mixer) {
  if (!mixer)
    return;
  mixer->setDesiredFromTwist(msg);
  double pitch_corr = 0.0, roll_corr = 0.0;
  mixer->getCorrections(pitch_corr, roll_corr);
  double pitch_combined = msg->linear.x + pitch_corr;
  double roll_combined = msg->linear.y + roll_corr;
  std_msgs::Float64 cmd47, cmd48, cmd49, cmd50;
  cmd47.data = mixer->clampMotor(msg->linear.z + pitch_combined -
                                 roll_combined + msg->angular.z);
  cmd48.data = mixer->clampMotor(msg->linear.z + pitch_combined +
                                 roll_combined - msg->angular.z);
  cmd49.data = mixer->clampMotor(msg->linear.z - pitch_combined +
                                 roll_combined + msg->angular.z);
  cmd50.data = mixer->clampMotor(msg->linear.z - pitch_combined -
                                 roll_combined - msg->angular.z);
  pub47.publish(cmd47);
  pub48.publish(cmd48);
  pub49.publish(cmd49);
  pub50.publish(cmd50);
  ROS_INFO("Motor cmds -> 47: %.2f, 48: %.2f, 49: %.2f, 50: %.2f (corr p:%.3f "
           "r:%.3f)",
           cmd47.data, cmd48.data, cmd49.data, cmd50.data, pitch_corr,
           roll_corr);
}

std::vector<double> joint_positions(4, 0.0);

void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  for (size_t i = 0; i < msg->name.size(); ++i)
  {
    if (msg->name[i] == "revolute_47_joint") joint_positions[0] = msg->position[i];
    else if (msg->name[i] == "revolute_48_joint") joint_positions[1] = msg->position[i];
    else if (msg->name[i] == "revolute_49_joint") joint_positions[2] = msg->position[i];
    else if (msg->name[i] == "revolute_50_joint") joint_positions[3] = msg->position[i];
  }
  ROS_INFO("Joint positions: [%.3f, %.3f, %.3f, %.3f]",
           joint_positions[0], joint_positions[1], joint_positions[2], joint_positions[3]);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "drone_mixer");
  ros::NodeHandle nh("~");

  pub47 = nh.advertise<std_msgs::Float64>(
      "/august/august_controller/revolute_47_effort_controller/command", 10);
  pub48 = nh.advertise<std_msgs::Float64>(
      "/august/august_controller/revolute_48_effort_controller/command", 10);
  pub49 = nh.advertise<std_msgs::Float64>(
      "/august/august_controller/revolute_49_effort_controller/command", 10);
  pub50 = nh.advertise<std_msgs::Float64>(
      "/august/august_controller/revolute_50_effort_controller/command", 10);

  DroneMixer mixer;
  mixer.init(nh);

  ros::Subscriber cmd_sub = nh.subscribe<geometry_msgs::Twist>(
      "/cmd_vel", 10, boost::bind(cmdVelCallback, _1, &mixer));

  ros::Subscriber joints_sub =
      nh.subscribe<sensor_msgs::JointState>("/joint_states", 10, jointStateCallback);

  ROS_INFO("DRONE_MIXER ACTIVE – AI /cmd_vel → motor efforts");
  ros::spin();

  return 0;
}