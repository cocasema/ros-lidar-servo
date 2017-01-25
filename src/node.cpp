/*******************************************************************************
 * MIT License
 *
 * Copyright (c) 2016 cocasema
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 ******************************************************************************/

#include "lidar_lite/driver.h"
#include "pca9685/driver.h"

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <boost/lexical_cast.hpp>
#include <boost/scope_exit.hpp>

#include <cmath>
#include <chrono>
#include <thread>

namespace lidar_servo {

} // namespace lidar_servo

int
main(int argc, char **argv)
{
  using namespace std::chrono_literals;
  using lidar_lite::LidarLiteDriver;
  using pca9685::PCA9685Driver;

  ros::init(argc, argv, "lidar_servo");

  /*
  if (ros::console::set_logger_level(ROSCONSOLE_ROOT_LOGGER_NAME, ros::console::levels::Debug)) {
     ros::console::notifyLoggerLevelsChanged();
  }
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
     ros::console::notifyLoggerLevelsChanged();
  }
  */

  std::string frame_id = "lidar_servo";

  ros::NodeHandle nh;

  try {
    ros::NodeHandle nh_("~");
    nh_.param("frame_id", frame_id, frame_id);

    ROS_INFO("Read params: {frame_id: %s}",
             frame_id.c_str());
  }
  catch (boost::bad_lexical_cast const& ex) {
    ROS_ERROR("Failed to read params: %s", ex.what());
    return 1;
  }

  struct lidar_lite_params
  {
    int32_t i2c_bus = LidarLiteDriver::DEFAULT_I2C_BUS;
    uint8_t i2c_address = LidarLiteDriver::DEFAULT_I2C_ADDR;
  };

  lidar_lite_params lidar_lite_params;
  try {
    ros::NodeHandle nh_("~/lidar_lite");

    nh_.param("i2c_bus", lidar_lite_params.i2c_bus, lidar_lite_params.i2c_bus);

    std::string i2c_address_str = std::to_string(lidar_lite_params.i2c_address);
    nh_.param("i2c_address", i2c_address_str, i2c_address_str);
    lidar_lite_params.i2c_address = (uint8_t)std::stoul(i2c_address_str, nullptr, 0);

    ROS_INFO("Read lidar_lite params: {i2c_bus: %i, i2c_address: 0x%0x}",
             lidar_lite_params.i2c_bus, lidar_lite_params.i2c_address);
  }
  catch (boost::bad_lexical_cast const& ex) {
    ROS_ERROR("Failed to read lidar_lite params: %s", ex.what());
    return 2;
  }

  LidarLiteDriver lidar_lite(lidar_lite_params.i2c_bus, lidar_lite_params.i2c_address);
  lidar_lite.configure(LidarLiteDriver::OperationMode::DEFAULT);

  struct pca9685_params
  {
    int32_t i2c_bus = PCA9685Driver::DEFAULT_I2C_BUS;
    uint8_t i2c_address = PCA9685Driver::DEFAULT_I2C_ADDR;
    int32_t frequency = PCA9685Driver::DEFAULT_FREQUENCY;
    int32_t dc_min = 150;
    int32_t dc_max = 450;
    int32_t dc_step = 1;
    int32_t deg_min = -90;
    int32_t deg_max = +90;
    int32_t pin = 0;
    int32_t delay = 10;
  };

  pca9685_params pca9685_params;
  try {
    ros::NodeHandle nh_("~/pca9685");

    nh_.param("i2c_bus", pca9685_params.i2c_bus, pca9685_params.i2c_bus);

    std::string i2c_address_str = std::to_string(pca9685_params.i2c_address);
    nh_.param("i2c_address", i2c_address_str, i2c_address_str);
    pca9685_params.i2c_address = (uint8_t)std::stoul(i2c_address_str, nullptr, 0);

    nh_.param("frequency", pca9685_params.frequency, pca9685_params.frequency);

    nh_.param("dc_min", pca9685_params.dc_min, pca9685_params.dc_min);
    nh_.param("dc_max", pca9685_params.dc_max, pca9685_params.dc_max);
    nh_.param("dc_step", pca9685_params.dc_step, pca9685_params.dc_step);

    nh_.param("deg_min", pca9685_params.deg_min, pca9685_params.deg_min);
    nh_.param("deg_max", pca9685_params.deg_max, pca9685_params.deg_max);

    nh_.param("pin", pca9685_params.pin, pca9685_params.pin);

    nh_.param("delay", pca9685_params.delay, pca9685_params.delay);

    ROS_INFO("Read pca9685 params: {i2c_bus: %i, i2c_address: 0x%0x, frequency: %i"
             ", dc_min: %i, dc_max: %i, pin: %i}",
             pca9685_params.i2c_bus, pca9685_params.i2c_address, pca9685_params.frequency,
             pca9685_params.dc_min, pca9685_params.dc_max, pca9685_params.pin);
  }
  catch (boost::bad_lexical_cast const& ex) {
    ROS_ERROR("Failed to read pca9685 params: %s", ex.what());
    return 3;
  }

  PCA9685Driver pca9685(pca9685_params.i2c_bus, pca9685_params.i2c_address);
  if (pca9685_params.frequency != PCA9685Driver::DEFAULT_FREQUENCY) {
    pca9685.set_frequency(pca9685_params.frequency);
  }

  ros::Publisher publisher = nh.advertise<sensor_msgs::LaserScan>("scan", 1024);

  pca9685.set_duty_cycle(pca9685_params.pin, PCA9685Driver::value_t(pca9685_params.dc_min));

  std::this_thread::sleep_for(1s);

  int num_steps = pca9685_params.dc_max - pca9685_params.dc_min;

  sensor_msgs::LaserScan msg;
  msg.header.frame_id = frame_id;

  msg.angle_min = M_PI * (pca9685_params.deg_min / 180.);
  msg.angle_max = M_PI * (pca9685_params.deg_max / 180.);
  msg.angle_increment = (msg.angle_max - msg.angle_min) / num_steps;

  msg.range_min = std::numeric_limits<float>::max();
  msg.range_max = 0.;
  msg.ranges.resize(num_steps);
  msg.intensities.resize(num_steps);

  auto t_start= std::chrono::high_resolution_clock::now();

  int step = 0;
  int dir = +1;

  while (ros::ok()) {
    float position = (float)step / num_steps;
    float angle = msg.angle_min + (msg.angle_max - msg.angle_min) * position;
    ROS_DEBUG("step: %i/%i, angle: %f rad", step, num_steps, angle);

    float dc = pca9685_params.dc_min + (pca9685_params.dc_max - pca9685_params.dc_min) * position;
    PCA9685Driver::value_t value(dc);
    pca9685.set_duty_cycle(pca9685_params.pin, value);
    std::this_thread::sleep_for(std::chrono::milliseconds(pca9685_params.delay));

    auto distance = lidar_lite.distance(false);

    float distance_in_meters = distance->value / 100.;

    msg.range_min = std::min(msg.range_min, distance_in_meters);
    msg.range_max = std::max(msg.range_max, distance_in_meters);

    msg.ranges[step] = distance_in_meters;
    msg.intensities[step] = distance->value;

    step += dir;
    if (0 == step || step == num_steps-1) {
      {
        msg.header.seq++;
        msg.header.stamp = ros::Time::now();

        auto t_end = std::chrono::high_resolution_clock::now();
        msg.scan_time = (float)std::chrono::duration_cast<std::chrono::seconds>(t_end - t_start).count();
        msg.time_increment = msg.scan_time / num_steps;

        publisher.publish(msg);
      }

      dir *= -1;

      t_start = std::chrono::high_resolution_clock::now();

      msg.range_min = std::numeric_limits<float>::max();
      msg.range_max = 0.;
    }
  }

  return 0;
}
