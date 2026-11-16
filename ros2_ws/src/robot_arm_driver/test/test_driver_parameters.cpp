#include <algorithm>

#include <gtest/gtest.h>
#include <rclcpp/parameter_map.hpp>

TEST(DriverParameters, DriverConfigurationHasTypedCalibrationParameters)
{
  // Use ROS's parser: a generic YAML parser accepts [] even though ROS cannot
  // infer its parameter type and rejects it before the node declares defaults.
  const auto parameters = rclcpp::parameter_map_from_yaml_file(ARM_DRIVER_PARAMETERS_FILE);
  const auto driver = parameters.find("/robot_arm_driver");
  ASSERT_NE(driver, parameters.end());
  const auto & values = driver->second;
  const auto joint_names = std::find_if(values.begin(), values.end(), [](const auto & parameter) {
    return parameter.get_name() == "joint_names";
  });
  ASSERT_NE(joint_names, values.end());
  ASSERT_EQ(joint_names->get_type(), rclcpp::ParameterType::PARAMETER_STRING_ARRAY);
  const auto joint_count = joint_names->as_string_array().size();
  ASSERT_GT(joint_count, 0U);

  const auto zero_ticks = std::find_if(values.begin(), values.end(), [](const auto & parameter) {
    return parameter.get_name() == "zero_ticks";
  });
  ASSERT_NE(zero_ticks, values.end());
  ASSERT_EQ(zero_ticks->get_type(), rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
  EXPECT_EQ(zero_ticks->as_double_array().size(), joint_count);

  // Omit this optional override to use zero_ticks. If explicitly configured,
  // it must be a complete, typed model-zero calibration.
  const auto model_zero = std::find_if(values.begin(), values.end(), [](const auto & parameter) {
    return parameter.get_name() == "urdf_zero_ticks";
  });
  if (model_zero != values.end()) {
    ASSERT_EQ(model_zero->get_type(), rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
    EXPECT_EQ(model_zero->as_double_array().size(), joint_count);
  }
}
