cmake_minimum_required(VERSION 3.8)
project(smart_labeling_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# --- ADD THIS LINE ---
# This makes the headers from sensor_msgs available to our code generator.
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ProductInfo.msg"
  "msg/ProcessStatus.msg"
  "msg/ValidationOutcome.msg"
  "srv/PredictLabelQuality.srv"
  # --- ADD THIS DEPENDENCY HERE ---
  DEPENDENCIES sensor_msgs
)

ament_package()