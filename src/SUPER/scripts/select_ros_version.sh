#!/bin/bash
# Global configuration

ENABLE_BACKUP=false  # 默认启用备份

# Function to generate ROS package files with standardized paths
# Parameters:
#   $1 - Package name (e.g., "super_planner")
#   $2 - ROS version ("ROS1" or "ROS2")
generate_standard_ros_package() {
  local package_name="$1"
  local ros_version="$2"

  # Auto-generate paths
  local template_dir="$SCRIPT_DIR/../${package_name}/ros"
  local root_dir="$SCRIPT_DIR/../${package_name}"

  # Validate parameters
  if [[ $# -ne 2 ]]; then
    echo "Error: Invalid arguments for generate_standard_ros_package"
    echo "Usage: generate_standard_ros_package <package_name> <ROS1|ROS2>"
    return 1
  fi

  # Convert ROS version
  local normalized_ros=$(echo "$ros_version" | tr '[:upper:]' '[:lower:]')

  # Path validation
  if [ ! -d "$template_dir" ]; then
    echo "Error: Missing template directory for $package_name (expected: $template_dir)"
    return 1
  fi

  if [ ! -d "$root_dir" ]; then
    echo "Error: Missing root directory for $package_name (expected: $root_dir)"
    return 1
  fi

  # File selection
  local cmake_source=""
  local package_source=""

  case "$normalized_ros" in
    "ros1")
      cmake_source="$template_dir/ros1.CMakeLists.txt"
      package_source="$template_dir/ros1.package.xml"
      ;;
    "ros2")
      cmake_source="$template_dir/ros2.CMakeLists.txt"
      package_source="$template_dir/ros2.package.xml"
      ;;
    *)
      echo "Error: Invalid ROS version for $package_name. Must be ROS1 or ROS2"
      return 1
      ;;
  esac

  # Template validation
  local missing_files=()
  [ ! -f "$cmake_source" ] && missing_files+=("$cmake_source")
  [ ! -f "$package_source" ] && missing_files+=("$package_source")

  if [ ${#missing_files[@]} -gt 0 ]; then
    echo "Error: Missing templates for $package_name:"
    printf '  %s\n' "${missing_files[@]}"
    return 1
  fi

  # Safe file deployment
  echo "Updating $package_name ($ros_version)..."
  (
    set -e  # Enable error trapping
    cd "$root_dir" || exit 1

 # Backup logic
    if $ENABLE_BACKUP; then
      echo "Creating backups..."
      for f in CMakeLists.txt package.xml; do
        [ -f "$f" ] && cp -v "$f" "$f.bak-$(date +%Y%m%d%H%M%S)"
      done
    fi

    # Deploy new files
    cp -v "$cmake_source" CMakeLists.txt
    cp -v "$package_source" package.xml
  )

  if [ $? -eq 0 ]; then
    echo "Successfully updated $package_name for $ros_version!"
    echo "---------------------------------------------------"
  else
    echo "Failed to update $package_name!"
    echo "---------------------------------------------------"
    return 1
  fi
}

# 获取脚本绝对路径
SCRIPT_DIR=$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" &>/dev/null && pwd -P)


# Check input arguments
if [ $# -ne 1 ]; then
  echo "Usage: $0 [ROS1|ROS2]"
  exit 1
fi
ROS_VERSION=$(echo "$1" | tr '[:upper:]' '[:lower:]')  # Normalize input to lowercase

# 批量处理配置 (只需指定包名和ROS版本)
declare -a packages=(
  "super_planner:$ROS_VERSION"
  "rog_map:$ROS_VERSION"
  "mars_uav_sim/mars_quadrotor_msgs:$ROS_VERSION"
  "mars_uav_sim/marsim_render:$ROS_VERSION"
  "mars_uav_sim/perfect_drone_sim:$ROS_VERSION"
  "mission_planner:$ROS_VERSION"
)

# 执行处理
for pkg in "${packages[@]}"; do
  IFS=':' read -ra params <<< "$pkg"
  generate_standard_ros_package "${params[0]}" "${params[1]}"
done