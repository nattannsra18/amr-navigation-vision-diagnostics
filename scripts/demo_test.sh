#!/usr/bin/env bash

# Interactive end-to-end demo test for the AMR project.
# Start navigation.launch.py in another terminal before running this script.

WORKSPACE="${HOME}/amr-navigation-vision-diagnostics"

GREEN="\033[0;32m"
RED="\033[0;31m"
YELLOW="\033[1;33m"
CYAN="\033[0;36m"
BOLD="\033[1m"
RESET="\033[0m"

PASS_COUNT=0
FAIL_COUNT=0

section() {
    printf "\n%b\n" "${CYAN}${BOLD}============================================================${RESET}"
    printf "%b\n" "${CYAN}${BOLD}$1${RESET}"
    printf "%b\n" "${CYAN}${BOLD}============================================================${RESET}"
}

pass() {
    printf "%b %s\n" "${GREEN}[PASS]${RESET}" "$1"
    PASS_COUNT=$((PASS_COUNT + 1))
}

fail() {
    printf "%b %s\n" "${RED}[FAIL]${RESET}" "$1"
    FAIL_COUNT=$((FAIL_COUNT + 1))
}

info() {
    printf "%b %s\n" "${YELLOW}[INFO]${RESET}" "$1"
}

wait_for_user() {
    printf "\n%b\n" "${YELLOW}$1${RESET}"
    read -rp "Press Enter to continue..."
}

measure_rate() {
    local topic="$1"
    local duration="${2:-4}"
    local output
    local rate

    output="$(timeout "$duration" ros2 topic hz "$topic" 2>&1 || true)"
    rate="$(
        printf "%s\n" "$output" |
            awk '/average rate:/ {value=$3} END {print value}'
    )"

    if [[ -n "$rate" ]]; then
        pass "$topic is publishing at approximately $rate Hz"
    else
        fail "$topic produced no measurable rate"
    fi
}

check_frame() {
    local topic="$1"
    local expected_frame="$2"
    local output
    local frame

    output="$(
        timeout 6 ros2 topic echo "$topic" \
            --once --field header 2>&1 || true
    )"
    frame="$(
        printf "%s\n" "$output" |
            awk '/frame_id:/ {print $2; exit}'
    )"

    if [[ "$frame" == "$expected_frame" ]]; then
        pass "$topic frame_id is $frame"
    elif [[ -n "$frame" ]]; then
        fail "$topic frame_id is $frame; expected $expected_frame"
    else
        fail "Unable to read frame_id from $topic"
    fi
}

check_tf() {
    local target_frame="$1"
    local source_frame="$2"
    local duration="${3:-5}"
    local output

    output="$(
        timeout "$duration" ros2 run tf2_ros tf2_echo \
            "$target_frame" "$source_frame" 2>&1 || true
    )"

    if printf "%s\n" "$output" | grep -q "Translation:"; then
        pass "TF $target_frame -> $source_frame is available"
        printf "%s\n" "$output" |
            grep -m1 -A1 "Translation:" || true
    else
        fail "TF $target_frame -> $source_frame is unavailable"
    fi
}

get_lifecycle_state() {
    local node="$1"
    ros2 lifecycle get "/$node" 2>&1 || true
}

check_lifecycle_nodes() {
    local node
    local state

    for node in "$@"; do
        state="$(get_lifecycle_state "$node")"

        if [[ "$state" == *"active [3]"* ]]; then
            pass "/$node is active [3]"
        else
            fail "/$node state is: $state"
        fi
    done
}

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
    printf "%b\n" "${RED}ROS 2 Jazzy setup file was not found.${RESET}"
    exit 1
fi

if [[ ! -f "$WORKSPACE/install/setup.bash" ]]; then
    printf "%b\n" "${RED}Workspace setup file was not found.${RESET}"
    printf "Build the workspace before running this script.\n"
    exit 1
fi

# ROS setup scripts are not compatible with Bash nounset mode (set -u).
set +u
source /opt/ros/jazzy/setup.bash
source "$WORKSPACE/install/setup.bash"

clear

section "AMR NAVIGATION, VISION & DIAGNOSTICS"
printf "Automated End-to-End Demo Test\n"
printf "ROS 2 Jazzy | Gazebo Harmonic | Navigation2 | OpenCV\n"

section "1. ROS 2 SYSTEM DISCOVERY"

if ros2 node list | grep -qx "/amcl"; then
    pass "ROS 2 navigation system is running"
else
    fail "/amcl was not found; navigation.launch.py may not be running"
fi

if ros2 node list | grep -qx "/aruco_detector"; then
    pass "ArUco detector node is running"
else
    fail "/aruco_detector was not found"
fi

if ros2 node list | grep -qx "/amr_system_monitor"; then
    pass "AMR system monitor node is running"
else
    fail "/amr_system_monitor was not found"
fi

wait_for_user \
"RViz: Set the robot position with 2D Pose Estimate.
Click the robot location and drag the arrow in its forward direction.
If the initial pose is already set, press Enter now."

section "2. AMCL LOCALIZATION"

AMCL_OUTPUT="$(
    timeout 10 ros2 topic echo /amcl_pose --once 2>&1 || true
)"

if printf "%s\n" "$AMCL_OUTPUT" | grep -q "frame_id: map"; then
    pass "AMCL published a pose in the map frame"
    printf "%s\n" "$AMCL_OUTPUT" |
        grep -m1 "frame_id:" || true
else
    fail "No map-frame pose was received from /amcl_pose"
fi

check_tf "map" "base_link" 6

section "3. NAV2 LIFECYCLE"

NAVIGATION_NODES=(
    planner_server
    controller_server
    bt_navigator
    behavior_server
    smoother_server
    waypoint_follower
    velocity_smoother
    collision_monitor
    route_server
)

NAVIGATION_READY=true

for node in "${NAVIGATION_NODES[@]}"; do
    state="$(get_lifecycle_state "$node")"

    if [[ "$state" != *"active [3]"* ]]; then
        NAVIGATION_READY=false
        break
    fi
done

if [[ "$NAVIGATION_READY" == "false" ]]; then
    info "Some navigation nodes are not active."
    info "Resetting and starting the navigation lifecycle manager..."

    RESET_OUTPUT="$(
        timeout 30 ros2 service call \
            /lifecycle_manager_navigation/manage_nodes \
            nav2_msgs/srv/ManageLifecycleNodes \
            "{command: 3}" 2>&1 || true
    )"

    if printf "%s\n" "$RESET_OUTPUT" | grep -q "success=True"; then
        info "Navigation lifecycle reset completed"
    else
        info "Reset response did not report success=True"
    fi

    STARTUP_OUTPUT="$(
        timeout 60 ros2 service call \
            /lifecycle_manager_navigation/manage_nodes \
            nav2_msgs/srv/ManageLifecycleNodes \
            "{command: 0}" 2>&1 || true
    )"

    if printf "%s\n" "$STARTUP_OUTPUT" | grep -q "success=True"; then
        info "Navigation lifecycle startup completed"
    else
        info "Startup response did not report success=True"
    fi

    sleep 3
fi

check_lifecycle_nodes \
    amcl \
    map_server \
    "${NAVIGATION_NODES[@]}"

section "4. SENSOR STREAMS"

measure_rate "/clock" 3
measure_rate "/scan" 4
measure_rate "/odom" 4
measure_rate "/camera/color/image_raw" 4
measure_rate "/camera/depth/image_raw" 4
measure_rate "/aruco/debug_image" 4

section "5. SENSOR FRAME IDs"

check_frame "/scan" "base_scan"
check_frame "/camera/color/image_raw" "camera_rgb_optical_frame"
check_frame "/camera/depth/image_raw" "camera_depth_frame"

section "6. AUTONOMOUS NAVIGATION"

wait_for_user \
"RViz: Send a Nav2 Goal to a free location.
Choose a goal far enough away for the robot to move for at least 10 seconds.
Press Enter here immediately after the robot starts moving."

measure_rate "/cmd_vel" 8
check_tf "map" "base_link" 5

section "7. ARUCO MARKER DETECTION"

wait_for_user \
"Point the RGB camera toward ArUco Marker ID 0.
Open /aruco/debug_image.
Do not press Enter until the green marker boundary,
coordinate axes, and ID 0 are visible."

info "Waiting up to 30 seconds for ArUco Marker ID 0..."

MARKER_OUTPUT="$(
    timeout 30 ros2 topic echo \
        /aruco/marker_ids \
        --once \
        --filter "0 in m.data" \
        2>&1 || true
)"

if printf "%s\n" "$MARKER_OUTPUT" |
    grep -Eq '^[[:space:]]*-[[:space:]]+0$'; then
    pass "ArUco marker ID 0 was detected"
else
    fail "ArUco marker ID 0 was not detected within 30 seconds"
fi

info "Waiting for a non-empty ArUco pose..."

POSE_OUTPUT="$(
    timeout 15 ros2 topic echo \
        /aruco/poses \
        --once \
        --filter "len(m.poses) > 0" \
        2>&1 || true
)"

if printf "%s\n" "$POSE_OUTPUT" | grep -q "position:"; then
    pass "ArUco marker pose was published"
    printf "%s\n" "$POSE_OUTPUT" |
        grep -m1 -A4 "position:" || true
else
    fail "No ArUco marker pose was received within 15 seconds"
fi

check_tf "map" "aruco_marker_0" 8

info "Waiting for the diagnostic monitor to update..."
sleep 2

section "8. AMR DIAGNOSTICS"

DIAGNOSTICS_OUTPUT="$(
    timeout 10 ros2 topic echo /diagnostics --once \
        --filter \
        "any(s.name.startswith('AMR/') for s in m.status)" \
        2>&1 || true
)"

EXPECTED_DIAGNOSTICS=(
    "AMR/LiDAR"
    "AMR/Odometry"
    "AMR/VelocityCommand"
    "AMR/Camera"
    "AMR/ArUco"
)

for diagnostic in "${EXPECTED_DIAGNOSTICS[@]}"; do
    if printf "%s\n" "$DIAGNOSTICS_OUTPUT" |
        grep -q "name: $diagnostic"; then
        pass "$diagnostic status is available"
    else
        fail "$diagnostic status was not found"
    fi
done

printf "\nDiagnostic messages:\n"
printf "%s\n" "$DIAGNOSTICS_OUTPUT" |
    grep -E "name: AMR/|message:" || true

section "9. END-TO-END TEST SUMMARY"

printf "%b\n" "${GREEN}Passed tests: $PASS_COUNT${RESET}"
printf "%b\n" "${RED}Failed tests: $FAIL_COUNT${RESET}"

if [[ "$FAIL_COUNT" -eq 0 ]]; then
    printf "\n%b\n" "${GREEN}${BOLD}ALL AMR DEMO TESTS PASSED${RESET}"
    printf "\nNavigation: PASS\n"
    printf "Localization and TF: PASS\n"
    printf "RGB-D Sensors: PASS\n"
    printf "ArUco Detection: PASS\n"
    printf "System Diagnostics: PASS\n"
else
    printf "\n%b\n" "${RED}${BOLD}ONE OR MORE DEMO TESTS FAILED${RESET}"
fi

printf "\nRepository:\n"
printf "https://github.com/nattannsra18/"
printf "amr-navigation-vision-diagnostics\n"

exit "$FAIL_COUNT"
