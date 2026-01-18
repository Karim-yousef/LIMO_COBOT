#!/usr/bin/env python3
"""
TWO-STAGE APPROACH with Depth Camera + EKF Integration

Features:
1. Synchronized RGB + Depth camera with blind spot handling
2. BLIND_APPROACH state for camera dead zone (< 0.2m)
3. Camera intrinsics for accurate 3D projection
4. Kalman filter for position smoothing
5. Depth-based distance estimation
6. Debug visualization

States:
1. SEARCH - Explore with forward + wide rotation, looking for TABLE or CUBE
2. APPROACH - When TABLE or CUBE detected, approach it (uses depth when available)
3. APPROACH_CUBE - When close (LIDAR < 0.5m), focus on BLUE CUBE only
4. BLIND_APPROACH - Camera blind spot, use dead reckoning with last known position
5. DONE - Stop and trigger arm pick
"""

import rospy
import cv2
import numpy as np
import random
from sensor_msgs.msg import Image, LaserScan, CameraInfo
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32MultiArray, String
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import message_filters
import math
from tf.transformations import euler_from_quaternion


class SimpleKalmanFilter:
    """Simple 1D Kalman filter for smoothing noisy measurements."""
    def __init__(self, process_noise=0.01, measurement_noise=0.1):
        self.x = None  # State estimate
        self.P = 1.0   # Estimate uncertainty
        self.Q = process_noise      # Process noise
        self.R = measurement_noise  # Measurement noise
        
    def update(self, measurement):
        if self.x is None:
            self.x = measurement
            return self.x
            
        # Prediction (assume constant model)
        self.P = self.P + self.Q
        
        # Update
        K = self.P / (self.P + self.R)  # Kalman gain
        self.x = self.x + K * (measurement - self.x)
        self.P = (1 - K) * self.P
        
        return self.x
        
    def reset(self):
        self.x = None
        self.P = 1.0


class CoverageMap:
    """
    Grid-based coverage tracking to avoid revisiting explored areas.
    Tracks robot position and marks cells as visited.
    Uses finer grid and smarter direction selection.
    """
    def __init__(self, grid_size=0.3, map_size=20.0):
        """
        Args:
            grid_size: Size of each grid cell in meters (0.3m = 30cm cells for finer tracking)
            map_size: Total map size in meters (20m x 20m area)
        """
        self.grid_size = grid_size
        self.map_size = map_size
        self.grid_dim = int(map_size / grid_size)
        self.half_dim = self.grid_dim // 2
        
        # Coverage grid: 0 = unexplored, 1+ = visit count
        self.grid = np.zeros((self.grid_dim, self.grid_dim), dtype=np.int32)
        
        # Robot pose (updated from odometry)
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        
        # Track recent positions to detect loops
        self.position_history = []
        self.HISTORY_SIZE = 50
        
        rospy.loginfo(f"Coverage map initialized: {self.grid_dim}x{self.grid_dim} grid ({grid_size}m cells)")
        
    def world_to_grid(self, x, y):
        """Convert world coordinates to grid indices."""
        gx = int(x / self.grid_size) + self.half_dim
        gy = int(y / self.grid_size) + self.half_dim
        # Clamp to grid bounds
        gx = max(0, min(self.grid_dim - 1, gx))
        gy = max(0, min(self.grid_dim - 1, gy))
        return gx, gy
        
    def mark_visited(self, x=None, y=None):
        """Mark current or specified position as visited."""
        if x is None:
            x = self.robot_x
        if y is None:
            y = self.robot_y
        gx, gy = self.world_to_grid(x, y)
        self.grid[gx, gy] += 1
        
        # Track position history for loop detection
        self.position_history.append((x, y))
        if len(self.position_history) > self.HISTORY_SIZE:
            self.position_history.pop(0)
        
    def get_visit_count(self, x=None, y=None):
        """Get how many times a position was visited."""
        if x is None:
            x = self.robot_x
        if y is None:
            y = self.robot_y
        gx, gy = self.world_to_grid(x, y)
        return self.grid[gx, gy]
        
    def is_explored(self, x=None, y=None):
        """Check if position has been visited."""
        return self.get_visit_count(x, y) > 0
    
    def detect_loop(self):
        """Detect if robot is stuck in a loop by checking position history."""
        if len(self.position_history) < 20:
            return False
        
        # Check if recent position is close to older positions (loop detected)
        current = self.position_history[-1]
        for i, pos in enumerate(self.position_history[:-15]):  # Check against older positions
            dist = math.sqrt((current[0] - pos[0])**2 + (current[1] - pos[1])**2)
            if dist < 0.5:  # Within 0.5m of a position from >15 steps ago = loop
                return True
        return False
        
    def get_unexplored_direction(self, front_dist, left_dist, right_dist):
        """
        Determine best direction to explore based on coverage and obstacles.
        Checks 8 directions for exploration.
        Returns: angle offset in radians (0 = forward, positive = left, negative = right)
        """
        best_score = -999
        best_angle = 0
        
        # Check 8 directions: 0°, ±45°, ±90°, ±135°, 180°
        angles = [0, math.pi/4, -math.pi/4, math.pi/2, -math.pi/2, 
                  3*math.pi/4, -3*math.pi/4, math.pi]
        
        # Get obstacle distances for each direction (approximate)
        dist_map = {
            0: front_dist,
            math.pi/4: min(front_dist, left_dist) * 0.8,
            -math.pi/4: min(front_dist, right_dist) * 0.8,
            math.pi/2: left_dist,
            -math.pi/2: right_dist,
            3*math.pi/4: left_dist * 0.7,
            -3*math.pi/4: right_dist * 0.7,
            math.pi: 1.5  # Assume some space behind
        }
        
        for angle in angles:
            # Position 1.5m in this direction
            check_dist = min(1.5, dist_map.get(angle, 1.0) - 0.3)
            if check_dist < 0.3:
                continue  # Can't go this way
                
            px = self.robot_x + check_dist * math.cos(self.robot_yaw + angle)
            py = self.robot_y + check_dist * math.sin(self.robot_yaw + angle)
            
            visits = self.get_visit_count(px, py)
            clearance = dist_map.get(angle, 0.5)
            
            # Score = unexplored bonus + clearance bonus - visit penalty
            score = (10 - visits * 3) + clearance * 2
            
            # Penalize going back (angle near ±180°)
            if abs(angle) > 2.5:
                score -= 5
                
            if score > best_score:
                best_score = score
                best_angle = angle
        
        # Convert to simple direction: 0=forward, 1=left, -1=right
        if abs(best_angle) < math.pi/6:  # Within 30° of forward
            return 0
        elif best_angle > 0:
            return 1  # Turn left
        else:
            return -1  # Turn right
            
    def get_coverage_percent(self):
        """Get percentage of map that has been explored."""
        explored = np.sum(self.grid > 0)
        total = self.grid_dim * self.grid_dim
        return (explored / total) * 100
        
    def update_pose(self, x, y, yaw):
        """Update robot pose and mark current position."""
        self.robot_x = x
        self.robot_y = y
        self.robot_yaw = yaw
        self.mark_visited()


class EnhancedTwoStageApproach:
    def __init__(self):
        rospy.init_node('blue_cube_approach_enhanced', anonymous=True)
        
        rospy.loginfo("="*60)
        rospy.loginfo("ENHANCED TWO-STAGE APPROACH")
        rospy.loginfo("  + Synchronized RGB + Depth camera")
        rospy.loginfo("  + Depth blind spot handling (< 0.2m)")
        rospy.loginfo("  + BLIND_APPROACH state for dead reckoning")
        rospy.loginfo("  + Kalman filtered position estimates")
        rospy.loginfo("  + Camera intrinsics for 3D projection")
        rospy.loginfo("="*60)
        rospy.loginfo("STAGES:")
        rospy.loginfo("  1. SEARCH: Forward + 360° rotation")
        rospy.loginfo("  2. APPROACH: Go toward TABLE or CUBE")
        rospy.loginfo("  3. APPROACH_CUBE: Precise approach to BLUE cube")
        rospy.loginfo("  4. BLIND_APPROACH: Dead reckoning in camera blind zone")
        rospy.loginfo("  5. DONE: Trigger arm pick")
        rospy.loginfo("="*60)
        
        # Wait for sensors with longer timeouts
        try:
            rospy.wait_for_message('/limo/scan', LaserScan, timeout=60.0)
            rospy.loginfo("✓ LIDAR OK!")
        except rospy.ROSException:
            rospy.logwarn("✗ LIDAR timeout - check /limo/scan topic!")
            
        try:
            rospy.wait_for_message('/limo/color/image_raw', Image, timeout=60.0)
            rospy.loginfo("✓ RGB Camera OK!")
        except rospy.ROSException:
            rospy.logwarn("✗ RGB Camera timeout - check /limo/color/image_raw topic!")
            
        try:
            rospy.wait_for_message('/limo/depth/image_raw', Image, timeout=30.0)
            rospy.loginfo("✓ Depth Camera OK!")
            self.depth_available = True
        except rospy.ROSException:
            rospy.logwarn("✗ Depth Camera timeout - will use area-based estimation")
            self.depth_available = False
            
        try:
            rospy.wait_for_message('/limo/color/camera_info', CameraInfo, timeout=30.0)
            rospy.loginfo("✓ Camera Info OK!")
        except rospy.ROSException:
            rospy.logwarn("✗ Camera Info timeout - using defaults")
        
        self.bridge = CvBridge()
        
        # Display control - disable on headless robots
        self.enable_display = rospy.get_param('~enable_display', True)
        
        # Display control - disable on headless robots
        self.enable_display = rospy.get_param('~enable_display', True)
        self.IMAGE_WIDTH = 640
        self.IMAGE_HEIGHT = 480
        self.IMAGE_CENTER_X = 320
        
        # Camera intrinsics (will be updated from CameraInfo)
        self.fx = 554.254  # Default focal length
        self.fy = 554.254
        self.cx = 320.5    # Principal point
        self.cy = 240.5
        self.camera_info_received = False
        
        # Main state machine
        self.state = "IDLE"  # Start in IDLE, wait for mission controller trigger
        self.mission_active = False  # Only publish cmd_vel when our phase is active
        self.emergency_stop = False  # Emergency stop flag
        self.search_state = "FORWARD"
        
        # Search timing - BOUNDARY-AWARE EXPLORATION WITH 360° SCANS
        self.state_start_time = rospy.Time.now()
        self.turn_direction = 1          # 1=left, -1=right
        self.forward_time = 5.0          # Forward segment time (longer!)
        self.forward_increment = 0.5     # Small increments
        self.max_forward_time = 8.0      # Max forward time
        self.turn_count = 0              # Count turns
        self.forward_count = 0           # Count forward segments for 360° scan trigger
        self.SCAN_360_EVERY = 3          # Do full 360° scan every N forward moves
        self.current_turn_angle = 90     # Variable turn angle to avoid rectangular patterns
        
        # ============================================
        # COVERAGE TRACKING - avoid revisiting areas
        # ============================================
        self.coverage_map = CoverageMap(grid_size=0.3, map_size=20.0)  # Finer grid (30cm)
        rospy.loginfo("Coverage tracking ENABLED - robot will prioritize unexplored areas")
        
        # LIDAR thresholds
        self.OBSTACLE_FRONT = 0.6
        self.OBSTACLE_SIDE = 0.4
        self.CLEAR_SPACE = 1.2
        
        # ============================================
        # DEPTH CAMERA PARAMETERS
        # ============================================
        self.DEPTH_MIN_VALID = 0.20      # Minimum valid depth (blind spot below this)
        self.DEPTH_MAX_VALID = 4.0       # Maximum valid depth
        self.DEPTH_BLIND_THRESHOLD = 0.25  # When to switch to BLIND_APPROACH
        self.depth_confidence = 0.0      # How confident we are in depth reading
        
        # Kalman filters for smoothing
        self.kf_depth = SimpleKalmanFilter(process_noise=0.001, measurement_noise=0.05)
        self.kf_y_offset = SimpleKalmanFilter(process_noise=0.001, measurement_noise=0.02)
        self.kf_z_offset = SimpleKalmanFilter(process_noise=0.001, measurement_noise=0.02)
        
        # Last known good position (for blind approach)
        self.last_valid_depth = None
        self.last_valid_y_offset = 0.0
        self.last_valid_z_offset = 0.0
        self.last_valid_time = None
        
        # Blind approach parameters
        self.blind_approach_start_time = None
        self.BLIND_APPROACH_DURATION = 1.8  # seconds of dead reckoning
        self.BLIND_APPROACH_SPEED = 0.05    # Very slow creep forward
        
        # DONE state - store final grasp position for arm
        self.final_grasp_distance = 0.12  # Default close distance
        self.final_grasp_y_offset = 0.0
        self.final_grasp_z_offset = 0.0
        
        # Arm pick coordination
        self.arm_pick_acknowledged = False
        self.last_trigger_time = None
        self.TRIGGER_INTERVAL = 2.0  # Re-send trigger every 2 seconds until ack
        
        # ============================================
        # TABLE DETECTION - PURPLE/MAGENTA (unique - nothing else is purple!)
        # ============================================
        self.table_detected = False
        self.table_cx = 320
        self.table_area = 0
        # PURPLE/MAGENTA in HSV: Hue 140-170 (far from orange/brown/blue)
        self.TABLE_LOW = np.array([140, 80, 80])    # Purple/Magenta
        self.TABLE_HIGH = np.array([170, 255, 255])
        self.TABLE_MIN_AREA = 150  # Lower threshold for far detection
        self.TABLE_MIN_ASPECT = 0.5  # Width/Height ratio (table is wide)
        self.TABLE_MAX_ASPECT = 3.0  # Not too extreme
        
        # ============================================
        # BLUE CUBE DETECTION
        # ============================================
        self.cube_detected = False
        self.cube_cx = 320
        self.cube_cy = 240
        self.cube_area = 0
        self.cube_depth = None  # Real depth from depth camera
        self.BLUE_LOW = np.array([85, 80, 40])
        self.BLUE_HIGH = np.array([145, 255, 255])
        self.CUBE_MIN_AREA = 100
        
        # Stage transition thresholds
        self.SWITCH_TO_CUBE_LIDAR = 0.55
        self.CUBE_CLOSE_AREA = 15000  # Larger = closer (cube appears bigger)
        # Robot must get very close to the table edge for the arm to reach the cube.
        # Tunable via param to match your world geometry.
        self.CUBE_STOP_LIDAR = rospy.get_param('~cube_stop_lidar', 0.06)
        
        # Detection confirmation
        self.detect_count = 0
        self.CONFIRM_DETECTIONS = 2
        self.no_detection_count = 0
        
        # LIDAR data
        self.front_dist = 10.0
        self.front_left_dist = 10.0
        self.front_right_dist = 10.0
        self.left_dist = 10.0
        self.right_dist = 10.0
        
        # ============================================
        # OBSTACLE AVOIDANCE (Bug-like algorithm)
        # ============================================
        self.OBSTACLE_DIST = 0.45        # Distance to trigger avoidance
        self.OBSTACLE_CLEAR_DIST = 0.60  # Distance to resume normal navigation
        self.avoiding_obstacle = False
        self.avoidance_direction = 1     # 1 = left, -1 = right
        self.avoidance_start_time = None
        self.target_before_avoidance = None  # Remember where we were going
        self.AVOIDANCE_TIMEOUT = 8.0     # Max time to spend avoiding
        
        # Depth image storage
        self.latest_depth_image = None
        self.depth_timestamp = None
        
        # Publishers
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.trigger_pub = rospy.Publisher('/arm_pick/trigger', Bool, queue_size=1)
        self.cube_pos_pub = rospy.Publisher('/cube_position', Float32MultiArray, queue_size=1)
        self.annotated_pub = rospy.Publisher('/blue_cube/annotated', Image, queue_size=1)
        self.state_pub = rospy.Publisher('/blue_cube/state', String, queue_size=1)
        self.status_pub = rospy.Publisher('/blue_cube_approach/status', String, queue_size=1, latch=True)
        
        # Subscribers
        # Camera info for intrinsics
        rospy.Subscriber('/limo/color/camera_info', CameraInfo, self.camera_info_cb, queue_size=1)
        
        # LIDAR
        rospy.Subscriber('/limo/scan', LaserScan, self.scan_cb, queue_size=1)
        
        # Arm status feedback
        rospy.Subscriber('/arm_pick/status', String, self.arm_status_cb, queue_size=1)
        
        # Mission controller trigger - starts the search
        rospy.Subscriber('/blue_cube_approach/trigger', Bool, self._trigger_cb, queue_size=1)
        
        # Emergency stop subscriber
        rospy.Subscriber('/emergency_stop', Bool, self._emergency_stop_cb, queue_size=1)
        
        # Odometry for coverage tracking
        rospy.Subscriber('/odom', Odometry, self.odom_cb, queue_size=1)
        
        # Synchronized RGB + Depth
        if self.depth_available:
            rgb_sub = message_filters.Subscriber('/limo/color/image_raw', Image)
            depth_sub = message_filters.Subscriber('/limo/depth/image_raw', Image)
            
            self.sync = message_filters.ApproximateTimeSynchronizer(
                [rgb_sub, depth_sub], queue_size=5, slop=0.1
            )
            self.sync.registerCallback(self.synced_image_cb)
            rospy.loginfo("Using synchronized RGB + Depth")
        else:
            # Fallback to RGB only
            rospy.Subscriber('/limo/color/image_raw', Image, self.rgb_only_cb, queue_size=1)
            rospy.loginfo("Using RGB only (no depth)")
        
        # Display frame storage (for GUI in main loop - prevents freezing)
        self.display_frame = None
        
        # ============================================
        # PUBLISH INITIAL IDLE STATUS
        # This lets mission_controller know we're ready IMMEDIATELY
        # (not just when run() loop starts)
        # ============================================
        rospy.sleep(0.5)  # Brief delay to ensure publisher is registered
        self.status_pub.publish(String(data="IDLE"))
        rospy.loginfo("="*60)
        rospy.loginfo("BLUE CUBE NODE READY - Published IDLE status")
        rospy.loginfo("Waiting for trigger from /blue_cube_approach/trigger...")
        rospy.loginfo("="*60)
        
    def camera_info_cb(self, msg):
        """Update camera intrinsics from CameraInfo message."""
        if not self.camera_info_received:
            self.fx = msg.K[0]
            self.fy = msg.K[4]
            self.cx = msg.K[2]
            self.cy = msg.K[5]
            self.camera_info_received = True
            rospy.loginfo(f"Camera intrinsics: fx={self.fx:.1f}, fy={self.fy:.1f}, cx={self.cx:.1f}, cy={self.cy:.1f}")
    
    def arm_status_cb(self, msg):
        """Handle arm pick status feedback."""
        status = msg.data
        rospy.loginfo(f"Arm status received: {status}")
        
        # Mark as acknowledged so we stop re-triggering
        self.arm_pick_acknowledged = True
        
        if status == "SUCCESS":
            rospy.loginfo("="*60)
            rospy.loginfo("ARM PICK SUCCESSFUL! Ready for next mission.")
            rospy.loginfo("="*60)
            # Stop DONE-loop spam; keep robot stopped.
            self.state = "IDLE"
        elif status == "CUBE_HELD":
            rospy.loginfo("="*60)
            rospy.loginfo("CUBE PICKED! Basket search will take over...")
            rospy.loginfo("="*60)
            # Set to IDLE so we stop cmd_vel and wait for mission controller
            
            
            self.state = "IDLE"  # cmd_vel arbitration is handled by mission_active flag
        elif status.startswith("FAILED"):
            rospy.logwarn(f"Arm pick failed: {status}")
            self.state = "IDLE"
        
    def scan_cb(self, msg):
        """Process LIDAR scan data."""
        ranges = list(msg.ranges)
        n = len(ranges)
        if n == 0:
            return
            
        def get_min(start_deg, end_deg):
            start_idx = int((start_deg + 180) * n / 360) % n
            end_idx = int((end_deg + 180) * n / 360) % n
            if start_idx <= end_idx:
                indices = range(start_idx, end_idx)
            else:
                indices = list(range(start_idx, n)) + list(range(0, end_idx))
            vals = [ranges[i] for i in indices 
                   if not math.isinf(ranges[i]) and not math.isnan(ranges[i]) and ranges[i] > 0.05]
            return min(vals) if vals else 10.0
        
        self.front_dist = get_min(-20, 20)
        self.front_left_dist = get_min(20, 50)
        self.front_right_dist = get_min(-50, -20)
        self.left_dist = get_min(50, 100)
        self.right_dist = get_min(-100, -50)
        
    def odom_cb(self, msg):
        """Update coverage map with robot position from odometry."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Extract yaw from quaternion
        orientation = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([
            orientation.x, orientation.y, orientation.z, orientation.w
        ])
        
        # Update coverage map
        self.coverage_map.update_pose(x, y, yaw)
        
    def get_depth_at_point(self, depth_image, px, py, window=5):
        """
        Get depth value at pixel (px, py) with median filtering.
        Handles depth camera blind spot by returning None for invalid depths.
        """
        if depth_image is None:
            return None, 0.0
            
        h, w = depth_image.shape[:2]
        
        # Bounds check
        x1 = max(0, px - window)
        x2 = min(w, px + window)
        y1 = max(0, py - window)
        y2 = min(h, py + window)
        
        roi = depth_image[y1:y2, x1:x2].copy()
        
        # Handle different depth formats
        if depth_image.dtype == np.uint16:
            roi = roi.astype(np.float32) / 1000.0  # mm to meters
        
        # Filter valid depths (handle blind spot)
        valid_mask = (roi > self.DEPTH_MIN_VALID) & (roi < self.DEPTH_MAX_VALID) & np.isfinite(roi)
        valid = roi[valid_mask]
        
        if len(valid) == 0:
            return None, 0.0
            
        # Confidence based on number of valid pixels
        confidence = len(valid) / roi.size
        
        return float(np.median(valid)), confidence
        
    def compute_3d_position(self, px, py, depth):
        """
        Compute 3D position in camera frame using camera intrinsics.
        Returns (x, y, z) where:
          x = right (camera frame)
          y = down (camera frame)  
          z = forward (camera frame / depth)
        """
        if depth is None or depth < self.DEPTH_MIN_VALID:
            return None
            
        # 3D position in camera optical frame
        x = (px - self.cx) * depth / self.fx
        y = (py - self.cy) * depth / self.fy
        z = depth
        
        return np.array([x, y, z])
        
    def detect_table(self, frame):
        """Detect CYAN table by color AND rectangular shape."""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.TABLE_LOW, self.TABLE_HIGH)
        
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        self.table_detected = False
        self.table_area = 0
        
        if contours:
            # Check all contours for rectangular shape
            for contour in sorted(contours, key=cv2.contourArea, reverse=True):
                area = cv2.contourArea(contour)
                
                if area < self.TABLE_MIN_AREA:
                    continue
                    
                # Check if shape is RECTANGULAR (4 corners)
                peri = cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, 0.04 * peri, True)
                
                # Must have 4-6 vertices (rectangle-ish)
                if len(approx) < 4 or len(approx) > 6:
                    continue
                
                # Check aspect ratio (table should be wider than tall from robot view)
                x, y, w, h = cv2.boundingRect(contour)
                aspect_ratio = float(w) / float(h) if h > 0 else 0
                
                if aspect_ratio < self.TABLE_MIN_ASPECT or aspect_ratio > self.TABLE_MAX_ASPECT:
                    continue
                
                # Check rectangularity (area vs bounding box)
                rect_area = w * h
                rectangularity = area / rect_area if rect_area > 0 else 0
                
                if rectangularity < 0.6:  # Must fill at least 60% of bounding box
                    continue
                
                # Passed all checks - this is the table!
                M = cv2.moments(contour)
                if M["m00"] > 0:
                    self.table_cx = int(M["m10"] / M["m00"])
                    self.table_area = area
                    self.table_detected = True
                    return  # Found it, stop looking
                               
    def detect_blue_cube(self, frame, depth_image=None):
        """
        Detect blue cube with depth integration.
        Uses real depth when available, falls back to area estimation.
        """
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.BLUE_LOW, self.BLUE_HIGH)
        
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        self.cube_detected = False
        self.cube_area = 0
        self.cube_depth = None
        self.depth_confidence = 0.0
        
        if contours:
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)
            
            if area > self.CUBE_MIN_AREA:
                M = cv2.moments(largest)
                if M["m00"] > 0:
                    self.cube_cx = int(M["m10"] / M["m00"])
                    self.cube_cy = int(M["m01"] / M["m00"])
                    self.cube_area = area
                    self.cube_detected = True
                    
                    # Get depth at cube center
                    if depth_image is not None:
                        depth, confidence = self.get_depth_at_point(depth_image, self.cube_cx, self.cube_cy)
                        
                        if depth is not None and confidence > 0.3:
                            # Apply Kalman filter for smoothing
                            self.cube_depth = self.kf_depth.update(depth)
                            self.depth_confidence = confidence
                            
                            # Store as last valid if above blind spot threshold
                            if depth > self.DEPTH_BLIND_THRESHOLD:
                                self.last_valid_depth = depth
                                self.last_valid_time = rospy.Time.now()
                                
                                # Compute 3D position
                                pos_3d = self.compute_3d_position(self.cube_cx, self.cube_cy, depth)
                                if pos_3d is not None:
                                    self.last_valid_y_offset = self.kf_y_offset.update(pos_3d[0])
                                    self.last_valid_z_offset = self.kf_z_offset.update(pos_3d[1])
                        else:
                            # Depth invalid - might be in blind spot
                            self.depth_confidence = 0.0
                    
                    # Visual display for cube
                    x, y, w, h = cv2.boundingRect(largest)
                    cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 3)
                    
                    # Show depth info
                    if self.cube_depth is not None:
                        cv2.putText(frame, f"CUBE: {self.cube_depth:.2f}m ({self.depth_confidence:.0%})", 
                                   (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                    else:
                        cv2.putText(frame, f"CUBE: area={int(area)} (no depth)", 
                                   (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 165, 0), 2)
                                   
    def synced_image_cb(self, rgb_msg, depth_msg):
        """Process synchronized RGB + Depth images."""
        try:
            frame = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            
            # Handle different depth encodings
            if depth_msg.encoding == "32FC1":
                depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "32FC1")
            elif depth_msg.encoding == "16UC1":
                depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "16UC1")
            else:
                depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")
                
            self.latest_depth_image = depth_image
            self.depth_timestamp = depth_msg.header.stamp
            
            self.process_frame(frame, depth_image)
            
        except Exception as e:
            rospy.logerr(f"Synced image error: {e}")
            
    def rgb_only_cb(self, msg):
        """Fallback for RGB-only processing."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.process_frame(frame, None)
        except Exception as e:
            rospy.logerr(f"RGB image error: {e}")
            
    def process_frame(self, frame, depth_image):
        """Main frame processing - detection and state updates."""
        self.IMAGE_WIDTH = frame.shape[1]
        self.IMAGE_HEIGHT = frame.shape[0]
        self.IMAGE_CENTER_X = self.IMAGE_WIDTH // 2
        
        # Detect table and cube
        self.detect_table(frame)
        self.detect_blue_cube(frame, depth_image)
        
        # Update detection counts for state machine
        if self.state == "SEARCH":
            if self.table_detected or self.cube_detected:
                self.detect_count += 1
                if self.table_detected:
                    rospy.loginfo_throttle(0.5, f"TABLE seen! count={self.detect_count}")
                if self.cube_detected:
                    rospy.loginfo_throttle(0.5, f"CUBE seen! count={self.detect_count}")
            else:
                self.detect_count = max(0, self.detect_count - 1)
                
        # Track detection loss in approach states
        if self.state in ["APPROACH", "APPROACH_CUBE"]:
            if self.cube_detected or self.table_detected:
                self.no_detection_count = 0
            else:
                self.no_detection_count += 1
        
        # Check state transitions
        self.check_state_transitions()
        
        # Publish cube position for arm
        if self.state in ["APPROACH", "APPROACH_CUBE", "BLIND_APPROACH", "DONE"]:
            self.publish_cube_position()
            
        # Draw display
        self.draw_display(frame)
        
        # Publish annotated image
        try:
            self.annotated_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
        except Exception as e:
            rospy.logwarn_throttle(10, f"Failed to publish annotated image: {e}")
        
        # Store frame for GUI display (handled in main loop)
        self.display_frame = frame.copy()
        
    def check_state_transitions(self):
        """Handle state transitions with depth-aware logic."""
        
        # Publish current state
        self.state_pub.publish(String(self.state))
        
        # SEARCH -> APPROACH
        if self.state == "SEARCH":
            if self.detect_count >= self.CONFIRM_DETECTIONS:
                what = "CUBE" if self.cube_detected else "TABLE"
                rospy.logwarn("="*50)
                rospy.logwarn(f"{what} CONFIRMED! Starting APPROACH...")
                rospy.logwarn("="*50)
                self.state = "APPROACH"
                self.detect_count = 0
                self.no_detection_count = 0
                
        # APPROACH -> APPROACH_CUBE
        elif self.state == "APPROACH":
            if self.front_dist < self.SWITCH_TO_CUBE_LIDAR:
                rospy.logwarn("="*50)
                rospy.logwarn("CLOSE TO TABLE! Switching to CUBE approach")
                rospy.logwarn("="*50)
                self.state = "APPROACH_CUBE"
                self.no_detection_count = 0
                
        # APPROACH_CUBE -> BLIND_APPROACH (when in camera dead zone)
        elif self.state == "APPROACH_CUBE":
            # Check if we're in the depth camera blind spot
            in_blind_spot = False
            
            # IMPORTANT: Only enter blind approach if cube is CENTERED
            cube_centered = False
            if self.cube_detected:
                error = self.cube_cx - self.IMAGE_CENTER_X
                cube_centered = abs(error) < 40  # Must be well centered
            
            if self.cube_detected:
                # Depth invalid or below threshold = blind spot
                if self.cube_depth is None or self.cube_depth < self.DEPTH_BLIND_THRESHOLD:
                    if self.cube_area > 8000:  # Cube is big enough that we're close
                        in_blind_spot = True
                        rospy.loginfo_throttle(0.5, f"Depth blind spot detected (area={self.cube_area})")
                        
            # Also trigger blind approach based on LIDAR
            if self.front_dist < self.CUBE_STOP_LIDAR:
                in_blind_spot = True
                rospy.loginfo_throttle(0.5, f"LIDAR close: {self.front_dist:.2f}m")
            
            # Only enter blind approach if CENTERED and have last valid depth
            if in_blind_spot and self.last_valid_depth is not None and cube_centered:
                rospy.logwarn("="*50)
                rospy.logwarn("ENTERING BLIND APPROACH (camera dead zone)")
                rospy.logwarn(f"Last valid depth: {self.last_valid_depth:.3f}m")
                rospy.logwarn(f"Last valid Y offset: {self.last_valid_y_offset:.3f}m")
                rospy.logwarn("="*50)
                self.state = "BLIND_APPROACH"
                self.blind_approach_start_time = rospy.Time.now()
            elif in_blind_spot and not cube_centered:
                rospy.logwarn_throttle(1.0, f"In blind spot but NOT CENTERED (err={error}px). Aligning first...")
                
    def publish_cube_position(self):
        """
        Publish cube position for arm pick.
        Uses real depth when available, otherwise area-based estimation.
        In DONE state, uses stored final grasp position.
        """
        msg = Float32MultiArray()
        
        if self.state == "DONE":
            # Use stored final grasp position - constant for arm pick
            msg.data = [self.final_grasp_distance, self.final_grasp_y_offset, self.final_grasp_z_offset]
            
        elif self.state == "BLIND_APPROACH":
            # Use last known good position
            if self.last_valid_depth is not None:
                # Estimate current distance based on time elapsed
                elapsed = (rospy.Time.now() - self.blind_approach_start_time).to_sec()
                distance_traveled = elapsed * self.BLIND_APPROACH_SPEED
                current_dist = max(0.08, self.last_valid_depth - distance_traveled)
                
                msg.data = [current_dist, self.last_valid_y_offset, self.last_valid_z_offset]
            else:
                # Fallback to LIDAR-based estimate
                msg.data = [max(0.08, self.front_dist), 0.0, 0.0]
        else:
            # Normal operation
            if self.cube_depth is not None and self.depth_confidence > 0.3:
                # Use real depth from camera
                distance = self.cube_depth
                
                # Compute offsets using camera intrinsics
                y_offset = (self.cube_cx - self.cx) * distance / self.fx
                z_offset = (self.cube_cy - self.cy) * distance / self.fy
                
                # Apply Kalman filtering
                y_offset = self.kf_y_offset.update(y_offset)
                z_offset = self.kf_z_offset.update(z_offset)
                
            else:
                # Fallback to area-based estimation
                distance = max(0.08, min(2.0, 800.0 / max(100, math.sqrt(self.cube_area))))
                y_offset = (self.cube_cx - self.IMAGE_CENTER_X) * 0.0008 * distance
                z_offset = (self.cube_cy - self.IMAGE_HEIGHT // 2) * 0.0008 * distance
                
            msg.data = [distance, y_offset, z_offset]
            
        self.cube_pos_pub.publish(msg)
        
        rospy.loginfo_throttle(1.0, 
            f"Cube pos: dist={msg.data[0]:.3f}m, y={msg.data[1]:.3f}m, z={msg.data[2]:.3f}m")
        
    def draw_display(self, frame):
        """Draw rich debug overlay on camera view."""
        h, w = frame.shape[:2]
        
        # Semi-transparent background for status
        overlay = frame.copy()
        cv2.rectangle(overlay, (5, 5), (350, 140), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.5, frame, 0.5, 0, frame)
        
        # State colors
        state_colors = {
            "SEARCH": (0, 255, 255),       # Yellow
            "APPROACH": (0, 165, 255),     # Orange
            "APPROACH_CUBE": (255, 0, 0),  # Blue
            "BLIND_APPROACH": (255, 0, 255),  # Magenta
            "DONE": (0, 255, 0)            # Green
        }
        color = state_colors.get(self.state, (255, 255, 255))
        
        # State
        sub = self.search_state if self.state == "SEARCH" else ""
        cv2.putText(frame, f"State: {self.state} {sub}", (10, 25), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        
        # Detection status
        if self.cube_detected:
            if self.cube_depth is not None:
                det_str = f"CUBE: {self.cube_depth:.2f}m ({self.depth_confidence:.0%} conf)"
            else:
                det_str = f"CUBE: area={int(self.cube_area)} (NO DEPTH)"
            det_color = (255, 0, 0)
        elif self.table_detected:
            det_str = f"TABLE: area={int(self.table_area)}"
            det_color = (0, 200, 200)
        else:
            det_str = "Searching..."
            det_color = (128, 128, 128)
            
        cv2.putText(frame, det_str, (10, 50), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, det_color, 2)
        
        # LIDAR
        lidar_str = f"LIDAR F:{self.front_dist:.2f}m L:{self.left_dist:.1f} R:{self.right_dist:.1f}"
        cv2.putText(frame, lidar_str, (10, 75),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        
        # Depth status
        if self.cube_depth is not None:
            depth_status = f"Depth: {self.cube_depth:.2f}m (valid)"
            depth_color = (0, 255, 0)
        elif self.last_valid_depth is not None:
            age = (rospy.Time.now() - self.last_valid_time).to_sec() if self.last_valid_time else 999
            depth_status = f"Depth: BLIND (last={self.last_valid_depth:.2f}m, {age:.1f}s ago)"
            depth_color = (0, 165, 255)
        else:
            depth_status = "Depth: N/A"
            depth_color = (128, 128, 128)
            
        cv2.putText(frame, depth_status, (10, 100),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, depth_color, 2)
        
        # Blind approach timer
        if self.state == "BLIND_APPROACH" and self.blind_approach_start_time:
            elapsed = (rospy.Time.now() - self.blind_approach_start_time).to_sec()
            remaining = max(0, self.BLIND_APPROACH_DURATION - elapsed)
            cv2.putText(frame, f"BLIND: {remaining:.1f}s remaining", (10, 125),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)
        
        # Center line
        cv2.line(frame, (w//2, 0), (w//2, h), (0, 255, 0), 1)
        
        # Depth blind zone indicator
        if self.cube_detected and (self.cube_depth is None or self.cube_depth < self.DEPTH_BLIND_THRESHOLD):
            cv2.putText(frame, "⚠ DEPTH BLIND ZONE", (w//2 - 100, h - 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
    def check_obstacle(self):
        """Check for obstacles using LIDAR."""
        if self.front_dist < self.OBSTACLE_FRONT:
            return True
        if self.front_left_dist < self.OBSTACLE_SIDE:
            return True
        if self.front_right_dist < self.OBSTACLE_SIDE:
            return True
        return False
    
    def change_search_state(self, new_state):
        """Change search sub-state and reset timer."""
        rospy.loginfo(f"Search: {self.search_state} -> {new_state}")
        self.search_state = new_state
        self.state_start_time = rospy.Time.now()
        
        # Set random turn angle when entering TURN_90 (to avoid rectangular patterns)
        if new_state == "TURN_90":
            self.current_turn_angle = 90 + random.randint(-30, 30)  # 60° to 120°
        
    def search(self):
        """
        FAST SMART SEARCH - Go toward obstacles where table might be!
        - Faster movement (0.35 m/s)
        - Shorter turns - explore more efficiently
        - Goes TOWARD obstacles to discover what's behind them
        - Quick 180° scan when stuck
        """
        # CRITICAL: If we see a target, stop searching and let state transition happen
        if self.cube_detected or self.table_detected:
            self._safe_publish_cmd(Twist())
            return
        
        twist = Twist()
        elapsed = (rospy.Time.now() - self.state_start_time).to_sec()
        
        # FASTER thresholds - don't be so cautious!
        STOP_DISTANCE = 0.5       # Only stop when really close to obstacle
        SLOW_DISTANCE = 1.0       # Start slowing down
        ESCAPE_THRESHOLD = 0.35   # Too close - back up
        SIDE_BUFFER = 0.4
        
        if self.search_state == "FORWARD":
            # Check if too close - escape!
            if self.front_dist < ESCAPE_THRESHOLD:
                rospy.logwarn(f"TOO CLOSE ({self.front_dist:.2f}m)! Escaping...")
                self.turn_direction = 1 if self.left_dist > self.right_dist else -1
                self.change_search_state("ESCAPE")
                
            # Check if we hit an obstacle - turn to go around it
            elif self.front_dist < STOP_DISTANCE:
                rospy.loginfo(f"Obstacle at {self.front_dist:.2f}m - looking around it")
                self.forward_count += 1
                
                # Every 2nd obstacle, do a quick scan
                if self.forward_count >= 2:
                    self.forward_count = 0
                    self.change_search_state("QUICK_SCAN")
                else:
                    # Turn toward more open space
                    self.turn_direction = 1 if self.left_dist > self.right_dist else -1
                    self.change_search_state("TURN_45")
                    
            # Forward time elapsed - do periodic scan
            elif elapsed >= 3.0:  # Shorter forward segments (was 5.0)
                self.forward_count += 1
                # Quick scan every 3 forward moves
                if self.forward_count >= 3:
                    self.forward_count = 0
                    self.change_search_state("QUICK_SCAN")
                else:
                    # Small turn to vary direction
                    self.turn_direction = 1 if self.forward_count % 2 == 0 else -1
                    self.change_search_state("TURN_45")
            else:
                # FAST forward movement!
                speed = 0.35  # Faster than before (was 0.25)
                
                # Slow down near obstacles
                if self.front_dist < SLOW_DISTANCE:
                    speed = max(0.2, 0.35 * (self.front_dist / SLOW_DISTANCE))
                
                twist.linear.x = speed
                
                # Steer away from side obstacles while moving
                if self.left_dist < SIDE_BUFFER:
                    twist.angular.z = -0.4  # Turn right
                elif self.right_dist < SIDE_BUFFER:
                    twist.angular.z = 0.4   # Turn left
                    
                rospy.loginfo_throttle(1.5, f"FORWARD: v={speed:.2f} front={self.front_dist:.1f}m")
        
        elif self.search_state == "QUICK_SCAN":
            # Quick 180° scan (not full 360) - faster!
            scan_time = 3.5  # ~180 degrees at 1.0 rad/s
            if elapsed >= scan_time:
                rospy.loginfo("Quick scan complete")
                # Go toward direction with obstacles (likely table is there!)
                if self.front_dist < 3.0:
                    self.change_search_state("FORWARD")
                else:
                    # Turn toward side with closer obstacle
                    self.turn_direction = 1 if self.left_dist < self.right_dist else -1
                    self.change_search_state("TURN_45")
            else:
                twist.angular.z = 1.2  # Fast rotation
                progress = (elapsed / scan_time) * 180
                rospy.loginfo_throttle(0.5, f"SCANNING: {progress:.0f}°/180°")
        
        elif self.search_state == "TURN_45":
            # Quick 45° turn - much faster than 90°!
            turn_time = 0.6  # ~45 degrees (was 1.5 for 90°)
            
            if elapsed >= turn_time:
                self.change_search_state("FORWARD")
            else:
                twist.angular.z = 1.0 * self.turn_direction
                rospy.loginfo_throttle(0.3, f"TURNING: {elapsed:.1f}s")
        
        elif self.search_state == "TURN_90":
            # Backward compat - 90° turn if needed
            turn_time = 1.2  # ~90 degrees
            
            if elapsed >= turn_time:
                self.change_search_state("FORWARD")
            else:
                twist.angular.z = 1.0 * self.turn_direction
        
        elif self.search_state == "SCAN_360":
            # Full 360° rotation - only if really stuck
            scan_time = 5.0  # Faster rotation
            if elapsed >= scan_time:
                rospy.loginfo("360° scan complete")
                self.turn_direction = 1 if self.left_dist > self.right_dist else -1
                self.change_search_state("FORWARD")
            else:
                twist.angular.z = 1.2
        
        elif self.search_state == "ESCAPE":
            if elapsed < 1.0:
                twist.linear.x = -0.2  # Back up faster
                rospy.loginfo_throttle(0.5, "ESCAPE: backing up...")
            elif elapsed < 2.0:
                twist.angular.z = 1.2 * self.turn_direction
                rospy.loginfo_throttle(0.5, "ESCAPE: turning...")
            else:
                if self.front_dist > STOP_DISTANCE:
                    self.change_search_state("FORWARD")
                else:
                    self.turn_direction *= -1
                    self.state_start_time = rospy.Time.now()
                    rospy.logwarn("Still stuck - trying other direction")
                
        # Only publish if our mission phase is active and no emergency
        if self.mission_active and not self.emergency_stop:
            self._safe_publish_cmd(twist)
    
    def check_obstacle_and_avoid(self):
        """
        Bug-like obstacle avoidance algorithm.
        Returns: (should_avoid, twist) - if should_avoid is True, use the returned twist
        """
        twist = Twist()
        
        # Check if there's an obstacle in front
        obstacle_front = self.front_dist < self.OBSTACLE_DIST
        obstacle_front_left = self.front_left_dist < self.OBSTACLE_DIST
        obstacle_front_right = self.front_right_dist < self.OBSTACLE_DIST
        
        # If we're currently avoiding
        if self.avoiding_obstacle:
            elapsed = (rospy.Time.now() - self.avoidance_start_time).to_sec()
            
            # Check timeout
            if elapsed > self.AVOIDANCE_TIMEOUT:
                rospy.logwarn("Obstacle avoidance timeout! Trying reverse direction")
                self.avoidance_direction *= -1
                self.avoidance_start_time = rospy.Time.now()
            
            # Check if we can resume (front is clear AND we can see target again)
            front_clear = self.front_dist > self.OBSTACLE_CLEAR_DIST
            target_visible = self.cube_detected or self.table_detected
            
            if front_clear and (target_visible or elapsed > 3.0):
                rospy.loginfo("✓ Obstacle cleared! Resuming approach.")
                self.avoiding_obstacle = False
                return False, twist
            
            # Continue avoiding - follow obstacle boundary
            # Move forward slowly while turning away from obstacle
            if self.avoidance_direction > 0:  # Going left
                side_clear = self.left_dist > 0.3
            else:  # Going right
                side_clear = self.right_dist > 0.3
            
            if obstacle_front:
                # Turn in place to get away from obstacle
                twist.linear.x = 0.0
                twist.angular.z = 0.6 * self.avoidance_direction
                rospy.loginfo_throttle(0.3, f"AVOIDING: Turning {'left' if self.avoidance_direction > 0 else 'right'} (front={self.front_dist:.2f}m)")
            elif side_clear:
                # Move forward while hugging the obstacle
                twist.linear.x = 0.12
                twist.angular.z = 0.3 * self.avoidance_direction
                rospy.loginfo_throttle(0.3, f"AVOIDING: Following boundary (front={self.front_dist:.2f}m)")
            else:
                # Tight space - just turn
                twist.angular.z = 0.5 * self.avoidance_direction
                rospy.loginfo_throttle(0.3, f"AVOIDING: Tight space, turning")
            
            return True, twist
        
        # Not currently avoiding - check if we should start
        if obstacle_front or (obstacle_front_left and obstacle_front_right):
            # Start avoidance!
            self.avoiding_obstacle = True
            self.avoidance_start_time = rospy.Time.now()
            
            # Choose direction: prefer side with more space
            if self.left_dist > self.right_dist:
                self.avoidance_direction = 1  # Turn left
            else:
                self.avoidance_direction = -1  # Turn right
            
            # If one side is blocked, go the other way
            if obstacle_front_left and not obstacle_front_right:
                self.avoidance_direction = -1  # Turn right
            elif obstacle_front_right and not obstacle_front_left:
                self.avoidance_direction = 1  # Turn left
            
            rospy.logwarn(f"⚠ OBSTACLE DETECTED at {self.front_dist:.2f}m! Avoiding {'left' if self.avoidance_direction > 0 else 'right'}")
            
            twist.angular.z = 0.6 * self.avoidance_direction
            return True, twist
        
        return False, twist

    def approach(self):
        """APPROACH state: Go toward TABLE or CUBE (prefer CUBE)."""
        twist = Twist()
        
        # Check for obstacles first (but not when we're very close to target)
        if self.front_dist > 0.3:  # Only avoid if not super close to target
            should_avoid, avoid_twist = self.check_obstacle_and_avoid()
            if should_avoid:
                self._safe_publish_cmd(avoid_twist)
                return
        
        if self.cube_detected:
            target_cx = self.cube_cx
            dist_info = f"depth={self.cube_depth:.2f}m" if self.cube_depth else f"area={int(self.cube_area)}"
            rospy.loginfo_throttle(0.5, f"APPROACH CUBE: {dist_info} LIDAR={self.front_dist:.2f}m")
        elif self.table_detected:
            target_cx = self.table_cx
            rospy.loginfo_throttle(0.5, f"APPROACH TABLE: area={int(self.table_area)} LIDAR={self.front_dist:.2f}m")
        else:
            if self.no_detection_count > 20:
                rospy.logwarn("Lost target! Returning to SEARCH")
                self.state = "SEARCH"
                self.avoiding_obstacle = False  # Reset avoidance
                self.kf_depth.reset()
                self.kf_y_offset.reset()
                self.kf_z_offset.reset()
                return
            twist.angular.z = 0.3
            rospy.loginfo_throttle(1, "Lost target, rotating to find...")
            self._safe_publish_cmd(twist)
            return
            
        error = target_cx - self.IMAGE_CENTER_X
        twist.angular.z = -error * 0.005
        
        if abs(error) < 150:
            twist.linear.x = 0.18
            
        self._safe_publish_cmd(twist)
        
    def approach_cube(self):
        """APPROACH_CUBE state: Precise approach to BLUE cube only.
        
        NO obstacle avoidance here - we're close and need to align precisely.
        The 'obstacles' detected are usually the table itself!
        
        CRITICAL: Robot must approach HEAD-ON, not from the side!
        """
        twist = Twist()
        
        # DISABLED obstacle avoidance in APPROACH_CUBE - we need to align, not avoid!
        # The LIDAR sees the table as an obstacle which causes loops
        self.avoiding_obstacle = False  # Reset any avoidance state
        
        if not self.cube_detected:
            if self.no_detection_count > 15:
                rospy.logwarn("Lost cube! Returning to APPROACH")
                self.state = "APPROACH"
                self.avoiding_obstacle = False  # Reset avoidance
                return
            twist.angular.z = 0.25
            rospy.loginfo_throttle(1, "Looking for blue cube...")
            self._safe_publish_cmd(twist)
            return
        
        # Calculate error - cube must be CENTERED in image
        error = self.cube_cx - self.IMAGE_CENTER_X
        
        # Use depth if available, otherwise area
        # CRITICAL: Arm can only reach ~25cm, robot MUST be within 12cm of table!
        if self.cube_depth is not None and self.depth_confidence > 0.3:
            close_enough = self.cube_depth < 0.14  # Must be very close!
            dist_str = f"depth={self.cube_depth:.2f}m"
        else:
            # In blind zone - use LIDAR!
            close_enough = self.front_dist < self.CUBE_STOP_LIDAR  # use configured stop distance
            dist_str = f"LIDAR={self.front_dist:.2f}m, area={int(self.cube_area)}"
        
        # ===========================================
        # VERY STRICT centering - cube MUST be almost perfectly centered!
        # This ensures the robot approaches HEAD-ON
        # ===========================================
        if self.cube_area > 8000:  # Very close - strictest
            center_threshold = 20  # ±20 pixels only!
        elif self.cube_area > 5000:  # Close
            center_threshold = 35
        else:  # Far
            center_threshold = 50
        
        centered = abs(error) < center_threshold
        
        rospy.loginfo_throttle(0.5, f"CUBE: {dist_str} err={error} thr={center_threshold} centered={centered} LIDAR={self.front_dist:.2f}m")
        
        # ===========================================
        # ALIGNMENT-FIRST APPROACH:
        # 1. If not centered: STOP completely and ROTATE only
        # 2. If centered + close: trigger arm
        # 3. If centered + not close: Move forward slowly
        # ===========================================
        
        if not centered:
            # NOT CENTERED - FULL STOP and rotate to align
            twist.linear.x = 0.0  # STOP forward motion completely
            
            # Calculate rotation speed - proportional to error but with minimum
            rot_speed = -error * 0.004
            # Minimum rotation speed to overcome friction
            min_rot = 0.18
            if abs(rot_speed) < min_rot:
                rot_speed = min_rot if error < 0 else -min_rot
            # Maximum rotation speed
            rot_speed = max(-0.5, min(0.5, rot_speed))
            
            twist.angular.z = rot_speed
            rospy.loginfo_throttle(0.3, f"  -> ALIGNING: STOPPED, rotating (err={error}, rot={rot_speed:.2f})")
            self._safe_publish_cmd(twist)
            return
        
        # CENTERED - now we can move forward
        # Check if close enough for arm pick
        if close_enough:
            # Final alignment check before triggering arm
            if self.cube_depth is not None and self.cube_depth > self.DEPTH_BLIND_THRESHOLD:
                y_offset_meters = (self.cube_cx - self.IMAGE_CENTER_X) * self.cube_depth / self.fx
                
                if abs(y_offset_meters) > 0.02:  # 2cm tolerance for final (stricter)
                    rospy.logwarn(f"Final alignment: Y-offset={y_offset_meters:.3f}m. Fine-tuning...")
                    twist.angular.z = -error * 0.008
                    twist.linear.x = 0.0
                    self._safe_publish_cmd(twist)
                    return
            
            # ALIGNED and CLOSE - trigger arm pick!
            self._safe_publish_cmd(Twist())  # Full stop
            rospy.logwarn("="*60)
            rospy.logwarn("PERFECTLY ALIGNED & CUBE REACHED! Triggering arm pick...")
            rospy.logwarn("="*60)
            self.state = "DONE"
            self.mission_active = False  # Release cmd_vel control
            rospy.sleep(0.5)
            self.trigger_pub.publish(Bool(data=True))
            return
        
        # CENTERED but not close enough - move forward SLOWLY with gentle steering
        twist.linear.x = 0.06  # Very slow forward
        twist.angular.z = -error * 0.001  # Minimal correction (keep centered)
        rospy.loginfo_throttle(0.5, f"  -> CENTERED: moving forward slowly")
        self._safe_publish_cmd(twist)
        
    def blind_approach(self):
        """
        BLIND_APPROACH state: Dead reckoning when depth camera is blind.
        Uses last known position and slow creep forward.
        """
        twist = Twist()
        
        elapsed = (rospy.Time.now() - self.blind_approach_start_time).to_sec()
        
        if elapsed >= self.BLIND_APPROACH_DURATION:
            # Blind approach complete - store final grasp position for arm
            self._safe_publish_cmd(Twist())
            
            # Calculate and store final position for arm pick
            distance_traveled = elapsed * self.BLIND_APPROACH_SPEED
            self.final_grasp_distance = max(0.10, self.last_valid_depth - distance_traveled) if self.last_valid_depth else 0.12
            self.final_grasp_y_offset = self.last_valid_y_offset
            self.final_grasp_z_offset = self.last_valid_z_offset
            
            rospy.logwarn("="*60)
            rospy.logwarn("BLIND APPROACH COMPLETE! Triggering arm pick...")
            rospy.logwarn(f"Final grasp: dist={self.final_grasp_distance:.3f}m, y={self.final_grasp_y_offset:.3f}m")
            rospy.logwarn("="*60)
            self.state = "DONE"
            rospy.sleep(0.5)
            self.trigger_pub.publish(Bool(data=True))
            return
            
        # Safety check with LIDAR
        if self.front_dist < 0.05:
            rospy.logwarn("LIDAR: Too close! Stopping blind approach.")
            self._safe_publish_cmd(Twist())
            
            # Store final position
            distance_traveled = elapsed * self.BLIND_APPROACH_SPEED
            self.final_grasp_distance = max(0.10, self.last_valid_depth - distance_traveled) if self.last_valid_depth else 0.10
            self.final_grasp_y_offset = self.last_valid_y_offset
            self.final_grasp_z_offset = self.last_valid_z_offset
            
            self.state = "DONE"
            rospy.sleep(0.5)
            self.trigger_pub.publish(Bool(data=True))
            return
            
        # Slow creep forward
        twist.linear.x = self.BLIND_APPROACH_SPEED
        
        # Use last known lateral offset for steering (if we had good vision data)
        if self.last_valid_y_offset is not None:
            # Steer to correct lateral offset
            twist.angular.z = -self.last_valid_y_offset * 0.5
            twist.angular.z = np.clip(twist.angular.z, -0.1, 0.1)
            
        rospy.loginfo_throttle(0.3, 
            f"BLIND: {elapsed:.1f}/{self.BLIND_APPROACH_DURATION}s, "
            f"LIDAR={self.front_dist:.2f}m, y_corr={twist.angular.z:.3f}")
            
        self._safe_publish_cmd(twist)
        
    def _emergency_stop_cb(self, msg):
        """Handle emergency stop signal."""
        if msg.data and not self.emergency_stop:
            rospy.logwarn("!" * 60)
            rospy.logwarn("EMERGENCY STOP RECEIVED - Stopping blue cube approach!")
            rospy.logwarn("!" * 60)
            self.emergency_stop = True
            self.mission_active = False
            self.cmd_pub.publish(Twist())  # Stop immediately
        elif not msg.data and self.emergency_stop:
            rospy.loginfo("Emergency stop cleared")
            self.emergency_stop = False

    def _safe_publish_cmd(self, twist):
        """Safely publish cmd_vel - only if mission is active and no emergency stop.
        
        This is the ONLY method that should publish to cmd_vel!
        Prevents race conditions when multiple nodes might publish.
        """
        if self.emergency_stop:
            # Emergency stop - always publish zero
            self.cmd_pub.publish(Twist())
            return
        if not self.mission_active:
            # Not our turn - don't publish
            return
        # Safe to publish
        self.cmd_pub.publish(twist)

    def _trigger_cb(self, msg):
        """Handle mission controller trigger to start search."""
        rospy.loginfo(f"[TRIGGER] Received: {msg.data}, current state: {self.state}")
        if msg.data and self.state == "IDLE":
            rospy.logwarn("="*60)
            rospy.logwarn("BLUE CUBE APPROACH TRIGGERED - Starting search!")
            rospy.logwarn("="*60)
            self.state = "SEARCH"
            self.mission_active = True  # We now have control of cmd_vel
            self.state_start_time = rospy.Time.now()
            # Publish status for mission controller
            self.status_pub.publish(String(data="SEARCHING"))
        elif not msg.data and self.mission_active:
            # Mission controller is deactivating us
            rospy.loginfo("Blue cube approach deactivated")
            self.mission_active = False
            self.cmd_pub.publish(Twist())  # Stop

    def run(self):
        """Main loop with GUI display."""
        rate = rospy.Rate(15)
        rospy.loginfo("Starting search - GUI window active (press 'q' to quit)")
        
        while not rospy.is_shutdown():
            # Execute state machine
            if self.state == "SEARCH":
                self.search()
            elif self.state == "APPROACH":
                self.approach()
            elif self.state == "APPROACH_CUBE":
                self.approach_cube()
            elif self.state == "BLIND_APPROACH":
                self.blind_approach()
            elif self.state == "DONE":
                self.cmd_pub.publish(Twist())
                # Always publish cube position in DONE state (uses stored final grasp)
                self.publish_cube_position()
                
                # Keep triggering arm pick until acknowledged (MoveIt may take time to start)
                if not self.arm_pick_acknowledged:
                    now = rospy.Time.now()
                    if self.last_trigger_time is None or (now - self.last_trigger_time).to_sec() > self.TRIGGER_INTERVAL:
                        rospy.logwarn(f"Sending arm pick trigger (waiting for MoveIt...)")
                        self.trigger_pub.publish(Bool(data=True))
                        self.last_trigger_time = now

            elif self.state == "IDLE":
                # In IDLE state, publish status so mission controller knows we're ready
                # But do NOT publish to cmd_vel
                self.status_pub.publish(String(data="IDLE"))
                rospy.loginfo_throttle(5, "Blue cube: IDLE - waiting for trigger")
            
            # Display GUI in main loop (only if display available - disabled on headless robots)
            if self.display_frame is not None and self.enable_display:
                try:
                    cv2.imshow("Enhanced Blue Cube Approach", self.display_frame)
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        rospy.signal_shutdown("User quit")
                except Exception as e:
                    # Display not available (headless mode) - disable future attempts
                    rospy.logwarn_once(f"Display disabled (headless mode): {e}")
                    self.enable_display = False
                    
            rate.sleep()
            
        cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        node = EnhancedTwoStageApproach()
        node.run()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()
