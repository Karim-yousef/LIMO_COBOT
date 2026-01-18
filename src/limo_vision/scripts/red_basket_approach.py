#!/usr/bin/env python3
"""
RED BASKET APPROACH - Navigate to red basket for cube placement

Features:
1. BACKUP phase before rotation (moves away from table first)
2. LIDAR-based obstacle avoidance during search
3. Red color detection with dual HSV range
4. State machine for navigation control
5. Debug visualization

States:
1. BACKUP - Move backward away from table
2. SEARCH - Rotate and look for red basket
3. APPROACH - Navigate toward the basket
4. FINAL_APPROACH - Get close enough for arm placement
5. DONE - Stop and trigger arm place
"""

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from cv_bridge import CvBridge
import math

class RedBasketApproach:
    """Navigate to dark red basket for placing cubes."""
    
    # States
    IDLE = "IDLE"
    BACKUP = "BACKUP"          # NEW: Back away from table first!
    SEARCH = "SEARCH"
    APPROACH = "APPROACH"
    FINAL_APPROACH = "FINAL_APPROACH"
    DONE = "DONE"
    
    def __init__(self):
        rospy.init_node('red_basket_approach', anonymous=True)
        
        rospy.loginfo("="*60)
        rospy.loginfo("RED BASKET APPROACH")

        rospy.loginfo("="*60)
        
        self.bridge = CvBridge()
        self.state = self.IDLE
        self.emergency_stop = False
        self.last_published_status = None  # Track to reduce log spam
        self.mission_active = False  # Only publish cmd_vel when our phase is active  # Emergency stop flag
        
        # Image dimensions (will be updated from actual image)
        self.image_width = 640
        self.image_height = 480
        self.image_center_x = 320
        
        # Movement parameters
        self.LINEAR_SPEED = 0.15
        self.ANGULAR_SPEED = 0.4
        self.SEARCH_ANGULAR = 0.5
        self.BACKUP_SPEED = -0.12  # Reverse speed (negative = backward)
        
        # Distance thresholds
        self.APPROACH_START_DIST = 2.0
        self.FINAL_APPROACH_DIST = 0.50
        self.STOP_DIST = 0.12  # Get VERY close to basket for arm reach!
        self.MIN_SAFE_DIST = 0.10  # Very close minimum
        
        # ==============================================
        # DARK RED COLOR DETECTION - UNIQUE COLOR!
        # Dark red is Hue ~0-10 AND ~170-180 (red wraps around)
        # Nothing else in scene is dark red - very unique!
        # ==============================================
        # Dark red HSV range - red hue with medium saturation/value
        # Red hue is 0-10 in OpenCV (wraps at 180)
        self.BASKET_LOW = np.array([0, 100, 50])     # Red hue, medium sat, medium value
        self.BASKET_HIGH = np.array([10, 255, 255])  # Red hue range 0-10
        
        # Alternative: Upper red range (170-180, wraps around)
        self.BASKET_BRIGHT_LOW = np.array([170, 100, 50])
        self.BASKET_BRIGHT_HIGH = np.array([180, 255, 255])
        
        # Minimum area to consider as basket
        self.MIN_BASKET_AREA = 500  # Lower threshold for distant basket
        self.MIN_BASKET_AREA_APPROACH = 800  # Slightly higher when approaching
        
        # Detection state
        self.basket_detected = False
        self.basket_cx = 0
        self.basket_cy = 0
        self.basket_area = 0
        self.consecutive_detections = 0
        self.CONFIRM_DETECTIONS = 3  # Need 5 consecutive detections to confirm basket
        
        # LIDAR data
        self.front_dist = 10.0
        self.front_left_dist = 10.0
        self.front_right_dist = 10.0
        self.left_dist = 10.0
        self.right_dist = 10.0
        self.rear_dist = 10.0  # NEW: For safe backup
        self.lidar_received = False  # Track if we're getting LIDAR data
        
        # ==============================================
        # BACKUP PHASE PARAMETERS (NEW!)
        # ==============================================
        self.BACKUP_DURATION = 2.5  # Seconds to back up
        self.BACKUP_SAFE_DIST = 0.8  # Minimum distance behind robot
        self.backup_start_time = None
        
        # Search state
        self.search_direction = 1
        self.search_start_time = None
        self.search_timeout = 120.0
        self.search_phase = "ROTATE_180"
        self.phase_start_time = None
        self.rotation_accumulated = 0.0  # Track actual rotation
        self.last_phase_time = None
        
        # Obstacle avoidance during approach
        self.OBSTACLE_DIST = 0.4
        self.avoiding_obstacle = False
        
        # Publishers
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.status_pub = rospy.Publisher('/basket_approach/status', String, queue_size=1)
        self.arm_place_pub = rospy.Publisher('/arm_place/trigger', Bool, queue_size=1)
        self.debug_pub = rospy.Publisher('/basket_debug/image', Image, queue_size=1)
        
        # ==============================================
        # LIDAR topic for LIMO robot
        # ==============================================

        # Camera subscriber
        rospy.Subscriber('/limo/color/image_raw', Image, self._image_cb, queue_size=1)
        
        # LIDAR subscriber - CRITICAL for obstacle detection!
        rospy.Subscriber('/limo/scan', LaserScan, self._scan_cb, queue_size=1)
        
        # Trigger subscriber
        rospy.Subscriber('/basket_approach/trigger', Bool, self._trigger_cb, queue_size=1)
        
        # Emergency stop subscriber
        rospy.Subscriber('/emergency_stop', Bool, self._emergency_stop_cb, queue_size=1)
        
        rospy.loginfo("Red basket approach ready")
        rospy.loginfo("Waiting for trigger on /basket_approach/trigger")
        rospy.loginfo(f"DARK RED HSV range: {self.BASKET_LOW} - {self.BASKET_HIGH}")
        rospy.loginfo(f"LIDAR topic: /limo/scan")
        
    def _emergency_stop_cb(self, msg):
        """Handle emergency stop signal."""
        if msg.data and not self.emergency_stop:
            rospy.logwarn("!" * 60)
            rospy.logwarn("EMERGENCY STOP - Basket approach halted!")
            rospy.logwarn("!" * 60)
            self.emergency_stop = True
            self.cmd_pub.publish(Twist())  # Stop immediately
        elif not msg.data and self.emergency_stop:
            rospy.loginfo("Emergency stop cleared")
            self.emergency_stop = False

    def _safe_publish_cmd(self, twist):
        """Safely publish cmd_vel - only if mission is active and no emergency stop."""
        if self.emergency_stop:
            self.cmd_pub.publish(Twist())
            return
        if not self.mission_active:
            return
        self.cmd_pub.publish(twist)

    def _trigger_cb(self, msg):
        """Trigger basket approach - starts with BACKUP phase!"""
        if msg.data and self.state == self.IDLE:
            self.mission_active = True  # We now have control of cmd_vel
            rospy.loginfo("="*60)
            rospy.loginfo("BASKET APPROACH TRIGGERED!")
            rospy.loginfo("Starting with BACKUP phase to clear table...")
            rospy.loginfo("="*60)
            
            # START WITH BACKUP, NOT SEARCH!
            self.state = self.BACKUP
            self.backup_start_time = rospy.Time.now()
            self.basket_detected = False
            self.consecutive_detections = 0
            
    def _scan_cb(self, msg):
        """Process LIDAR data with angle-based sector extraction."""
        ranges = np.array(msg.ranges, dtype=np.float32)
        n = len(ranges)
        
        if n == 0:
            return
            
        self.lidar_received = True
        
        # Replace invalid values
        ranges = np.where(np.isnan(ranges) | np.isinf(ranges) | (ranges <= 0.01), 10.0, ranges)
        
        # Build angle array from actual LaserScan parameters
        angles = msg.angle_min + np.arange(n) * msg.angle_increment
        
        def sector_min(start_rad, end_rad):
            """Get minimum range in radian range using actual angles."""
            # Handle wrap-around (e.g., near -pi/pi boundary)
            if start_rad <= end_rad:
                mask = (angles >= start_rad) & (angles <= end_rad)
            else:
                # Wrap around: e.g., start=2.6, end=-2.6 means rear sector
                mask = (angles >= start_rad) | (angles <= end_rad)
            
            vals = ranges[mask]
            return float(np.min(vals)) if vals.size > 0 else 10.0
        
        # Convert degrees to radians for clarity
        import math
        deg2rad = math.pi / 180.0
        
        # Front: -20 to +20 degrees (-0.35 to +0.35 rad)
        self.front_dist = sector_min(-20 * deg2rad, 20 * deg2rad)
        
        # Front-left: 20 to 50 degrees
        self.front_left_dist = sector_min(20 * deg2rad, 50 * deg2rad)
        
        # Front-right: -50 to -20 degrees
        self.front_right_dist = sector_min(-50 * deg2rad, -20 * deg2rad)
        
        # Left: 60 to 120 degrees
        self.left_dist = sector_min(60 * deg2rad, 120 * deg2rad)
        
        # Right: -120 to -60 degrees
        self.right_dist = sector_min(-120 * deg2rad, -60 * deg2rad)
        
        # REAR: 150 to 180 AND -180 to -150 degrees (wraps around)
        # This is the sector directly behind the robot
        rear1 = sector_min(150 * deg2rad, 180 * deg2rad)   # 150 to 180
        rear2 = sector_min(-180 * deg2rad, -150 * deg2rad) # -180 to -150
        self.rear_dist = min(rear1, rear2)
        
        rospy.loginfo_throttle(5, f"LIDAR: F={self.front_dist:.2f} FL={self.front_left_dist:.2f} FR={self.front_right_dist:.2f} REAR={self.rear_dist:.2f}")
        
    def _image_cb(self, msg):
        """Process camera image for dark red basket detection."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logwarn(f"Image conversion error: {e}")
            return
        
        # Update image dimensions
        self.image_height, self.image_width = frame.shape[:2]
        self.image_center_x = self.image_width // 2
            
        # Always detect (even in IDLE for debugging)
        self._detect_yellow_basket(frame)
        
        # Only run state machine if not IDLE
        if self.state != self.IDLE:
            self._run_state_machine()
        
    def _detect_yellow_basket(self, frame):
        """Detect dark red basket in frame - DARK RED ONLY (no magenta - that's the table!)."""
        h, w = frame.shape[:2]
        
        # Convert to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # DARK RED ONLY - basket is now CYAN color (0.0, 0.9, 0.9 RGB)
        # Dark Red in HSV is approximately Hue 90, high saturation
        # This is UNIQUE - no cones or other objects are black!
        
        # 1. Standard black range
        mask1 = cv2.inRange(hsv, self.BASKET_LOW, self.BASKET_HIGH)
        # 2. Bright/pure black 
        mask2 = cv2.inRange(hsv, self.BASKET_BRIGHT_LOW, self.BASKET_BRIGHT_HIGH)
        
        # ONLY CYAN - no yellow masks (to avoid cones!)
        mask = cv2.bitwise_or(mask1, mask2)
        
        # Morphological operations to clean up
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Find largest yellow region
        prev_detected = self.basket_detected
        self.basket_detected = False
        max_area = 0
        best_contour = None
        
        # Minimum area depends on state - use smaller threshold during BACKUP/SEARCH
        if self.state in [self.SEARCH, self.BACKUP]:
            min_area = self.MIN_BASKET_AREA  # 500 - detect from far away
        else:
            min_area = self.MIN_BASKET_AREA_APPROACH  # 800 - need more confidence when approaching
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > max_area and area > min_area:
                # Get bounding box
                x, y, bw, bh = cv2.boundingRect(cnt)
                aspect = float(bw) / bh if bh > 0 else 0
                
                # =====================================================
                # SHAPE FILTERING TO REJECT CONES
                # =====================================================
                # CONES are TALL and NARROW: aspect ratio < 0.7 (height > width)
                # BASKET is WIDE and SHORT: aspect ratio > 0.8 (width >= height)
                # Reject anything too tall/narrow (likely a cone!)
                if aspect < 0.20:  # Lowered from 0.8 - basket can appear tall from certain angles
                    rospy.loginfo_throttle(1.0, f"Rejected: tall/narrow shape (aspect={aspect:.2f}) - likely cone")
                    continue
                
                # Also reject if too wide (aspect > 2.5 is unrealistic for basket)
                if aspect > 2.5:
                    continue
                
                # POSITION FILTER: Basket should be in CENTER of image
                center_x = x + bw // 2
                center_y = y + bh // 2
                
                # Only accept if center is in middle 70% of image width
                if center_x < w * 0.05 or center_x > w * 0.95:  # Allow detection near edges during rotation
                    rospy.loginfo_throttle(1.0, f"Rejected: at edge of image (cx={center_x})")
                    continue
                
                # Reject if at very top of image (sky/background)
                if center_y < h * 0.2:
                    continue
                
                # SOLIDITY FILTER: Basket should be fairly solid/filled
                hull = cv2.convexHull(cnt)
                hull_area = cv2.contourArea(hull)
                if hull_area > 0:
                    solidity = area / hull_area
                    if solidity < 0.4:  # Lowered - basket shape can vary
                        rospy.loginfo_throttle(1.0, f"Rejected: irregular shape (solidity={solidity:.2f})")
                        continue
                
                # Passed all filters!
                max_area = area
                best_contour = cnt
                
        if best_contour is not None:
            self.basket_detected = True
            self.basket_area = max_area
            
            # Get centroid
            M = cv2.moments(best_contour)
            if M["m00"] > 0:
                self.basket_cx = int(M["m10"] / M["m00"])
                self.basket_cy = int(M["m01"] / M["m00"])
            
            # ONLY count consecutive detections during SEARCH or later states!
            # Don't count during BACKUP (we're still facing the table/cones)
            if self.state in [self.SEARCH, self.APPROACH, self.FINAL_APPROACH]:
                self.consecutive_detections += 1
                rospy.loginfo_throttle(0.5, f"BASKET DETECTED! area={self.basket_area}, cx={self.basket_cx}, cy={self.basket_cy}, consec={self.consecutive_detections}")
            else:
                # During BACKUP, just note detection but don't count
                rospy.loginfo_throttle(1.0, f"Yellow seen (not counting - state={self.state}): area={self.basket_area}, cx={self.basket_cx}")
        else:
            self.consecutive_detections = 0
            
        # Draw debug visualization
        debug_frame = frame.copy()
        
        # Show the yellow mask as overlay
        red_overlay = np.zeros_like(frame)
        red_overlay[:,:,1] = mask  # Green channel shows DARK RED detection
        debug_frame = cv2.addWeighted(debug_frame, 0.7, red_overlay, 0.3, 0)
        
        if best_contour is not None:
            cv2.drawContours(debug_frame, [best_contour], -1, (0, 255, 255), 3)
            cv2.circle(debug_frame, (self.basket_cx, self.basket_cy), 10, (0, 0, 255), -1)
            cv2.putText(debug_frame, f"BASKET: area={self.basket_area}", 
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        else:
            cv2.putText(debug_frame, "NO BASKET DETECTED", 
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        # State info
        cv2.putText(debug_frame, f"State: {self.state}", 
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        
        # LIDAR info
        lidar_color = (0, 255, 0) if self.lidar_received else (0, 0, 255)
        cv2.putText(debug_frame, f"LIDAR F:{self.front_dist:.2f}m R:{self.rear_dist:.2f}m", 
                    (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, lidar_color, 2)
        
        # Search phase if in SEARCH
        if self.state == self.SEARCH:
            cv2.putText(debug_frame, f"Phase: {self.search_phase}", 
                        (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)
        
        # Publish debug image
        try:
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug_frame, "bgr8"))
        except Exception as e:
            rospy.logwarn_throttle(10, f"Debug image publish error: {e}")
                
    def _run_state_machine(self):
        """Execute state machine."""
        twist = Twist()
        
        if self.state == self.BACKUP:
            twist = self._backup_state()
            
        elif self.state == self.SEARCH:
            twist = self._search_state()
            
        elif self.state == self.APPROACH:
            twist = self._approach_state()
            
        elif self.state == self.FINAL_APPROACH:
            twist = self._final_approach_state()
            
        elif self.state == self.DONE:
            twist = Twist()  # Stop
            
        # Publish command
        self._safe_publish_cmd(twist)
        
        # Publish status
        self.status_pub.publish(String(self.state))
        
    def _backup_state(self):
        """BACKUP state: Move backward AWAY from table before rotating!
        
        This is CRITICAL - the robot is right next to the table after picking
        up the cube. If it rotates in place, it will hit the table.
        
        Time-based approach - don't check front_dist (it starts at 10.0 which
        would cause immediate exit!)
        """
        twist = Twist()
        
        # Initialize backup timer
        if self.backup_start_time is None:
            self.backup_start_time = rospy.Time.now()
            rospy.loginfo("="*50)
            rospy.loginfo("BACKUP PHASE STARTED!")
            rospy.loginfo(f"Will reverse for {self.BACKUP_DURATION}s")
            rospy.loginfo("="*50)
            
        elapsed = (rospy.Time.now() - self.backup_start_time).to_sec()
        
        rospy.loginfo_throttle(0.5, f"BACKUP: elapsed={elapsed:.1f}s/{self.BACKUP_DURATION}s, rear_dist={self.rear_dist:.2f}m, front_dist={self.front_dist:.2f}m")
        
        # Safety: Don't back into something! (lowered threshold - 0.15m is very close)
        # Only stop if there's a REAL obstacle very close behind
        if self.rear_dist < 0.15:
            rospy.logwarn(f"Obstacle very close behind at {self.rear_dist:.2f}m! Stopping backup.")
            self._transition_to_search()
            return twist
        
        # Time-based approach - don't check front_dist (starts at 10.0)
        # Must backup for the full duration
        if elapsed >= self.BACKUP_DURATION:
            rospy.loginfo("="*50)
            rospy.loginfo(f"BACKUP COMPLETE! Moved backward for {elapsed:.1f}s")
            rospy.loginfo("Starting SEARCH phase...")
            rospy.loginfo("="*50)
            self._transition_to_search()
            return twist
        
        # Move backward (negative = reverse)
        twist.linear.x = self.BACKUP_SPEED
        
        return twist
    
    def _transition_to_search(self):
        """Helper to transition from BACKUP to SEARCH."""
        self.state = self.SEARCH
        self.search_start_time = rospy.Time.now()
        self.search_phase = "ROTATE_180"
        self.phase_start_time = rospy.Time.now()
        self.rotation_accumulated = 0.0
        # CRITICAL: Reset detection count when starting search!
        # This ensures we don't count detections from BACKUP phase
        self.consecutive_detections = 0
        self.basket_detected = False
        rospy.loginfo("Detection counters RESET for SEARCH phase")

    def _search_state(self):
        """SEARCH state: Multi-phase search for dark red basket.
        
        IMPROVED PHASES:
        Phase 1: ROTATE_180 - Turn around 180 degrees (check for obstacles!)
        Phase 2: SCAN - Slow rotation to scan for basket
        Phase 3: MOVE_FORWARD - Move forward carefully with obstacle avoidance
        
        If basket detected -> immediately switch to APPROACH
        """
        twist = Twist()
        
        # Check overall timeout
        if self.search_start_time:
            total_elapsed = (rospy.Time.now() - self.search_start_time).to_sec()
            if total_elapsed > self.search_timeout:
                rospy.logwarn("Search timeout! Basket not found.")
                self.state = self.IDLE
                return twist
        
        # If basket detected with confirmation, switch to approach
        if self.basket_detected and self.consecutive_detections >= self.CONFIRM_DETECTIONS:
            rospy.loginfo("="*50)
            rospy.loginfo(f"BASKET CONFIRMED! Area={self.basket_area}")
            rospy.loginfo("Switching to APPROACH")
            rospy.loginfo("="*50)
            self.state = self.APPROACH
            return twist
        
        # Phase timing
        if self.phase_start_time is None:
            self.phase_start_time = rospy.Time.now()
        phase_elapsed = (rospy.Time.now() - self.phase_start_time).to_sec()
        
        # Track rotation for accurate 180-degree turn
        dt = 0.1  # Approximate loop rate
        if self.last_phase_time is not None:
            dt = (rospy.Time.now() - self.last_phase_time).to_sec()
        self.last_phase_time = rospy.Time.now()
        
        rospy.loginfo_throttle(1.0, f"SEARCH: {phase_elapsed:.1f}s, basket_detected={self.basket_detected}, consec={self.consecutive_detections}")
        
        # SIMPLE APPROACH: Just rotate LEFT until we see the basket
        # At 0.5 rad/s, 180 degrees (pi rad) takes ~6.3 seconds
        # Keep rotating - the basket check at the top will stop us when found
        
        # ALWAYS rotate LEFT (positive angular.z)
        twist.angular.z = 0.35  # LEFT rotation
        
        # If we've been rotating for a very long time (30s = almost 2 full rotations)
        # without finding basket, try backing up
        if phase_elapsed >= 30.0:
            self.search_phase = "REPOSITION_BACK"
            self.phase_start_time = rospy.Time.now()
            rospy.loginfo("SEARCH: Long rotation without finding basket, backing up...")
                
        if self.search_phase == "REPOSITION_BACK":
            # Move BACKWARD to get a different view
            if self.rear_dist < 0.20:
                rospy.logwarn(f"Cannot back up more, continuing rotation...")
                self.search_phase = "ROTATE"
                self.phase_start_time = rospy.Time.now()
                return twist
            
            # Back up slowly
            twist.linear.x = -0.10
            twist.angular.z = 0.0  # Stop rotation while backing up
            
            # Back up for 2 seconds, then rotate again
            if phase_elapsed >= 2.0:
                self.search_phase = "ROTATE"
                self.phase_start_time = rospy.Time.now()
                rospy.loginfo("SEARCH: Backed up, rotating again...")
        
        return twist

    def _approach_state(self):
        """APPROACH state: Navigate toward basket."""
        twist = Twist()
        
        # Check for confirmed detection
        if not self.basket_detected:
            # Lost basket briefly - slow down and look
            if self.consecutive_detections == 0:
                rospy.logwarn_throttle(1, "Lost basket, slowing down to search...")
                twist.linear.x = 0.05
                twist.angular.z = 0.2 * self.search_direction
                return twist
        
        # Check if close enough for final approach
        if self.front_dist < self.FINAL_APPROACH_DIST:
            rospy.loginfo(f"Close to basket ({self.front_dist:.2f}m), switching to FINAL_APPROACH")
            self.state = self.FINAL_APPROACH
            return twist
        
        # Obstacle avoidance
        if self.front_dist < self.OBSTACLE_DIST:
            rospy.logwarn(f"Obstacle at {self.front_dist:.2f}m during approach!")
            # Try to go around
            if self.left_dist > self.right_dist:
                twist.angular.z = 0.4
            else:
                twist.angular.z = -0.4
            twist.linear.x = 0.05
            return twist
            
        # Calculate steering based on basket position
        error = (self.basket_cx - self.image_center_x) / float(self.image_center_x)  # -1 to 1
        
        # If very misaligned, prioritize rotation over forward movement
        if abs(error) > 0.25:
            # Mostly rotate, minimal forward
            twist.linear.x = 0.05
            twist.angular.z = -error * 0.6  # Stronger rotation
            rospy.loginfo_throttle(1, f"APPROACH: ALIGNING - error={error:.2f}")
        else:
            # Move forward and steer toward basket
            twist.linear.x = self.LINEAR_SPEED
            twist.angular.z = -error * self.ANGULAR_SPEED
        
        # Slow down as we get closer
        if self.front_dist < 1.0:
            twist.linear.x *= 0.5
            
        rospy.loginfo_throttle(1, f"APPROACH: dist={self.front_dist:.2f}m, error={error:.2f}, area={self.basket_area}")
        
        return twist
        
    def _final_approach_state(self):
        """FINAL_APPROACH state: Careful approach for placement."""
        twist = Twist()
        
        # Stop if close enough
        if self.front_dist < self.STOP_DIST:
            rospy.loginfo("="*60)
            rospy.loginfo("ARRIVED AT BASKET!")
            rospy.loginfo(f"Distance: {self.front_dist:.2f}m")
            rospy.loginfo("="*60)
            self.state = self.DONE
            
            # Wait for robot to stop, then trigger arm
            rospy.sleep(1.0)
            self.arm_place_pub.publish(Bool(True))
            rospy.loginfo("Arm place triggered!")
            return twist
            
        # Safety check
        if self.front_dist < self.MIN_SAFE_DIST:
            rospy.logwarn("Too close! Stopping.")
            self.state = self.DONE
            self.arm_place_pub.publish(Bool(True))
            return twist
            
        if not self.basket_detected:
            # In final approach, continue slowly even without detection
            rospy.loginfo_throttle(1, "FINAL: Lost visual, creeping forward...")
            twist.linear.x = 0.05
            return twist
            
        # Careful approach - ALIGN FIRST, then move forward
        error = (self.basket_cx - self.image_center_x) / float(self.image_center_x)
        
        # If basket not centered, STOP and ROTATE to align first!
        if abs(error) > 0.15:  # More than 15% off center
            rospy.loginfo_throttle(0.5, f"FINAL: ALIGNING - error={error:.2f}, rotating only")
            twist.linear.x = 0.0  # STOP forward motion
            twist.angular.z = -error * 0.5  # Rotate to center basket
        else:
            # Basket centered, move forward slowly with minor corrections
            twist.linear.x = 0.06  # Very slow approach
            twist.angular.z = -error * 0.25
        
        rospy.loginfo_throttle(0.5, f"FINAL: dist={self.front_dist:.2f}m, error={error:.2f}")
        
        return twist
        
    def run(self):
        """Main loop."""
        rate = rospy.Rate(10)
        
        rospy.loginfo("Red basket approach main loop starting...")
        rospy.loginfo("Send True to /basket_approach/trigger to start")
        
        while not rospy.is_shutdown():
            # Publish status even when idle
            # Only publish if status changed (reduce spam)
            if self.state != self.last_published_status:
                self.status_pub.publish(String(self.state))
                self.last_published_status = self.state
            
            # Log periodic status
            if self.state == self.IDLE:
                rospy.loginfo_throttle(10, f"IDLE - waiting for trigger. LIDAR received: {self.lidar_received}")
            
            rate.sleep()

if __name__ == '__main__':
    try:
        node = RedBasketApproach()
        node.run()
    except rospy.ROSInterruptException:
        pass
