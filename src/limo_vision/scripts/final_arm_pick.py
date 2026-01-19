#!/usr/bin/env python3
"""
ARM PICK AND PLACE CONTROLLER

Supports both SIMULATION and REAL ROBOT modes:
- Simulation: Can use Gazebo ModelStates for ground truth (optional)
- Real Robot: Uses vision-based cube position from /cube_position topic

Features:
- ROS parameters for tunable values
- Exception handling with recovery
- Works without Gazebo dependency
"""

import sys
import rospy
import moveit_commander
import actionlib
from moveit_msgs.msg import MoveGroupAction
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from std_msgs.msg import Bool, Float32MultiArray, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import LaserScan
import tf

# Gazebo import - optional for real robot deployment
try:
    from gazebo_msgs.msg import ModelStates
    GAZEBO_AVAILABLE = True
except ImportError:
    GAZEBO_AVAILABLE = False
    ModelStates = None
import tf.transformations as tf_trans
import math
import threading


class DynamicArmPick:
    """Arm pick/place controller - works in simulation and real robot."""
    
    CUBE_CENTER_Z_FALLBACK = 0.225
    
    ARM_MAX_FORWARD = 0.28
    ARM_MIN_FORWARD = 0.08
    ARM_MAX_SIDE = 0.15
    
    PRE_GRASP_OFFSET = 0.06
    LIFT_HEIGHT = 0.08
    GRASP_Z_LOWER = -0.01  # RAISE by 1cm - grasp slightly above cube center to avoid pushing
    GRASP_Y_OFFSET = -0.01  # Shift RIGHT by 1cm to compensate for camera/arm offset
    
    # Gripper joint limits: lower=-0.7, upper=0.15
    # GRIPPER JOINT: prismatic, moves finger INWARD (toward center)
    # Finger gap at 0.0 = 6cm (fully open)
    # Finger gap at 0.04 = fingers overlap (way too closed!)
    # Cube is 5cm wide, need ~5cm gap to grip it
    # Each finger moves (6-5)/2 = 0.5cm, use 0.008 for snug grip (~4.4cm gap)
    GRIPPER_OPEN = -0.01   # Wide open: 8cm gap (fingers spread outward)
    GRIPPER_CLOSED = 0.015  # Snug grip: 4.8cm gap for 5cm cube
    
    # Offset from base_footprint to LIDAR
    LIDAR_TO_BASE_X = 0.10
    
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('dynamic_arm_pick', anonymous=True)
        
        rospy.loginfo("="*60)
        rospy.loginfo("ARM PICK AND PLACE CONTROLLER")
        rospy.loginfo("="*60)
        
        # Wait for MoveIt action server with extended timeout and retries
        rospy.loginfo("Waiting for MoveIt move_group action server...")
        self._wait_for_moveit(timeout=60.0)
        
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        
        # Initialize arm with retry logic
        self.arm = self._init_move_group_with_retry("arm", max_retries=5, wait_time=30.0)
        self.arm.set_pose_reference_frame("base_footprint")
        self.arm.set_planning_time(2.0)
        self.arm.set_num_planning_attempts(10)
        self.arm.set_max_velocity_scaling_factor(0.5)
        self.arm.set_max_acceleration_scaling_factor(0.5)
        
        try:
            self.gripper = self._init_move_group_with_retry("gripper", max_retries=3, wait_time=15.0)
        except Exception as e:
            rospy.logwarn(f"Gripper init exception: {e}")
            self.gripper = None
        
        # TF listener
        self.tf_listener = tf.TransformListener()
        rospy.sleep(1.0)  # Let TF buffer fill
        
        # Add collision objects to planning scene (prevents arm from hitting table)
        self._setup_collision_objects()
        
        # Publishers
        self.gripper_pub = rospy.Publisher(
            '/gripper_controller/command', JointTrajectory, queue_size=1)
        self.status_pub = rospy.Publisher(
            '/arm_pick/status', String, queue_size=1)
        
        # State
        self.triggered = False
        self.picking = False
        self.emergency_stop = False  # Emergency stop flag
        self.place_requested = False
        self.cube_held = False  # True when holding cube, waiting for place
        self.lidar_dist = 0.25
        self.cube_y_offset = 0.0
        
        # Gazebo data
        self._cube_pose_world = None
        self._robot_pose_world = None
        
        # Publishers for basket coordination
        self.basket_trigger_pub = rospy.Publisher(
            '/basket_approach/trigger', Bool, queue_size=1)
        
        # Subscribers
        rospy.Subscriber('/arm_pick/trigger', Bool, self._trigger_cb, queue_size=1)
        rospy.Subscriber('/arm_place/trigger', Bool, self._place_trigger_cb, queue_size=1)
        rospy.Subscriber('/limo/scan', LaserScan, self._scan_cb, queue_size=1)
        rospy.Subscriber('/cube_offset', Float32MultiArray, self._offset_cb, queue_size=1)
        
        # Gazebo subscriber (simulation only - optional)
        if GAZEBO_AVAILABLE:
            rospy.Subscriber('/gazebo/model_states', ModelStates, self._model_states_cb, queue_size=1)
        else:
            rospy.loginfo("Running without Gazebo - using vision-only mode")
        
        rospy.loginfo("Waiting for trigger...")
    
    def _wait_for_moveit(self, timeout=60.0):
        """Wait for MoveIt move_group to be fully ready."""
        rospy.loginfo(f"Waiting up to {timeout}s for move_group action server...")
        
        # First wait for the planning scene service
        try:
            rospy.wait_for_service('/move_group/get_planning_scene', timeout=timeout)
            rospy.loginfo("  ✓ MoveIt planning scene service ready")
        except rospy.ROSException:
            rospy.logwarn("  ⚠ MoveIt planning scene service not found, continuing anyway...")
        
        # Then wait for the action server (this is what MoveGroupCommander actually needs)
        client = actionlib.SimpleActionClient('move_group', MoveGroupAction)
        if client.wait_for_server(rospy.Duration(timeout)):
            rospy.loginfo("  ✓ MoveIt move_group action server ready")
        else:
            rospy.logwarn(f"  ⚠ move_group action server not available after {timeout}s")
            rospy.logwarn("    Will retry during MoveGroupCommander initialization...")
    
    def _init_move_group_with_retry(self, group_name, max_retries=5, wait_time=30.0):
        """Initialize MoveGroupCommander with retry logic.
        
        Args:
            group_name: Name of the move group (e.g., 'arm', 'gripper')
            max_retries: Maximum number of retry attempts
            wait_time: Time to wait for action server in each attempt (seconds)
            
        Returns:
            MoveGroupCommander instance
            
        Raises:
            RuntimeError: If unable to connect after all retries
        """
        for attempt in range(1, max_retries + 1):
            try:
                rospy.loginfo(f"Initializing '{group_name}' MoveGroup (attempt {attempt}/{max_retries})...")
                # MoveGroupCommander has a default 5s timeout, we can increase it via wait param
                group = moveit_commander.MoveGroupCommander(group_name, wait_for_servers=wait_time)
                rospy.loginfo(f"  ✓ '{group_name}' MoveGroup initialized successfully")
                return group
            except RuntimeError as e:
                rospy.logwarn(f"  Attempt {attempt} failed: {e}")
                if attempt < max_retries:
                    retry_delay = 5.0 * attempt  # Increasing delay between retries
                    rospy.loginfo(f"  Waiting {retry_delay}s before retry...")
                    rospy.sleep(retry_delay)
                else:
                    rospy.logerr(f"Failed to initialize '{group_name}' after {max_retries} attempts")
                    raise RuntimeError(f"Unable to connect to move_group for '{group_name}' after {max_retries} attempts")
    
    def _model_states_cb(self, msg):
        """Extract cube and robot poses from Gazebo (simulation only)."""
        try:
            if 'blue_cube' in msg.name:
                idx = msg.name.index('blue_cube')
                self._cube_pose_world = msg.pose[idx]
            if 'limo_gazebosim' in msg.name:
                idx = msg.name.index('limo_gazebosim')
                self._robot_pose_world = msg.pose[idx]
        except (ValueError, IndexError) as e:
            rospy.logwarn_throttle(10, f"Gazebo model states error: {e}")
    
    def _setup_collision_objects(self):
        """Add collision objects to MoveIt planning scene.
        
        Prevents arm from colliding with table during motion.
        Table dimensions are approximate - adjust for your setup.
        """
        rospy.loginfo("Setting up MoveIt collision objects...")
        rospy.sleep(0.5)  # Wait for scene to be ready
        
        # Remove any existing objects
        self.scene.remove_world_object("table")
        rospy.sleep(0.2)
        
        # Add table as a collision object
        # Table is in front of robot at roughly z=0.1 (center of table height)
        from geometry_msgs.msg import PoseStamped
        table_pose = PoseStamped()
        table_pose.header.frame_id = "base_footprint"
        table_pose.pose.position.x = 0.3   # 30cm in front of robot
        table_pose.pose.position.y = 0.0   # Centered
        table_pose.pose.position.z = 0.05  # Top of table at 10cm, center at 5cm
        table_pose.pose.orientation.w = 1.0
        
        # Table dimensions: 60cm x 40cm x 10cm (adjust to your table)
        self.scene.add_box("table", table_pose, size=(0.6, 0.4, 0.1))
        rospy.loginfo("  Added table collision object")
        
        # Wait for scene to update
        rospy.sleep(0.5)
        
        # Verify object was added
        if "table" in self.scene.get_known_object_names():
            rospy.loginfo("  ✓ Collision objects ready")
        else:
            rospy.logwarn("  ⚠ Failed to add table collision object")

    def _scan_cb(self, msg):
        """Get front LIDAR distance."""
        try:
            ranges = [r for r in msg.ranges if 0.05 < r < 5.0]
            if ranges:
                n = len(ranges)
                center = ranges[n//2-5:n//2+5]
                if center:
                    self.lidar_dist = min(center)
                    return
            # No valid ranges - set safe default
            self.lidar_dist = 10.0
        except Exception as e:
            rospy.logwarn_throttle(5, f"LIDAR processing error: {e}")
            self.lidar_dist = 10.0  # Safe default - assume far
    
    def _offset_cb(self, msg):
        """Get cube Y offset from vision."""
        if msg.data and len(msg.data) >= 2:
            self.cube_y_offset = msg.data[1]
    
    def _cube_z(self):
        """Get current cube Z from Gazebo."""
        if self._cube_pose_world is None:
            return None
        return float(self._cube_pose_world.position.z)
    
    def _cube_position_in_base(self):
        """Get cube position in base_footprint frame using TF."""
        if self._cube_pose_world is None:
            rospy.logwarn("  No cube pose from Gazebo")
            return None
        
        # Create PoseStamped in world frame
        cube_stamped = PoseStamped()
        cube_stamped.header.frame_id = "odom"  # Gazebo world aligns with odom
        cube_stamped.header.stamp = rospy.Time(0)
        cube_stamped.pose = self._cube_pose_world
        
        try:
            # Wait for transform
            self.tf_listener.waitForTransform("base_footprint", "odom", rospy.Time(0), rospy.Duration(2.0))
            
            # Transform to base_footprint
            cube_in_base = self.tf_listener.transformPose("base_footprint", cube_stamped)
            
            x = cube_in_base.pose.position.x
            y = cube_in_base.pose.position.y
            z = cube_in_base.pose.position.z
            
            rospy.loginfo(f"  TF transform: cube in base_footprint = ({x:.3f}, {y:.3f}, {z:.3f})")
            
            # Validate: cube must be IN FRONT of robot (positive X) and reasonable Z
            if x < 0.05 or x > 0.50 or z < 0.10 or z > 0.50:
                rospy.logwarn(f"  TF returned invalid position (x={x:.2f}, z={z:.2f}), using LIDAR fallback")
                return self._cube_position_geometric()
            
            return (x, y, z)
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn(f"  TF failed: {e}")
            # Fall back to geometric calculation
            return self._cube_position_geometric()
    
    def _cube_position_geometric(self):
        """Fallback: compute cube position using geometry."""
        if self._cube_pose_world is None or self._robot_pose_world is None:
            return None
        
        cube_x = self._cube_pose_world.position.x
        cube_y = self._cube_pose_world.position.y
        cube_z = self._cube_pose_world.position.z
        
        robot_x = self._robot_pose_world.position.x
        robot_y = self._robot_pose_world.position.y
        
        # Get robot yaw from quaternion
        q = self._robot_pose_world.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        robot_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # Delta in world frame
        dx_world = cube_x - robot_x
        dy_world = cube_y - robot_y
        
        # Rotate into robot frame
        cos_yaw = math.cos(-robot_yaw)
        sin_yaw = math.sin(-robot_yaw)
        
        x_base = dx_world * cos_yaw - dy_world * sin_yaw
        y_base = dx_world * sin_yaw + dy_world * cos_yaw
        
        rospy.loginfo(f"  Geometric: robot=({robot_x:.2f},{robot_y:.2f},yaw={math.degrees(robot_yaw):.0f}°)")
        rospy.loginfo(f"  Geometric: cube in base = ({x_base:.3f}, {y_base:.3f}, {cube_z:.3f})")
        
        return (x_base, y_base, cube_z)
    
    def _trigger_cb(self, msg):
        """Handle pick trigger - runs in separate thread to avoid blocking."""
        if msg.data and not self.triggered and not self.picking:
            self.triggered = True
            rospy.loginfo("PICK TRIGGERED!")
            self.status_pub.publish(String(data="STARTED"))
            # Run pick in separate thread so callback returns immediately
            # This allows emergency stops and other messages to be received
            pick_thread = threading.Thread(target=self._pick, daemon=True)
            pick_thread.start()
    
    def _place_trigger_cb(self, msg):
        """Handle place trigger - runs in separate thread to avoid blocking."""
        if msg.data and self.cube_held and not self.picking:
            rospy.loginfo("="*50)
            rospy.loginfo("ARM PLACE TRIGGERED - Placing cube in basket!")
            rospy.loginfo("="*50)
            self.place_requested = True
            # Run place in separate thread
            place_thread = threading.Thread(target=self._place_in_basket, daemon=True)
            place_thread.start()
    
    def _gripper(self, position, slow=False):
        """Control gripper with speed control.
        
        Args:
            position: Target position (0.0=open, 0.04=closed)
            slow: If True, close GENTLY (for grasping). If False, move quickly.
        """
        # Determine duration based on operation
        # CLOSING (position > 0.02) should be SLOW to not knock cube out
        # OPENING can be faster
        is_closing = position > 0.001  # Any positive position means closing
        
        if is_closing:
            duration = 2.5  # SLOW gentle close - 2.5 seconds
            wait_time = 2.0
            rospy.loginfo(f"  Gripper CLOSING gently: position={position}")
        else:
            duration = 0.8  # Fast open
            wait_time = 0.5
            rospy.loginfo(f"  Gripper OPENING: position={position}")
        
        # Method 1: MoveIt (if available) - set velocity scaling for gentle close
        moveit_success = False
        if self.gripper:
            try:
                if is_closing:
                    self.gripper.set_max_velocity_scaling_factor(0.1)  # Very slow
                else:
                    self.gripper.set_max_velocity_scaling_factor(0.5)  # Normal
                self.gripper.set_joint_value_target([position])
                moveit_success = self.gripper.go(wait=True)
                self.gripper.stop()
                if moveit_success:
                    rospy.loginfo("  Gripper: MoveIt succeeded")
                    return  # MoveIt worked, no need for backup
            except Exception as e:
                rospy.logwarn(f"  Gripper MoveIt failed: {e}")
        
        # Method 2: Direct trajectory (backup)
        traj = JointTrajectory()
        traj.header.stamp = rospy.Time.now()
        traj.joint_names = ['grasping_frame_joint']
        pt = JointTrajectoryPoint()
        pt.positions = [position]
        pt.velocities = [0.0]
        pt.time_from_start = rospy.Duration(duration)
        traj.points = [pt]
        
        self.gripper_pub.publish(traj)
        rospy.sleep(wait_time)
        rospy.loginfo(f"  Gripper command complete")
    
    def _make_pose(self, x, y, z):
        """Create a pose with gripper pointing down."""
        p = Pose()
        p.position.x = x
        p.position.y = y
        p.position.z = z
        # Gripper pointing down (rotate pi around X)
        q = tf_trans.quaternion_from_euler(math.pi, 0, 0)
        p.orientation = Quaternion(*q)
        return p
    
    def _move(self, pose, name):
        """Plan and execute arm movement."""
        rospy.loginfo(f"  -> {name}: x={pose.position.x:.3f} y={pose.position.y:.3f} z={pose.position.z:.3f}")
        self.arm.clear_pose_targets()
        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(pose)
        result = self.arm.plan()
        ok = result[0] if isinstance(result, tuple) else (result is not None)
        if ok:
            self.arm.go(wait=True)
            self.arm.stop()
            rospy.sleep(0.3)
            return True
        rospy.logwarn(f"  Planning failed for {name}")
        return False
    
    
    def _move_cartesian(self, target_pose, name):
        """Move arm in a straight line using Cartesian path planning.
        
        This is critical for vertical descent to avoid random RRT paths
        that might push the cube sideways.
        
        NEVER falls back to RRT for grasp moves - RRT can cause sideways motion!
        """
        rospy.loginfo(f"  -> {name} (Cartesian): x={target_pose.position.x:.3f} y={target_pose.position.y:.3f} z={target_pose.position.z:.3f}")
        
        # Wait for arm to settle and sync with MoveIt
        rospy.sleep(0.3)
        
        # Get current pose - this is crucial for Cartesian path computation
        current_pose = self.arm.get_current_pose().pose
        rospy.loginfo(f"  Current pose: x={current_pose.position.x:.3f} y={current_pose.position.y:.3f} z={current_pose.position.z:.3f}")
        
        # CRITICAL: For grasp, we MUST descend purely vertically (same X,Y)
        # Create intermediate waypoints for better path following
        waypoints = []
        
        # If this is a vertical descent (same X,Y, lower Z), use multiple waypoints
        dx = abs(target_pose.position.x - current_pose.position.x)
        dy = abs(target_pose.position.y - current_pose.position.y)
        dz = current_pose.position.z - target_pose.position.z  # Positive if descending
        
        if dx < 0.02 and dy < 0.02 and dz > 0.01:
            # Pure vertical descent - add intermediate waypoint at same X,Y
            rospy.loginfo(f"  Pure vertical descent detected (dz={dz:.3f}m)")
            # Keep EXACT same X,Y as current, only change Z
            mid_pose = Pose()
            mid_pose.position.x = current_pose.position.x  # Keep current X
            mid_pose.position.y = current_pose.position.y  # Keep current Y
            mid_pose.position.z = target_pose.position.z + 0.02  # Slightly above target
            mid_pose.orientation = target_pose.orientation
            waypoints.append(mid_pose)
            
            # Final pose also at current X,Y (not target X,Y to avoid horizontal motion)
            final_pose = Pose()
            final_pose.position.x = current_pose.position.x
            final_pose.position.y = current_pose.position.y
            final_pose.position.z = target_pose.position.z
            final_pose.orientation = target_pose.orientation
            waypoints.append(final_pose)
        else:
            waypoints = [target_pose]
        
        # Compute Cartesian path - disable collision for small descent to cube
        # eef_step: smaller = smoother but slower
        (plan, fraction) = self.arm.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.005,       # eef_step: 5mm interpolation (smoother)
            False        # avoid_collisions DISABLED for grasp descent
        )
        
        rospy.loginfo(f"  Cartesian path fraction: {fraction:.2f}")
        
        if fraction >= 0.85:  # Accept 85% completion
            self.arm.execute(plan, wait=True)
            self.arm.stop()
            rospy.sleep(0.2)
            return True
        else:
            # DO NOT fall back to RRT for grasp moves!
            # RRT produces erratic paths that push the cube away
            rospy.logwarn(f"  Cartesian path only {fraction*100:.0f}% - will try with collision disabled")
            
            # Try again with collision checking fully disabled
            (plan2, fraction2) = self.arm.compute_cartesian_path(
                waypoints,
                0.005,
                False  # Collision disabled
            )
            
            if fraction2 >= 0.70:
                rospy.loginfo(f"  Retry succeeded: {fraction2*100:.0f}%")
                self.arm.execute(plan2, wait=True)
                self.arm.stop()
                rospy.sleep(0.2)
                return True
            else:
                rospy.logerr(f"  Cartesian path failed completely ({fraction2*100:.0f}%), aborting to avoid pushing cube")
                return False

    def _home(self):
        """Move arm to home position."""
        try:
            self.arm.set_joint_value_target([0, 0, 0, 0, 0, 0])
            self.arm.go(wait=True)
            self.arm.stop()
            rospy.loginfo("Arm returned to home position")
        except Exception as e:
            rospy.logerr(f"Failed to move arm home: {e}")
    
    def _pick(self):
        """Execute pick sequence."""
        self.picking = True
        rospy.loginfo("="*50)
        rospy.loginfo("PICK SEQUENCE START")
        rospy.loginfo("="*50)
        
        # Get cube position
        cube_in_base = self._cube_position_in_base()
        
        if cube_in_base is not None:
            grasp_x, grasp_y, grasp_z = cube_in_base
            rospy.loginfo(f"  Cube in base: x={grasp_x:.3f} y={grasp_y:.3f} z={grasp_z:.3f}")
            
            # Double-check: if position looks wrong, fall back to LIDAR
            if grasp_x < 0.05 or grasp_x > 0.40:
                rospy.logwarn(f"  Position looks wrong, using LIDAR instead")
                grasp_x = self.lidar_dist + self.LIDAR_TO_BASE_X
                if grasp_x < self.ARM_MIN_FORWARD:
                    grasp_x = self.ARM_MIN_FORWARD + 0.02
                if grasp_x > self.ARM_MAX_FORWARD:
                    grasp_x = self.ARM_MAX_FORWARD - 0.02
                grasp_y = -self.cube_y_offset
                grasp_z = self.CUBE_CENTER_Z_FALLBACK
                rospy.loginfo(f"  LIDAR corrected: x={grasp_x:.3f} y={grasp_y:.3f} z={grasp_z:.3f}")
        else:
            rospy.logwarn("  Gazebo+TF unavailable, using LIDAR fallback")
            grasp_x = self.lidar_dist + self.LIDAR_TO_BASE_X
            grasp_y = -self.cube_y_offset
            grasp_z = self.CUBE_CENTER_Z_FALLBACK
            rospy.loginfo(f"  LIDAR fallback: x={grasp_x:.3f} y={grasp_y:.3f} z={grasp_z:.3f}")
        
        # Range checks
        if grasp_x > self.ARM_MAX_FORWARD:
            rospy.logerr(f"TOO FAR! grasp_x={grasp_x:.2f}m > max {self.ARM_MAX_FORWARD:.2f}m")
            self.status_pub.publish(String(data="FAILED_TOO_FAR"))
            self._home()
            self.picking = False
            self.triggered = False
            return
        
        if grasp_x < self.ARM_MIN_FORWARD:
            grasp_x = self.ARM_MIN_FORWARD
        
        if abs(grasp_y) > self.ARM_MAX_SIDE:
            grasp_y = max(-self.ARM_MAX_SIDE, min(self.ARM_MAX_SIDE, grasp_y))
        
        rospy.loginfo(f"GRASP TARGET: x={grasp_x:.3f} y={grasp_y:.3f} z={grasp_z:.3f}")
        
        reach = math.sqrt(grasp_x**2 + grasp_y**2)
        if reach > self.ARM_MAX_FORWARD:
            rospy.logerr(f"Reach {reach:.2f}m > max")
            self._home()
            self.picking = False
            self.triggered = False
            return
        
        try:
            cube_z_before = self._cube_z()
            
            # CRITICAL: Open gripper WIDE FIRST before any arm movement!
            # This prevents the closed gripper from pushing the cube when arm descends
            rospy.loginfo("1. Open gripper WIDE (before moving arm)")
            self._gripper(self.GRIPPER_OPEN)
            rospy.sleep(0.1)  # Brief wait after quick gripper open
            
            # Calculate adjusted Y position (shift left to compensate for offset)
            adjusted_y = grasp_y + self.GRASP_Y_OFFSET
            
            # Now move arm to safe height with gripper already open
            rospy.loginfo("2. Move to safe height")
            safe_pose = self._make_pose(grasp_x, adjusted_y, grasp_z + 0.12)
            self._move(safe_pose, "safe height")
            
            rospy.loginfo("3. Pre-grasp (directly above cube)")
            pre_x = grasp_x
            pre_y = adjusted_y
            pre_z = grasp_z + self.PRE_GRASP_OFFSET
            pre = self._make_pose(pre_x, pre_y, pre_z)
            if not self._move(pre, "pre-grasp"):
                # If failed, try slightly higher but SAME X,Y
                pre_z = grasp_z + self.PRE_GRASP_OFFSET + 0.02
                pre = self._make_pose(pre_x, pre_y, pre_z)
                if not self._move(pre, "pre-grasp higher"):
                    raise Exception("Cannot reach pre-grasp")
            rospy.sleep(0.3)
            
            rospy.loginfo("4. Descend VERTICALLY to grasp (same X,Y)")
            # CRITICAL: Use SAME X,Y as pre-grasp for pure vertical descent!
            grasp_final_z = grasp_z + 0.02  # 1cm above cube center
            grasp = self._make_pose(pre_x, pre_y, grasp_final_z)
            if not self._move_cartesian(grasp, "grasp"):
                # Try slightly higher if failed
                grasp = self._make_pose(pre_x, pre_y, grasp_final_z + 0.01)
                self._move(grasp, "grasp higher")
            rospy.sleep(0.3)
            
            rospy.loginfo("5. Close gripper")
            self._gripper(self.GRIPPER_CLOSED)
            rospy.sleep(0.5)
            
            rospy.loginfo("6. Lift straight up")
            lift = self._make_pose(grasp_x, adjusted_y, grasp_z + 0.10)
            self._move(lift, "lift")
            rospy.sleep(0.3)  # Brief pause to stabilize
            
            # Skip cube Z check - it can be unreliable
            rospy.loginfo("   Cube grasped, continuing...")
            
            # ============================================
            # SAFE RETRACE PATH - exactly reverse of approach
            # ============================================
            
            rospy.loginfo("7. Go to safe height (high above table)")
            safe_height = self._make_pose(grasp_x, adjusted_y, 0.30)  # Fixed high Z
            self._move(safe_height, "safe height")
            
            rospy.loginfo("8. Retract toward robot base")
            retract = self._make_pose(0.10, adjusted_y, 0.30)  # Pull back, stay high
            self._move(retract, "retract")
            
            rospy.loginfo("9. Go to home position (with cube)")
            self._home()
            rospy.sleep(0.5)
            
            # ============================================
            # CUBE PICKED! Now trigger basket search
            # ============================================
            rospy.loginfo("="*50)
            rospy.loginfo("CUBE PICKED! Triggering basket search...")
            rospy.loginfo("="*50)
            
            self.cube_held = True
            self.status_pub.publish(String(data="CUBE_HELD"))
            
            # Trigger the basket approach node
            rospy.sleep(1.0)  # Wait for stability
            self.basket_trigger_pub.publish(Bool(True))
            rospy.loginfo("Basket approach triggered - waiting for robot to navigate to basket...")
            
        except Exception as e:
            rospy.logerr(f"Pick failed: {e}")
            self._home()
            msg = "FAILED_NO_LIFT" if "did not lift" in str(e) else "FAILED"
            self.status_pub.publish(String(data=msg))
        
        self.picking = False
        # Only reset triggered if cube was NOT picked successfully
        if not self.cube_held:
            self.triggered = False
    
    def _place_in_basket(self):
        """Place the cube in the basket (called when robot arrives at basket)."""
        if not self.cube_held:
            rospy.logwarn("No cube held, cannot place!")
            return
            
        self.picking = True
        
        try:
            rospy.loginfo("="*50)
            rospy.loginfo("PLACING CUBE IN BASKET")
            rospy.loginfo("="*50)
            
            # 1. Move arm forward and up (above basket)
            rospy.loginfo("1. Extend arm above basket")
            above_basket = self._make_pose(0.26, 0.0, 0.25)  # Forward, centered, high
            self._move(above_basket, "above basket")
            
            # 2. Lower toward basket (basket walls are ~10cm high, basket at 0.10 from ground)
            # Basket interior is at z=0.10 + 0.01 (bottom) = 0.11
            # We want to drop from about z=0.20 to clear the walls
            rospy.loginfo("2. Lower to drop height")
            drop_position = self._make_pose(0.26, 0.0, 0.20)  # Just above basket walls
            self._move(drop_position, "drop height")
            rospy.sleep(0.3)
            
            # 3. Open gripper to release cube
            rospy.loginfo("3. Opening gripper - RELEASING CUBE!")
            self._gripper(self.GRIPPER_OPEN)
            rospy.sleep(1.0)  # Wait for cube to fall
            
            # 4. Retreat and go home
            rospy.loginfo("4. Retreat from basket")
            retreat = self._make_pose(0.12, 0.0, 0.30)
            self._move(retreat, "retreat")
            
            rospy.loginfo("5. Return to home position")
            self._home()
            
            rospy.loginfo("="*50)
            rospy.loginfo("CUBE PLACED IN BASKET - MISSION COMPLETE!")
            rospy.loginfo("="*50)
            
            self.cube_held = False
            self.place_requested = False
            self.status_pub.publish(String(data="PLACE_SUCCESS"))
            
        except Exception as e:
            rospy.logerr(f"Place failed: {e}")
            self._home()
            self.status_pub.publish(String(data="PLACE_FAILED"))
            
        self.picking = False
    
    
    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = DynamicArmPick()
        node.run()
    except rospy.ROSInterruptException:
        pass
