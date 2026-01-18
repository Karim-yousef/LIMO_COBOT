#!/usr/bin/env python3
"""
MISSION CONTROLLER - Event-Driven Mission Coordinator

ARCHITECTURE:
- Event-driven coordination (NO SLEEP-BASED SEQUENCING!)
- Single point of control for mission phases
- cmd_vel arbitration through phase activation
- Retry logic for failed operations
- Timeout handling with configurable limits
- Emergency stop support

Flow:
1. Wait for system ready (sensors, MoveIt)
2. Trigger blue cube search/approach
3. Wait for cube reached signal
4. Trigger arm pick (with retry on failure)
5. Wait for cube held signal
6. Trigger basket approach
7. Wait for basket reached signal
8. Trigger arm place (with retry on failure)
9. Mission complete!

Usage:
    rosrun limo_vision mission_controller.py
    
Topics Published:
    /blue_cube_approach/trigger (Bool) - Start cube search
    /basket_approach/trigger (Bool) - Start basket search
    /arm_pick/trigger (Bool) - Trigger pick operation
    /arm_place/trigger (Bool) - Trigger place operation
    /mission/status (String) - Current mission status
    /emergency_stop (Bool) - Emergency stop broadcast

Topics Subscribed:
    /blue_cube_approach/status (String) - Cube approach feedback
    /arm_pick/status (String) - Arm operation feedback
    /basket_approach/status (String) - Basket approach feedback
    /emergency_stop (Bool) - Emergency stop input

Author: Mission controller for real-robot deployment
"""

import rospy
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist
import sys

# Import shared status constants
try:
    from mission_status import Status, CUBE_APPROACH_DONE_STATUSES, ARM_PICK_DONE_STATUSES, \
        ARM_PICK_FAILED_STATUSES, BASKET_APPROACH_DONE_STATUSES, ARM_PLACE_DONE_STATUSES, \
        ARM_PLACE_FAILED_STATUSES
    USING_STATUS_CONSTANTS = True
except ImportError:
    USING_STATUS_CONSTANTS = False
    rospy.logwarn("mission_status.py not found - using hardcoded status strings")


class MissionController:
    """Event-driven mission coordinator with retry logic."""
    
    # Mission phases
    INIT = "INIT"
    FIND_CUBE = "FIND_CUBE"
    PICK_CUBE = "PICK_CUBE"
    FIND_BASKET = "FIND_BASKET"
    PLACE_CUBE = "PLACE_CUBE"
    COMPLETE = "COMPLETE"
    ABORTED = "ABORTED"
    
    def __init__(self):
        rospy.init_node('mission_controller')
        
        rospy.loginfo("=" * 60)
        rospy.loginfo("MISSION CONTROLLER - Event-Driven Coordinator")
        rospy.loginfo("  + Retry logic for failed operations")
        rospy.loginfo("  + Timeout handling")
        rospy.loginfo("  + cmd_vel arbitration")
        rospy.loginfo("=" * 60)
        
        # State
        self.phase = self.INIT
        self.emergency_stop = False
        self.phase_start_time = None
        
        # ============================================
        # CONFIGURABLE PARAMETERS (can be ROS params)
        # ============================================
        self.FIND_CUBE_TIMEOUT = rospy.get_param('~find_cube_timeout', 180.0)
        self.PICK_CUBE_TIMEOUT = rospy.get_param('~pick_cube_timeout', 60.0)
        self.FIND_BASKET_TIMEOUT = rospy.get_param('~find_basket_timeout', 180.0)
        self.PLACE_CUBE_TIMEOUT = rospy.get_param('~place_cube_timeout', 60.0)
        
        # Retry configuration
        self.MAX_PICK_RETRIES = rospy.get_param('~max_pick_retries', 3)
        self.MAX_PLACE_RETRIES = rospy.get_param('~max_place_retries', 3)
        self.RETRY_DELAY = rospy.get_param('~retry_delay', 2.0)
        
        # Status tracking
        self.cube_approach_status = "NOT_READY"  # Wait for actual IDLE from node
        self.arm_status = "NOT_READY"  # Wait for actual status from node
        self.basket_approach_status = "NOT_READY"  # Wait for actual IDLE from node
        
        # Retry counters
        self.pick_attempts = 0
        self.place_attempts = 0
        
        # ============================================
        # PUBLISHERS
        # ============================================
        self.blue_trigger_pub = rospy.Publisher(
            '/blue_cube_approach/trigger', Bool, queue_size=1, latch=True)
        self.arm_pick_trigger_pub = rospy.Publisher(
            '/arm_pick/trigger', Bool, queue_size=1, latch=True)
        self.basket_trigger_pub = rospy.Publisher(
            '/basket_approach/trigger', Bool, queue_size=1, latch=True)
        self.arm_place_trigger_pub = rospy.Publisher(
            '/arm_place/trigger', Bool, queue_size=1, latch=True)
        self.emergency_stop_pub = rospy.Publisher(
            '/emergency_stop', Bool, queue_size=1, latch=True)
        self.cmd_pub = rospy.Publisher(
            '/cmd_vel', Twist, queue_size=1)
        self.mission_status_pub = rospy.Publisher(
            '/mission/status', String, queue_size=1, latch=True)
        
        # ============================================
        # SUBSCRIBERS
        # ============================================
        rospy.Subscriber('/blue_cube_approach/status', String, 
                        self._cube_approach_status_cb, queue_size=1)
        rospy.Subscriber('/arm_pick/status', String,
                        self._arm_status_cb, queue_size=1)
        rospy.Subscriber('/basket_approach/status', String,
                        self._basket_approach_status_cb, queue_size=1)
        rospy.Subscriber('/emergency_stop', Bool,
                        self._emergency_stop_cb, queue_size=1)
        
        rospy.loginfo("Mission controller initialized")
        rospy.loginfo(f"  Pick retries: {self.MAX_PICK_RETRIES}")
        rospy.loginfo(f"  Place retries: {self.MAX_PLACE_RETRIES}")
        
    def _cube_approach_status_cb(self, msg):
        """Track blue cube approach status."""
        if msg.data != self.cube_approach_status:
            rospy.loginfo(f"[CUBE APPROACH] Status changed: {self.cube_approach_status} -> {msg.data}")
        self.cube_approach_status = msg.data
        
    def _arm_status_cb(self, msg):
        """Track arm status."""
        if msg.data != self.arm_status:
            rospy.loginfo(f"[ARM] Status changed: {self.arm_status} -> {msg.data}")
        self.arm_status = msg.data
        
    def _basket_approach_status_cb(self, msg):
        """Track basket approach status."""
        if msg.data != self.basket_approach_status:
            rospy.loginfo(f"[BASKET APPROACH] Status changed: {self.basket_approach_status} -> {msg.data}")
        self.basket_approach_status = msg.data
        
    def _emergency_stop_cb(self, msg):
        """Handle emergency stop - deactivate ALL nodes immediately."""
        if msg.data and not self.emergency_stop:
            rospy.logwarn("!" * 50)
            rospy.logwarn("EMERGENCY STOP - Mission aborted!")
            rospy.logwarn("!" * 50)
            self.emergency_stop = True
            self.phase = self.ABORTED
            
            # Stop robot immediately
            self.cmd_pub.publish(Twist())
            
            # Deactivate ALL nodes by sending False to all trigger topics
            rospy.logwarn("Deactivating all nodes...")
            self.blue_trigger_pub.publish(Bool(data=False))
            self.arm_pick_trigger_pub.publish(Bool(data=False))
            self.basket_trigger_pub.publish(Bool(data=False))
            self.arm_place_trigger_pub.publish(Bool(data=False))
            
            # Keep publishing stop commands for a bit to ensure they're received
            for _ in range(5):
                self.cmd_pub.publish(Twist())
                rospy.sleep(0.1)
            
            self._publish_status("EMERGENCY_STOP")
            rospy.logwarn("All nodes deactivated - robot stopped")
            
    def _publish_status(self, status):
        """Publish mission status."""
        self.mission_status_pub.publish(String(data=status))
        rospy.loginfo(f"[MISSION] {status}")
        
    def _check_timeout(self, timeout):
        """Check if current phase has timed out."""
        if self.phase_start_time is None:
            return False
        elapsed = (rospy.Time.now() - self.phase_start_time).to_sec()
        return elapsed > timeout
        
    def _start_phase(self, phase):
        """Start a new mission phase."""
        self.phase = phase
        self.phase_start_time = rospy.Time.now()
        self._publish_status(f"Phase: {phase}")
        
    def _wait_for_system_ready(self):
        """Wait for all required systems to be ready."""
        rospy.loginfo("=" * 60)
        rospy.loginfo("Waiting for system ready...")
        rospy.loginfo("=" * 60)
        
        # Check for camera
        try:
            rospy.loginfo("Checking camera...")
            rospy.wait_for_message('/limo/color/image_raw', rospy.AnyMsg, timeout=30.0)
            rospy.loginfo("✓ Camera ready")
        except rospy.ROSException:
            rospy.logwarn("⚠ Camera not detected - continuing anyway")
            
        # Check for LIDAR
        try:
            rospy.loginfo("Checking LIDAR...")
            rospy.wait_for_message('/limo/scan', rospy.AnyMsg, timeout=10.0)
            rospy.loginfo("✓ LIDAR ready")
        except rospy.ROSException:
            rospy.logwarn("⚠ LIDAR not detected - continuing anyway")
        
        # Wait for MoveIt
        try:
            rospy.loginfo("Checking MoveIt...")
            rospy.wait_for_service('/move_group/get_planning_scene', timeout=30.0)
            rospy.loginfo("✓ MoveIt ready")
        except rospy.ROSException:
            rospy.logwarn("⚠ MoveIt service not found - arm may not work")
        
        # Brief delay for node connections
        rospy.sleep(2.0)
        rospy.loginfo("System check complete - starting mission")

    def _is_status_in_list(self, status, status_list):
        """Check if status matches any in the list (case-insensitive)."""
        return status.upper() in [s.upper() for s in status_list]

    def run(self):
        """Main mission loop with retry logic."""
        rate = rospy.Rate(10)
        
        # Wait for system ready
        self._wait_for_system_ready()
        
        # =====================================================
        # PHASE 1: FIND AND APPROACH BLUE CUBE
        # =====================================================
        self._start_phase(self.FIND_CUBE)
        rospy.loginfo("=" * 60)
        rospy.loginfo("PHASE 1: FIND AND APPROACH BLUE CUBE")
        rospy.loginfo("=" * 60)
        
        # Wait for blue_cube node to be ready (publishing IDLE)
        rospy.loginfo("Waiting for blue_cube node to be ready...")
        rospy.loginfo(f"  Current status: {self.cube_approach_status}")
        ready_timeout = rospy.Time.now() + rospy.Duration(30.0)
        wait_count = 0
        while self.cube_approach_status != "IDLE" and rospy.Time.now() < ready_timeout:
            if rospy.is_shutdown():
                return
            wait_count += 1
            if wait_count % 4 == 0:  # Every 2 seconds
                rospy.loginfo(f"  Still waiting... status={self.cube_approach_status}")
            rospy.sleep(0.5)
        
        if self.cube_approach_status != "IDLE":
            rospy.logwarn("Blue cube node not responding - triggering anyway")
        else:
            rospy.loginfo("Blue cube node ready - sending trigger")
        
        # Publish trigger multiple times to ensure reception
        for i in range(3):
            self.blue_trigger_pub.publish(Bool(data=True))
            rospy.sleep(0.2)
        
        done_statuses = CUBE_APPROACH_DONE_STATUSES if USING_STATUS_CONSTANTS else \
            ["CUBE_REACHED", "DONE", "FINAL_APPROACH_DONE"]
        
        while not rospy.is_shutdown() and self.phase == self.FIND_CUBE:
            if self.emergency_stop:
                return
                
            if self._is_status_in_list(self.cube_approach_status, done_statuses):
                rospy.loginfo("✓ Cube approach complete!")
                # Deactivate cube approach node
                self.blue_trigger_pub.publish(Bool(data=False))
                break
                
            if self._check_timeout(self.FIND_CUBE_TIMEOUT):
                rospy.logerr("✗ TIMEOUT: Failed to find cube")
                self._abort_mission("Cube search timeout")
                return
                
            rate.sleep()
            
        # =====================================================
        # PHASE 2: PICK UP THE CUBE (WITH RETRY)
        # =====================================================
        self._start_phase(self.PICK_CUBE)
        rospy.loginfo("=" * 60)
        rospy.loginfo("PHASE 2: PICK UP THE CUBE")
        rospy.loginfo("=" * 60)
        
        pick_success = False
        self.pick_attempts = 0
        
        pick_done = ARM_PICK_DONE_STATUSES if USING_STATUS_CONSTANTS else ["CUBE_HELD"]
        pick_failed = ARM_PICK_FAILED_STATUSES if USING_STATUS_CONSTANTS else ["PICK_FAILED", "FAILED"]
        
        while self.pick_attempts < self.MAX_PICK_RETRIES and not pick_success:
            self.pick_attempts += 1
            rospy.loginfo(f"Pick attempt {self.pick_attempts}/{self.MAX_PICK_RETRIES}")
            
            # Reset arm status and trigger
            self.arm_status = "NOT_READY"  # Wait for actual status from node
            rospy.sleep(0.5)
            self.arm_pick_trigger_pub.publish(Bool(data=True))
            
            # Wait for result
            attempt_start = rospy.Time.now()
            while not rospy.is_shutdown() and self.phase == self.PICK_CUBE:
                if self.emergency_stop:
                    return
                    
                if self._is_status_in_list(self.arm_status, pick_done):
                    rospy.loginfo("✓ Cube picked up!")
                    pick_success = True
                    break
                    
                if self._is_status_in_list(self.arm_status, pick_failed):
                    rospy.logwarn(f"✗ Pick attempt {self.pick_attempts} failed")
                    break
                    
                # Check timeout for this attempt
                if (rospy.Time.now() - attempt_start).to_sec() > self.PICK_CUBE_TIMEOUT:
                    rospy.logwarn(f"✗ Pick attempt {self.pick_attempts} timed out")
                    break
                    
                rate.sleep()
            
            if not pick_success and self.pick_attempts < self.MAX_PICK_RETRIES:
                rospy.loginfo(f"Repositioning before retry...")
                # Back up slightly to get a fresh approach
                self._reposition_for_retry()
                rospy.loginfo(f"Retrying in {self.RETRY_DELAY}s...")
                rospy.sleep(self.RETRY_DELAY)
        
        if not pick_success:
            self._abort_mission(f"Pick failed after {self.MAX_PICK_RETRIES} attempts")
            return
            
        # =====================================================
        # PHASE 3: FIND AND APPROACH BASKET
        # =====================================================
        self._start_phase(self.FIND_BASKET)
        rospy.loginfo("=" * 60)
        rospy.loginfo("PHASE 3: FIND AND APPROACH RED BASKET")
        rospy.loginfo("=" * 60)
        
        # Wait for basket node to be ready
        rospy.loginfo("Waiting for basket node to be ready...")
        ready_timeout = rospy.Time.now() + rospy.Duration(30.0)
        while self.basket_approach_status != "IDLE" and rospy.Time.now() < ready_timeout:
            if rospy.is_shutdown():
                return
            rospy.sleep(0.5)
        
        rospy.loginfo("Basket node ready - sending trigger")
        for i in range(3):
            self.basket_trigger_pub.publish(Bool(data=True))
            rospy.sleep(0.2)
        
        basket_done = BASKET_APPROACH_DONE_STATUSES if USING_STATUS_CONSTANTS else \
            ["BASKET_REACHED", "DONE"]
        
        while not rospy.is_shutdown() and self.phase == self.FIND_BASKET:
            if self.emergency_stop:
                return
                
            if self._is_status_in_list(self.basket_approach_status, basket_done):
                rospy.loginfo("✓ Basket approach complete!")
                # Deactivate basket approach
                self.basket_trigger_pub.publish(Bool(data=False))
                break
                
            if self._check_timeout(self.FIND_BASKET_TIMEOUT):
                rospy.logerr("✗ TIMEOUT: Failed to find basket")
                self._abort_mission("Basket search timeout")
                return
                
            rate.sleep()
            
        # =====================================================
        # PHASE 4: PLACE CUBE IN BASKET (WITH RETRY)
        # =====================================================
        self._start_phase(self.PLACE_CUBE)
        rospy.loginfo("=" * 60)
        rospy.loginfo("PHASE 4: PLACE CUBE IN BASKET")
        rospy.loginfo("=" * 60)
        
        place_success = False
        self.place_attempts = 0
        
        place_done = ARM_PLACE_DONE_STATUSES if USING_STATUS_CONSTANTS else \
            ["PLACE_DONE", "MISSION_COMPLETE"]
        place_failed = ARM_PLACE_FAILED_STATUSES if USING_STATUS_CONSTANTS else \
            ["PLACE_FAILED", "FAILED"]
        
        while self.place_attempts < self.MAX_PLACE_RETRIES and not place_success:
            self.place_attempts += 1
            rospy.loginfo(f"Place attempt {self.place_attempts}/{self.MAX_PLACE_RETRIES}")
            
            # Reset arm status and trigger
            self.arm_status = "NOT_READY"  # Wait for actual status from node
            rospy.sleep(0.5)
            self.arm_place_trigger_pub.publish(Bool(data=True))
            
            # Wait for result
            attempt_start = rospy.Time.now()
            while not rospy.is_shutdown() and self.phase == self.PLACE_CUBE:
                if self.emergency_stop:
                    return
                    
                if self._is_status_in_list(self.arm_status, place_done):
                    rospy.loginfo("✓ Cube placed!")
                    place_success = True
                    break
                    
                if self._is_status_in_list(self.arm_status, place_failed):
                    rospy.logwarn(f"✗ Place attempt {self.place_attempts} failed")
                    break
                    
                if (rospy.Time.now() - attempt_start).to_sec() > self.PLACE_CUBE_TIMEOUT:
                    rospy.logwarn(f"✗ Place attempt {self.place_attempts} timed out")
                    break
                    
                rate.sleep()
            
            if not place_success and self.place_attempts < self.MAX_PLACE_RETRIES:
                rospy.loginfo(f"Repositioning before retry...")
                self._reposition_for_retry()
                rospy.loginfo(f"Retrying in {self.RETRY_DELAY}s...")
                rospy.sleep(self.RETRY_DELAY)
        
        if not place_success:
            self._abort_mission(f"Place failed after {self.MAX_PLACE_RETRIES} attempts")
            return
            
        # =====================================================
        # MISSION COMPLETE!
        # =====================================================
        self._start_phase(self.COMPLETE)
        rospy.loginfo("=" * 60)
        rospy.loginfo("🎉 MISSION COMPLETE! 🎉")
        rospy.loginfo(f"  Pick attempts: {self.pick_attempts}")
        rospy.loginfo(f"  Place attempts: {self.place_attempts}")
        rospy.loginfo("=" * 60)
        self._publish_status("MISSION_COMPLETE")
        
    def _reposition_for_retry(self):
        """Back up slightly before retrying pick/place operation.
        
        This helps when the robot's position was slightly off during
        the failed attempt. Backing up gives it a fresh approach angle.
        """
        rospy.loginfo("Repositioning: backing up 0.2m")
        twist = Twist()
        twist.linear.x = -0.10  # Reverse at 0.1 m/s
        
        # Back up for 2 seconds (20 iterations at 10Hz)
        for _ in range(20):
            if self.emergency_stop:
                break
            self.cmd_pub.publish(twist)
            rospy.sleep(0.1)
        
        # Stop
        self.cmd_pub.publish(Twist())
        rospy.loginfo("Repositioning complete")

    def _abort_mission(self, reason):
        """Abort mission cleanly."""
        rospy.logerr(f"MISSION ABORTED: {reason}")
        self.phase = self.ABORTED
        self._publish_status(f"ABORTED: {reason}")
        
        # Stop robot
        self.cmd_pub.publish(Twist())
        
        # Deactivate all nodes
        self.blue_trigger_pub.publish(Bool(data=False))
        self.basket_trigger_pub.publish(Bool(data=False))


def main():
    try:
        controller = MissionController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Mission controller error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
