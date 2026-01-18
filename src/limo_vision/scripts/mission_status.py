#!/usr/bin/env python3
"""
SHARED MISSION STATUS CONSTANTS

This module defines standardized status codes used across all mission nodes.
Using constants prevents typo-related bugs and enables IDE autocompletion.

Usage:
    from mission_status import Status, Phase
    
    self.status_pub.publish(String(data=Status.CUBE_REACHED))
"""

from enum import Enum, auto


class Phase(Enum):
    """Mission phases - used by mission controller."""
    INIT = auto()
    FIND_CUBE = auto()
    PICK_CUBE = auto()
    FIND_BASKET = auto()
    PLACE_CUBE = auto()
    COMPLETE = auto()
    ABORTED = auto()


class Status:
    """Standardized status strings for inter-node communication.
    
    Using string constants (not Enum) for ROS String message compatibility.
    """
    # General states
    IDLE = "IDLE"
    STARTED = "STARTED"
    DONE = "DONE"
    FAILED = "FAILED"
    
    # Blue cube approach states
    SEARCHING = "SEARCHING"
    CUBE_DETECTED = "CUBE_DETECTED"
    APPROACHING = "APPROACHING"
    FINAL_APPROACH = "FINAL_APPROACH"
    CUBE_REACHED = "CUBE_REACHED"
    
    # Arm states
    PICKING = "PICKING"
    CUBE_HELD = "CUBE_HELD"
    PICK_FAILED = "PICK_FAILED"
    PLACING = "PLACING"
    PLACE_DONE = "PLACE_DONE"
    PLACE_FAILED = "PLACE_FAILED"
    
    # Basket approach states  
    BACKING_UP = "BACKING_UP"
    BASKET_DETECTED = "BASKET_DETECTED"
    BASKET_REACHED = "BASKET_REACHED"
    
    # Mission states
    MISSION_COMPLETE = "MISSION_COMPLETE"
    MISSION_ABORTED = "MISSION_ABORTED"
    EMERGENCY_STOP = "EMERGENCY_STOP"


class State(Enum):
    """State machine states - use instead of string comparisons.
    
    Usage:
        self.state = State.IDLE
        if self.state == State.SEARCH:
            ...
    """
    # Common states
    IDLE = auto()
    
    # Blue cube states
    SEARCH = auto()
    APPROACH_CUBE = auto()
    FINAL_APPROACH = auto()
    BLIND_APPROACH = auto()
    ALIGNING = auto()
    
    # Basket states
    BACKUP = auto()
    APPROACH_BASKET = auto()
    
    # Terminal states
    DONE = auto()
    ABORTED = auto()


# Status codes that indicate phase completion (for mission controller)
CUBE_APPROACH_DONE_STATUSES = [Status.CUBE_REACHED, Status.DONE, "FINAL_APPROACH_DONE"]
ARM_PICK_DONE_STATUSES = [Status.CUBE_HELD]
ARM_PICK_FAILED_STATUSES = [Status.PICK_FAILED, Status.FAILED]
BASKET_APPROACH_DONE_STATUSES = [Status.BASKET_REACHED, Status.DONE]
ARM_PLACE_DONE_STATUSES = [Status.PLACE_DONE, Status.MISSION_COMPLETE]
ARM_PLACE_FAILED_STATUSES = [Status.PLACE_FAILED, Status.FAILED]
