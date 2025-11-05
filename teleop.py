#!/usr/bin/env python3
"""
Enhanced keyboard control for SO100/SO101 robot with full 3D IK
Uses ENU coordinate system: X=forward, Y=left, Z=up
Supports full 3D end effector control with inverse kinematics
"""

import time
import logging
import traceback
import math

# Set up logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Joint calibration coefficients - manually edited
# Format: [joint_name, zero_position_offset(degrees), scale_factor]
JOINT_CALIBRATION = [
    ['shoulder_pan', 6.0, 1.0],      # Joint 1: zero position offset, scale factor
    ['shoulder_lift', 2.0, 0.97],     # Joint 2: zero position offset, scale factor
    ['elbow_flex', 0.0, 1.05],        # Joint 3: zero position offset, scale factor
    ['wrist_flex', 0.0, 0.94],        # Joint 4: zero position offset, scale factor
    ['wrist_roll', 0.0, 0.5],        # Joint 5: zero position offset, scale factor
    ['gripper', 0.0, 1.0],           # Joint 6: zero position offset, scale factor
]

def apply_joint_calibration(joint_name, raw_position):
    """
    Apply joint calibration coefficients
    
    Args:
        joint_name: joint name
        raw_position: raw position value
    
    Returns:
        calibrated_position: calibrated position value
    """
    for joint_cal in JOINT_CALIBRATION:
        if joint_cal[0] == joint_name:
            offset = joint_cal[1]  # zero position offset
            scale = joint_cal[2]   # scale factor
            calibrated_position = (raw_position - offset) * scale
            return calibrated_position
    return raw_position  # if no calibration coefficient found, return original value

def inverse_kinematics_3d(x, y, z, l1=0.1159, l2=0.1350, shoulder_offset_z=0.0):
    """
    Calculate 3D inverse kinematics for the robotic arm
    ENU coordinate system: X=forward, Y=left, Z=up
    
    Parameters:
        x: End effector X coordinate (forward, meters)
        y: End effector Y coordinate (left, meters)
        z: End effector Z coordinate (up, meters)
        l1: Upper arm length (default 0.1159 m)
        l2: Lower arm length (default 0.1350 m)
        shoulder_offset_z: Height offset of shoulder joint from base (meters)
        
    Returns:
        joint1, joint2, joint3: Joint angles in degrees
    """
    # Joint angle offsets
    theta1_offset = math.atan2(0.028, 0.11257)  # theta1 offset when joint2=0
    theta2_offset = math.atan2(0.0052, 0.1349) + theta1_offset  # theta2 offset when joint3=0
    
    # Calculate shoulder_pan angle (joint1) from Y and X
    # Negative Y (right) should give positive shoulder_pan for correct motion
    joint1_rad = math.atan2(-y, x)  # FIXED: Inverted Y axis
    
    # Calculate horizontal distance in XY plane from shoulder pan axis
    r_xy = math.sqrt(x**2 + y**2)
    
    # Adjust Z coordinate for shoulder height offset
    z_adjusted = z - shoulder_offset_z
    
    # Now solve 2D IK in the vertical plane using r_xy and z_adjusted
    r = math.sqrt(r_xy**2 + z_adjusted**2)
    r_max = l1 + l2
    
    # If target point is beyond maximum workspace, scale it to the boundary
    if r > r_max:
        scale_factor = r_max / r
        r_xy *= scale_factor
        z_adjusted *= scale_factor
        r = r_max
    
    # If target point is less than minimum workspace, scale it
    r_min = abs(l1 - l2)
    if r < r_min and r > 0:
        scale_factor = r_min / r
        r_xy *= scale_factor
        z_adjusted *= scale_factor
        r = r_min
    
    # Use law of cosines to calculate theta2 (elbow angle)
    cos_theta2 = -(r**2 - l1**2 - l2**2) / (2 * l1 * l2)
    # Clamp to valid range to avoid numerical errors
    cos_theta2 = max(-1.0, min(1.0, cos_theta2))
    theta2 = math.pi - math.acos(cos_theta2)
    
    # Calculate theta1 (shoulder lift angle)
    beta = math.atan2(z_adjusted, r_xy)
    gamma = math.atan2(l2 * math.sin(theta2), l1 + l2 * math.cos(theta2))
    theta1 = beta + gamma
    
    # Convert theta1 and theta2 to joint2 and joint3 angles
    joint2_rad = theta1 + theta1_offset
    joint3_rad = theta2 + theta2_offset
    
    # Ensure angles are within URDF limits
    joint2_rad = max(-0.1, min(3.45, joint2_rad))
    joint3_rad = max(-0.2, min(math.pi, joint3_rad))
    
    # Convert from radians to degrees
    joint1_deg = math.degrees(joint1_rad)
    joint2_deg = math.degrees(joint2_rad)
    joint3_deg = math.degrees(joint3_rad)
    
    # Apply transformations to match robot convention
    joint2_deg = 90 - joint2_deg
    joint3_deg = joint3_deg - 90
    
    return joint1_deg, joint2_deg, joint3_deg

def move_to_zero_position(robot, duration=3.0, kp=0.5):
    """
    Use P control to slowly move robot to zero position
    
    Args:
        robot: robot instance
        duration: time to move to zero position (seconds)
        kp: proportional gain
    """
    print("Using P control to slowly move robot to zero position...")
    
    # Get current robot state
    current_obs = robot.get_observation()
    
    # Extract current joint positions
    current_positions = {}
    for key, value in current_obs.items():
        if key.endswith('.pos'):
            motor_name = key.removesuffix('.pos')
            current_positions[motor_name] = value
    
    # Zero position targets
    zero_positions = {
        'shoulder_pan': 0.0,
        'shoulder_lift': 0.0,
        'elbow_flex': 0.0,
        'wrist_flex': 0.0,
        'wrist_roll': 0.0,
        'gripper': 0.0
    }
    
    # Calculate control steps
    control_freq = 50  # 50Hz control frequency
    total_steps = int(duration * control_freq)
    step_time = 1.0 / control_freq
    
    print(f"Will use P control to move to zero position in {duration} seconds, control frequency: {control_freq}Hz, proportional gain: {kp}")
    
    for step in range(total_steps):
        # Get current robot state
        current_obs = robot.get_observation()
        current_positions = {}
        for key, value in current_obs.items():
            if key.endswith('.pos'):
                motor_name = key.removesuffix('.pos')
                # Apply calibration coefficients
                calibrated_value = apply_joint_calibration(motor_name, value)
                current_positions[motor_name] = calibrated_value
        
        # P control calculation
        robot_action = {}
        for joint_name, target_pos in zero_positions.items():
            if joint_name in current_positions:
                current_pos = current_positions[joint_name]
                error = target_pos - current_pos
                
                # P control: output = Kp * error
                control_output = kp * error
                
                # Convert control output to position command
                new_position = current_pos + control_output
                robot_action[f"{joint_name}.pos"] = new_position
        
        # Send action to robot
        if robot_action:
            robot.send_action(robot_action)
        
        # Show progress
        if step % (control_freq // 2) == 0:  # Show progress every 0.5 seconds
            progress = (step / total_steps) * 100
            #print(f"Moving to zero position progress: {progress:.1f}%")
        
        time.sleep(step_time)
    
    print("Robot has moved to zero position")

def return_to_start_position(robot, start_positions, kp=0.5, control_freq=50):
    """
    Use P control to return to start position
    
    Args:
        robot: robot instance
        start_positions: start joint position dictionary
        kp: proportional gain
        control_freq: control frequency (Hz)
    """
    print("Returning to start position...")
    
    control_period = 1.0 / control_freq
    max_steps = int(5.0 * control_freq)  # Maximum 5 seconds
    
    for step in range(max_steps):
        # Get current robot state
        current_obs = robot.get_observation()
        current_positions = {}
        for key, value in current_obs.items():
            if key.endswith('.pos'):
                motor_name = key.removesuffix('.pos')
                current_positions[motor_name] = value  # Don't apply calibration coefficients
        
        # P control calculation
        robot_action = {}
        total_error = 0
        for joint_name, target_pos in start_positions.items():
            if joint_name in current_positions:
                current_pos = current_positions[joint_name]
                error = target_pos - current_pos
                total_error += abs(error)
                
                # P control: output = Kp * error
                control_output = kp * error
                
                # Convert control output to position command
                new_position = current_pos + control_output
                robot_action[f"{joint_name}.pos"] = new_position
        
        # Send action to robot
        if robot_action:
            robot.send_action(robot_action)
        
        # Check if reached start position
        if total_error < 2.0:  # If total error is less than 2 degrees, consider reached
            print("Returned to start position")
            break
        
        time.sleep(control_period)
    
    print("Return to start position completed")

def p_control_loop(robot, keyboard, target_positions, start_positions, current_x, current_y, current_z, kp=0.5, control_freq=50):
    """
    P control loop with 3D end effector control
    
    Args:
        robot: robot instance
        keyboard: keyboard instance
        target_positions: target joint position dictionary
        start_positions: start joint position dictionary
        current_x: current X coordinate (forward)
        current_y: current Y coordinate (left)
        current_z: current Z coordinate (up)
        kp: proportional gain
        control_freq: control frequency (Hz)
    """
    control_period = 1.0 / control_freq
    
    # Initialize pitch control variables
    pitch = 0.0  # Initial pitch adjustment
    pitch_step = 1  # Pitch adjustment step size
    
    # Initialize wrist_roll offset to maintain gripper orientation
    wrist_roll_offset = 0.0
    
    print(f"Starting P control loop, control frequency: {control_freq}Hz, proportional gain: {kp}")
    print(f"ENU Coordinate System: X=forward, Y=left, Z=up")
    
    while True:
        try:
            # Get keyboard input
            keyboard_action = keyboard.get_action()
            
            if keyboard_action:
                # Process keyboard input, update target positions
                for key, value in keyboard_action.items():
                    if key == 'x':
                        # Exit program, first return to start position
                        print("Exit command detected, returning to start position...")
                        return_to_start_position(robot, start_positions, 0.2, control_freq)
                        return
                    
                    # Direct joint control mapping
                    joint_controls = {
                        '1': ('wrist_roll_offset', -1),   # Manual wrist roll adjustment
                        '3': ('wrist_roll_offset', 1),    # Manual wrist roll adjustment
                        'c': ('gripper', -1),              # Joint 6 decrease
                        'v': ('gripper', 1),               # Joint 6 increase
                    }
                    
                    # 3D coordinate control (ENU system)
                    xyz_controls = {
                        'w': ('x', 0.004),   # X increase (forward)
                        's': ('x', -0.004),  # X decrease (backward)
                        'd': ('y', -0.004),  # Y decrease (right) - FIXED
                        'a': ('y', 0.004),   # Y increase (left) - FIXED
                        'r': ('z', 0.004),   # Z increase (up)
                        'f': ('z', -0.004),  # Z decrease (down)
                    }
                    
                    # Pitch control
                    if key == 't':
                        pitch += pitch_step
                        #print(f"Increase pitch adjustment: {pitch:.3f}")
                    elif key == 'g':
                        pitch -= pitch_step
                        #print(f"Decrease pitch adjustment: {pitch:.3f}")
                    
                    if key in joint_controls:
                        joint_name, delta = joint_controls[key]
                        if joint_name == 'wrist_roll_offset':
                            wrist_roll_offset += delta
                            #print(f"Manual wrist roll offset: {wrist_roll_offset:.3f}")
                        elif joint_name in target_positions:
                            current_target = target_positions[joint_name]
                            new_target = int(current_target + delta)
                            target_positions[joint_name] = new_target
                            #print(f"Update target position {joint_name}: {current_target} -> {new_target}")
                    
                    elif key in xyz_controls:
                        coord, delta = xyz_controls[key]
                        if coord == 'x':
                            current_x += delta
                        elif coord == 'y':
                            current_y += delta
                        elif coord == 'z':
                            current_z += delta
                        
                        # Calculate target angles for all three joints using 3D IK
                        joint1_target, joint2_target, joint3_target = inverse_kinematics_3d(
                            current_x, current_y, current_z
                        )
                        target_positions['shoulder_pan'] = joint1_target
                        target_positions['shoulder_lift'] = joint2_target
                        target_positions['elbow_flex'] = joint3_target
                        #print(f"Update position: X={current_x:.4f}, Y={current_y:.4f}, Z={current_z:.4f}")
                        #print(f"  joint1={joint1_target:.3f}, joint2={joint2_target:.3f}, joint3={joint3_target:.3f}")
            
            # Apply pitch adjustment to wrist_flex
            # Calculate wrist_flex target position based on shoulder_lift and elbow_flex
            if 'shoulder_lift' in target_positions and 'elbow_flex' in target_positions:
                target_positions['wrist_flex'] = - target_positions['shoulder_lift'] - target_positions['elbow_flex'] + pitch
            
            # Keep gripper yaw constant by counter-rotating wrist_roll relative to shoulder_pan
            # When shoulder_pan rotates, wrist_roll should rotate in opposite direction
            if 'shoulder_pan' in target_positions:
                target_positions['wrist_roll'] = target_positions['shoulder_pan'] + wrist_roll_offset
            
            # Show current state periodically
            if hasattr(p_control_loop, 'step_counter'):
                p_control_loop.step_counter += 1
            else:
                p_control_loop.step_counter = 0
            
            if p_control_loop.step_counter % 100 == 0:
                print(f"Position: X={current_x:.4f}, Y={current_y:.4f}, Z={current_z:.4f} | Pitch: {pitch:.3f} | Wrist offset: {wrist_roll_offset:.1f}")
            
            # Get current robot state
            current_obs = robot.get_observation()
            
            # Extract current joint positions
            current_positions = {}
            for key, value in current_obs.items():
                if key.endswith('.pos'):
                    motor_name = key.removesuffix('.pos')
                    # Apply calibration coefficients
                    calibrated_value = apply_joint_calibration(motor_name, value)
                    current_positions[motor_name] = calibrated_value
            
            # P control calculation
            robot_action = {}
            for joint_name, target_pos in target_positions.items():
                if joint_name in current_positions:
                    current_pos = current_positions[joint_name]
                    error = target_pos - current_pos
                    
                    # P control: output = Kp * error
                    control_output = kp * error
                    
                    # Convert control output to position command
                    new_position = current_pos + control_output
                    robot_action[f"{joint_name}.pos"] = new_position
            
            # Send action to robot
            if robot_action:
                robot.send_action(robot_action)
            
            time.sleep(control_period)
            
        except KeyboardInterrupt:
            print("User interrupted program")
            break
        except Exception as e:
            print(f"P control loop error: {e}")
            traceback.print_exc()
            break

def main():
    """Main function"""
    print("LeRobot 3D Keyboard Control with ENU Coordinates (P Control)")
    print("="*50)
    
    try:
        # Import necessary modules
        from lerobot.robots.so100_follower import SO100Follower, SO100FollowerConfig
        from lerobot.teleoperators.keyboard import KeyboardTeleop, KeyboardTeleopConfig
        
        # Get port
        port = input("Please enter the USB port for SO100 robot (e.g., /dev/ttyACM0): ").strip()
        
        # If directly press Enter, use default port
        if not port:
            port = "/dev/ttyACM0"
            print(f"Using default port: {port}")
        else:
            print(f"Connecting to port: {port}")
        
        # Configure robot
        robot_config = SO100FollowerConfig(port=port)
        robot = SO100Follower(robot_config)
        
        # Configure keyboard
        keyboard_config = KeyboardTeleopConfig()
        keyboard = KeyboardTeleop(keyboard_config)
        
        # Connect devices
        robot.connect()
        keyboard.connect()
        
        print("Device connection successful!")
        
        # Ask whether to recalibrate
        while True:
            calibrate_choice = input("Do you want to recalibrate the robot? (y/n): ").strip().lower()
            if calibrate_choice in ['y', 'yes']:
                print("Starting recalibration...")
                robot.calibrate()
                print("Calibration completed!")
                break
            elif calibrate_choice in ['n', 'no']:
                print("Using previous calibration file")
                break
            else:
                print("Please enter y or n")
        
        # Read initial joint angles
        print("Reading initial joint angles...")
        start_obs = robot.get_observation()
        start_positions = {}
        for key, value in start_obs.items():
            if key.endswith('.pos'):
                motor_name = key.removesuffix('.pos')
                start_positions[motor_name] = int(value)  # Don't apply calibration coefficients
        
        print("Initial joint angles:")
        for joint_name, position in start_positions.items():
            print(f"  {joint_name}: {position}°")
        
        # Move to zero position
        move_to_zero_position(robot, duration=3.0)
        
        # Initialize target positions as current positions (integers)
        target_positions = {
            'shoulder_pan': 0.0,
            'shoulder_lift': 0.0,
            'elbow_flex': 0.0,
            'wrist_flex': 0.0,
            'wrist_roll': 0.0,
            'gripper': 0.0
        }
        
        # Initialize x,y,z coordinate control (ENU system)
        # Starting position: forward, center, up
        x0, y0, z0 = 0.1629, 0.0, 0.1131
        current_x, current_y, current_z = x0, y0, z0
        print(f"Initialize end effector position (ENU): X={current_x:.4f} (forward), Y={current_y:.4f} (left), Z={current_z:.4f} (up)")
        
        print("\nKeyboard control instructions (ENU Coordinate System):")
        print("="*50)
        print("3D Position Control:")
        print("- W/S: Move forward/backward (X axis)")
        print("- A/D: Move left/right (Y axis)")
        print("- R/F: Move up/down (Z axis)")
        print("\nOrientation Control:")
        print("- T/G: Pitch adjustment increase/decrease (affects wrist_flex)")
        print("- 1/3: Manual wrist roll adjustment (fine-tune yaw)")
        print("\nGripper Control:")
        print("- C/V: Gripper close/open")
        print("\nProgram Control:")
        print("- X: Exit program (return to start position first)")
        print("- ESC: Exit program")
        print("="*50)
        print("Note: Gripper yaw orientation maintained automatically during pan motion")
        
        # Start P control loop
        p_control_loop(robot, keyboard, target_positions, start_positions, 
                      current_x, current_y, current_z, kp=0.5, control_freq=50)
        
        # Disconnect
        robot.disconnect()
        keyboard.disconnect()
        print("Program ended")
        
    except Exception as e:
        print(f"Program execution failed: {e}")
        traceback.print_exc()
        print("Please check:")
        print("1. Whether the robot is properly connected")
        print("2. Whether the USB port is correct")
        print("3. Whether you have sufficient permissions to access USB devices")
        print("4. Whether the robot is properly configured")

if __name__ == "__main__":
    main()