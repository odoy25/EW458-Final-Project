import time
import threading
import math
import roslibpy
import pygame

# ROS Connection Details
ip = '192.168.8.104'
port = 9012
robot_name = 'echo'

# Establish connection to ROS
ros = roslibpy.Ros(host=ip, port=port)
ros.run()
print(f"Connected to ROS: {ros.is_connected}")


class RobotController:
    """Handles joystick inputs, movement, and LED status."""

    def __init__(self, ros, robot_name):

        # Robot Variables
        self.ros = ros
        self.robot_name = robot_name

        # Topics
        self.imu_topic = roslibpy.Topic(self.ros, f'/{self.robot_name}/imu', 'sensor_msgs/Imu')
        self.odom_topic = roslibpy.Topic(self.ros, f'/{self.robot_name}/odom', 'nav_msgs/Odometry')

        # Subscriptions
        self.imu_sub = self.imu_topic.subscribe(self.callback_imu)
        self.odom_sub = self.odom_topic.subscribe(self.callback_odom)
        
        # Publishers
        self.led_pub = roslibpy.Topic(self.ros, f'/{self.robot_name}/cmd_lightring', 'irobot_create_msgs/msg/LightringLeds') # LED Pub
        self.vel_pub = roslibpy.Topic(self.ros, f'/{self.robot_name}/cmd_vel', 'geometry_msgs/msg/Twist')                    # Velocity/Rotation Cmd Pub
        self.audio_pub = roslibpy.Topic(self.ros, f'/{self.robot_name}/cmd_audio', 'irobot_create_msgs/AudioNoteVector')     # Audio Playback Pub

        #self.safety_pub = roslibpy.Topic(ros, f'/{robot_name}/safety_override', 'irobot_create_msgs/msg/SafetyOverride')
        

        # Desired Waypoint Coordniates (ODOM Frame)
        self.des_x = [4.4, 4.4, 0.44, 0.44, 3.60, 3.60, 0.46, 0.46, 3.60]
        self.des_y = [-0.08,  0.40, 0.44, 0.90, 0.90, 1.30, 1.30, 1.70, 1.65]

        # Controller Gains
        self.Ks = 0.12  # Speed gain (m/s) acutal speed
        self.Kr = 0.6   # Rotation gain
        
        self.yaw_threshold = 0.02  # Precision threshold for rotation control
        self.initial_yaw = None # set inital yaw to 0
        self.x, self.y, self.yaw = 0, 0, 0  # Position variables set to 0

        # Check to see if I am able to subscribe to topics
        try:
            self.imu_topic.subscribe(self.callback_imu)
            self.odom_topic.subscribe(self.callback_odom)
        except Exception as e:
            print(f"Failed to subscribe to ROS topics: {e}")

        # Threading Set-Up
        self.stop_event = threading.Event() # Create a threading event to stop/start threads
        self.lock = threading.Lock()        # Prevent Threads from overwriting the same variables

        # Create threads for joystick controller and other tasks
        self.led_thread = threading.Thread(target=self.led_blinker, daemon=True)
        self.controller_thread = threading.Thread(target=self.controller, daemon=True)
        self.nav_thread = threading.Thread(target=self.navigate_waypoints,daemon=True)
        
        # Thread List
        self.threads = [            
            self.controller_thread,
            self.led_thread,
            self.nav_thread 
        ] 

        # Inital Conditions and Modes
        self.arm = False  
        self.mode = "manual"  
        self.exit_pressed_time = None  
        self.reset_odom_pressed_time = None
        self.controller_exit = False


    def wrap_to_pi(self, angle):
        """Wraps an angle to the range [-π, π] radians."""
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def callback_imu(self, message):
        """IMU callback to update yaw orientation."""
        o = message.get('orientation')
        qx, qy, qz, qw = o.get('x'), o.get('y'), o.get('z'), o.get('w')
        raw_yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        if self.initial_yaw is None:
            self.initial_yaw = raw_yaw
        self.yaw = self.wrap_to_pi(raw_yaw - self.initial_yaw)

    def callback_odom(self, message):
        """Odometry callback to update position."""
        pos = message.get('pose').get('pose').get('position')
        self.x, self.y = pos.get('x'), pos.get('y')


    def reset_odometry(self):
        """Reset odometry to the origin and set Inital Yaw to 0 rads."""
        try:
            reset_odom_service = roslibpy.Service(self.ros, f'/{self.robot_name}/reset_pose', 'irobot_create_msgs/ResetPose')
            result = reset_odom_service.call(roslibpy.ServiceRequest())
            print("ODOM Reset:", result)

            self.initial_yaw = None
            self.yaw = 0

            print(f"Inital Yaw angle set to {self.yaw}")
        except Exception as e:
            print(f"Failed to reset odometry: {e}") 

    def update_led(self, red, green, blue):
        """Sends an LED update."""
        led_colors = [{"red": red, "green": green, "blue": blue}] * 6
        message_light = {'leds': led_colors, 'override_system': True}
        with self.lock:
            self.led_pub.publish(roslibpy.Message(message_light))

    def led_blinker(self):
        """Handles LED updates based on mode."""
        while not self.stop_event.is_set():
            if self.arm:
                if self.mode == "auto":
                    while self.arm and self.mode == "auto":
                        self.update_led(255, 255, 0)  # Yellow (Blink in auto)
                        time.sleep(0.2)
                        self.update_led(0, 0, 0)
                        time.sleep(0.2)
                        
                elif self.mode == "manual":
                    self.update_led(0, 255, 0)  # Green (Solid in manual)
                    time.sleep(1)
            else:
                self.update_led(0, 0, 255)  # Blue (Disarmed)
                time.sleep(1)
    
    def play_single_note(self, frequency):
        """Plays a single note with the specified frequency."""
        # Define the note duration (in seconds and nanoseconds)
        note_duration_seconds = 0# Duration in seconds
        note_duration_nanoseconds = 500000000  # 0.5 seconds in nanoseconds (1 second = 1e9 ns)

        # Create the message payload including the frequency and max_runtime
        audio_command = {
            'frequency': frequency,  # Frequency for the note
            'max_runtime': {
                'sec': note_duration_seconds,
                'nanosec': note_duration_nanoseconds
            }  # Set max_runtime with seconds and nanoseconds
        }
        audio_dict = {'notes': [audio_command],'append': False}
        # Create the ROS message and publish it
        audio_msg = roslibpy.Message(audio_dict)
        self.audio_pub.publish(audio_msg)

        print(f"Playing note at {frequency} Hz for {note_duration_nanoseconds/1000000000} seconds.")

    def play_audio(self, sequence_type="power_on"):
        """Plays the power-on or power-off sequence."""
        # Define the note vector (frequencies for notes)
        note_vector = {
            'C4': 261, 'D4': 293, 'E4': 329, 'F4': 349,
            'G4': 392, 'A4': 440, 'B4': 493, 'C5': 523,
            'D5': 587, 'E5': 659
        }

        # Define the sequence of notes for power-on and power-off
        if sequence_type == "power_on":
            sequence = ['D4', 'D4', 'D4', 'E4', 'C5']  # Rising notes for power-on

        # Play each note in the sequence
        for note in sequence:
            if note in note_vector:
                freq = note_vector[note]
                self.play_single_note(freq)
                time.sleep(0.2)  # Delay between notes (500ms)

    def arm_audio(self, sequence_type="arm_on"):
        """Plays the power-on or power-off sequence."""
        # Define the note vector (frequencies for notes)
        note_vector = {
            'C4': 261, 'D4': 293, 'E4': 329, 'F4': 349,
            'G4': 392, 'A4': 440, 'B4': 493, 'C5': 523,
            'D5': 587, 'E5': 659
        }

        # Define the sequence of notes for power-on and power-off
        if sequence_type == "arm_on":
            sequence = ['C4', 'E4', 'D4', 'F4', 'E5']  # Rising notes for power-on

        # Play each note in the sequence
        for note in sequence:
            if note in note_vector:
                freq = note_vector[note]
                self.play_single_note(freq)
                time.sleep(0.2)  # Delay between notes (500ms)


    def disarm_audio(self, sequence_type="disarm_off"):
        """Plays the disarm sequence."""
        # Define the note vector (frequencies for notes)
        note_vector = {
            'C4': 261, 'D4': 293, 'E4': 329, 'F4': 349,
            'G4': 392, 'A4': 440, 'B4': 493, 'C5': 523,
            'D5': 587, 'E5': 659
        }

        # Define the sequence of notes for disarm
        if sequence_type == "disarm_off":
            sequence = ['E5', 'C5', 'D5', 'B4', 'C4']  # Falling notes for disarm

        # Create the audio command
        for note in sequence:
            if note in note_vector:
                freq = note_vector[note]
                self.play_single_note(freq)
                time.sleep(0.2)

        print(f"Disarm audio sequence: {sequence}")

    def stop_audio(self, sequence_type="power_off"):
        """Stops the audio and plays the stop sound."""
        # Define the note vector (frequencies for notes)
        note_vector = {
            'C4': 261, 'D4': 293, 'E4': 329, 'F4': 349,
            'G4': 392, 'A4': 440, 'B4': 493, 'C5': 523,
            'D5': 587, 'E5': 659
        }
        if sequence_type == "power_off":
            sequence = ['C5', 'C5', 'C5', 'E4', 'D4']  # Falling notes for power-off
        # Define the sequence of notes for power-off

        # Play each note in the sequence
        for note in sequence:
            if note in note_vector:
                freq = note_vector[note]
                self.play_single_note(freq)
                time.sleep(0.2)


    # Manual Controller
    def vel_cmd(self, rotation, velocity):
        """Publishes velocity commands for the Manual Controller"""
        geometry_msg = roslibpy.Message({
            'linear': {'x': velocity, 'y': 0, 'z': 0},
            'angular': {'x': 0, 'y': 0, 'z': -rotation} # Rotation is set to negative IOT turn robot in the correct direction according to the controller inputs
        })
        with self.lock:
            self.vel_pub.publish(geometry_msg)
            #self.safety_pub.publish({'data': 'full'})
        #print(f"Velocity: {velocity:.2f}, Rotation: {rotation:.2f}")

    # Waypoint Controller
    def move_robot(self, speed, rotation):
        """Publishes velocity commands for the Waypoint Controller"""
        twist = roslibpy.Message({'linear': {'x': speed, 'y': 0, 'z': 0}, 
                                  'angular': {'x': 0, 'y': 0, 'z': rotation}}) # Rotation set to positive
        with self.lock:
            self.vel_pub.publish(twist)

    
    def rotate_to_next_waypoint(self, next_x, next_y):
        """Rotates the robot precisely to face the next waypoint."""
        target_yaw = math.atan2(next_y - self.y, next_x - self.x) # desired yaw angle based on the difference from your position to the next waypoint 
        yaw_err = self.wrap_to_pi(target_yaw - self.yaw) # yaw error
        while abs(yaw_err) > self.yaw_threshold: # if the yaw error is greater than 0.02 conduct the turn, else stop rotation
            self.move_robot(0, self.Kr * yaw_err)
            time.sleep(0.1)
            yaw_err = self.wrap_to_pi(target_yaw - self.yaw) # update yaw error within the check loop
        self.move_robot(0, 0)
        time.sleep(0.2)


    def navigate_waypoints(self):
        """Executes waypoint navigation logic only when it is called or activated"""
        try:
            while not self.stop_event.is_set():
                if self.arm and self.mode == "auto":            # First Layer Logic statement to start/end waypoint nav
                    for i in range(len(self.des_x)):
                        while self.arm and self.mode == "auto": # Second Layer Logic built into the while loop IOT stop waypoint nav for mode changes and arm/disarm
                            x_err = self.des_x[i] - self.x      
                            y_err = self.des_y[i] - self.y
                            des_dist = math.sqrt(x_err**2 + y_err**2) 

                            des_yaw = math.atan2(y_err, x_err)
                            yaw_err = self.wrap_to_pi(des_yaw - self.yaw)

                            print(f"Waypoint {i+1}: X={self.x:.2f}, Y={self.y:.2f}, Target X={self.des_x[i]}, Target Y={self.des_y[i]}, Yaw={self.yaw:.2f}, Target Yaw={des_yaw:.2f}, Yaw Err={yaw_err:.2f}")

                            # Move with proportional yaw correction
                            self.move_robot(self.Ks, self.Kr * yaw_err)

                            # Stop when close to the waypoint
                            if des_dist <= 0.1:
                                self.move_robot(0, 0)
                                print(f"Reached Waypoint {i+1}")

                                 # If it's the last waypoint, stop the robot completely
                                if i == len(self.des_x) - 1:
                                    print("Reached final waypoint. Stopping robot.")
                                    self.move_robot(0, 0)  # Ensure it stops
                                    return  # Exit the function after the last waypoint is reached

                                if i < len(self.des_x) - 1:
                                    print("Rotating towards next waypoint...")
                                    self.rotate_to_next_waypoint(self.des_x[i + 1], self.des_y[i + 1])

                                break

                            time.sleep(0.1)

                    else:
                        continue

            time.sleep(1) # Check for mode change every second (Don't know if this is needed)

        except KeyboardInterrupt:
            self.move_robot(0, 0)



    def controller(self):
        """Handles joystick inputs."""
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            print("No joystick found. Exiting controller thread.")
            pygame.quit()
            return

        joy = pygame.joystick.Joystick(0)
        joy.init()
        print("Joystick connected.")


        try:
            while not self.stop_event.is_set():
                for event in pygame.event.get():  # Process all events
                    if event.type == pygame.QUIT: # Condition to quit pygame
                        pygame.quit()
                        return

                # Arm condition (button or axis)
                if joy.get_numaxes() > 4 and joy.get_axis(4) >= 0.7:  # Check for arm condition
                        self.arm = True
                        self.arm_audio("arm_on")
                        print("Vehicle armed")

                # Disarm condition (button 4)
                if joy.get_button(4):  # Button for disarming
                        self.arm = False
                        self.disarm_audio("disarm_off")

                    # Reset arm flag when disarmed (so arm audio can play again)

                # Auto mode switching (hat 0 up)
                if joy.get_hat(0) == (0, 1):  # Auto mode
                        self.mode = "auto"
                        self.stop_audio("power_off")
                        print("Mode: Auto")

                # Manual mode switching (hat 0 left)
                if joy.get_hat(0) == (-1, 0):  # Manual mode
                        self.mode = "manual"
                        self.play_audio("power_on")
                        print("Mode: Manual")

                
                # Hold R1 (button 7) for 3 seconds to exit
                if joy.get_button(7):  
                    if self.exit_pressed_time is None:
                        self.exit_pressed_time = time.time()
                    elif time.time() - self.exit_pressed_time >= 3:
                        print("Exit button held for 3 seconds. Stopping threads.")
                        self.controller_exit = True

                else:
                    self.exit_pressed_time = None

                # Hold Center (button 10) for 3 seconds to reset ODOM
                if joy.get_button(10):  
                    if self.reset_odom_pressed_time is None:
                        self.reset_odom_pressed_time = time.time()
                    elif time.time() - self.reset_odom_pressed_time >= 3:
                        self.reset_odometry()
                else:
                    self.reset_odom_pressed_time = None

                # Manual Control
                if self.arm and self.mode == "manual":
                    fwd_speed = max(min(joy.get_axis(5) * 0.5, 0.3), 0)  
                    #print(fwd_speed)
                    rot_speed = max(min(joy.get_axis(0) * 1.5, 1), -1)  

                    if abs(joy.get_axis(5) + 1) > 0.1 or abs(joy.get_axis(0)) > 0.1:
                        self.vel_cmd(rot_speed, fwd_speed)
                    else:
                        self.vel_cmd(0, 0)

                time.sleep(0.1)  # Control loop frequency

        except KeyboardInterrupt:
            print("Joystick control interrupted.")
        finally:
            pygame.quit()
            print("Joystick control thread exiting.")

    def start_threads(self):
        """Starts control and LED threads."""
        for t in self.threads:
            t.start()

    def end_threads(self):
        """Stops all threads gracefully."""
        self.stop_event.set()
        for t in self.threads:
            t.join()
        print("All threads stopped.")


if __name__ == "__main__":
    robot_interface = RobotController(ros, robot_name)  # Pass sensor instance to controller
    robot_interface.start_threads()
    try:
        while True:  
            time.sleep(1)  # Keep running indefinitely
    except KeyboardInterrupt:
        print("\nManual interrupt received. Shutting down...")
        robot_interface.end_threads()
        ros.terminate()
        print("Shutdown complete.")