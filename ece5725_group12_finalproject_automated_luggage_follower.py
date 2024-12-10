import RPi.GPIO as GPIO
import cv2
import numpy as np
from picamera2 import Picamera2
from time import sleep
import os
import pygame
from pygame.locals import *

# Setup environment variables for PiTFT
os.environ['SDL_FBDEV'] = '/dev/fb0'
os.environ['SDL_VIDEODRIVER'] = 'fbcon'

# GPIO Pin Settings
EXIT_BUTTON_PIN = 17  # GPIO pin number (can be changed as needed)
AI1_PIN = 5  # GPIO pin for Motor A direction 1
AI2_PIN = 6  # GPIO pin for Motor A direction 2
BI1_PIN = 16  # GPIO pin for Motor B direction 1
BI2_PIN = 20  # GPIO pin for Motor B direction 2
PWMA_PIN = 13  # GPIO pin for Motor A PWM
PWMB_PIN = 21  # GPIO pin for Motor B PWM

# Constants
pwm_frequency = 100  # Set PWM frequency
AREA_MIN_THRESHOLD = 1000    
MOVEMENT_THRESHOLD = 1000    
DISTANCE_THRESHOLD = 10000  
DISTANCE_FAR = 500         # Distance bar minimum value
DISTANCE_NEAR = 20000      # Distance bar maximum value
PADDING = 15               # Target box padding
lost_target_count = 0
MAX_LOST_COUNT = 10  
search_direction = 1  
SEARCH_SPEED = 60    
last_valid_position = None  # Record the last position where target was seen

def setup_gpio():
    """
    Setup GPIO pins
    """
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(EXIT_BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Set as pull-up input
    GPIO.setup(AI1_PIN, GPIO.OUT)
    GPIO.setup(AI2_PIN, GPIO.OUT)
    GPIO.setup(BI1_PIN, GPIO.OUT)
    GPIO.setup(BI2_PIN, GPIO.OUT)
    GPIO.setup(PWMA_PIN, GPIO.OUT)
    GPIO.setup(PWMB_PIN, GPIO.OUT)
    print("GPIO Setup Complete")

# Call setup_gpio() to initialize GPIO pins
setup_gpio()

# PWM setup
motorA_pwm = GPIO.PWM(PWMA_PIN, pwm_frequency)
motorB_pwm = GPIO.PWM(PWMB_PIN, pwm_frequency)

# Start both PWMs at 0 duty cycle (stopped)
motorA_pwm.start(0)
motorB_pwm.start(0)

# Movement control functions
def move_forward():
    GPIO.output(AI1_PIN, GPIO.HIGH)
    GPIO.output(AI2_PIN, GPIO.LOW)
    GPIO.output(BI1_PIN, GPIO.HIGH)
    GPIO.output(BI2_PIN, GPIO.LOW)
    motorA_pwm.ChangeDutyCycle(60)  # Increase forward speed (original value 40)
    motorB_pwm.ChangeDutyCycle(60)  # Increase forward speed (original value 40)
    print("Moving Forward")

def move_backward():
    GPIO.output(AI1_PIN, GPIO.LOW)
    GPIO.output(AI2_PIN, GPIO.HIGH)
    GPIO.output(BI1_PIN, GPIO.LOW)
    GPIO.output(BI2_PIN, GPIO.HIGH)
    motorA_pwm.ChangeDutyCycle(60)  # Increase backward speed (original value 40)
    motorB_pwm.ChangeDutyCycle(60)  # Increase backward speed (original value 40)
    print("Moving Backward")

def move_right():
    GPIO.output(AI1_PIN, GPIO.LOW)
    GPIO.output(AI2_PIN, GPIO.HIGH)
    GPIO.output(BI1_PIN, GPIO.HIGH)
    GPIO.output(BI2_PIN, GPIO.LOW)
    motorA_pwm.ChangeDutyCycle(80)  # Increase turning speed (original value 50)
    motorB_pwm.ChangeDutyCycle(80)  # Increase turning speed (original value 50)
    print("Turning Left")

def move_left():
    GPIO.output(AI1_PIN, GPIO.HIGH)
    GPIO.output(AI2_PIN, GPIO.LOW)
    GPIO.output(BI1_PIN, GPIO.LOW)
    GPIO.output(BI2_PIN, GPIO.HIGH)
    motorA_pwm.ChangeDutyCycle(80)  # Increase turning speed (original value 50)
    motorB_pwm.ChangeDutyCycle(80)  # Increase turning speed (original value 50)
    print("Turning Right")

def stop():
    motorA_pwm.ChangeDutyCycle(0)
    motorB_pwm.ChangeDutyCycle(0)
    print("Stopped")

def gpio_exit_check():
    """
    Check if GPIO exit button is pressed
    """
    return GPIO.input(EXIT_BUTTON_PIN) == GPIO.LOW  # Low level when button is pressed

def search_target(frame_width):
    """
    Execute search when target is lost
    """
    global search_direction, last_valid_position
    
    # If last position is recorded, determine initial search direction based on position
    if last_valid_position is not None:
        if last_valid_position < frame_width // 2:
            search_direction = -1  # Search left
        else:
            search_direction = 1   # Search right
    
    # Use lower speed for search
    if search_direction == 1:
        GPIO.output(AI1_PIN, GPIO.LOW)
        GPIO.output(AI2_PIN, GPIO.HIGH)
        GPIO.output(BI1_PIN, GPIO.HIGH)
        GPIO.output(BI2_PIN, GPIO.LOW)
        motorA_pwm.ChangeDutyCycle(SEARCH_SPEED)
        motorB_pwm.ChangeDutyCycle(SEARCH_SPEED)
        return "Searching Right..."
    else:
        GPIO.output(AI1_PIN, GPIO.HIGH)
        GPIO.output(AI2_PIN, GPIO.LOW)
        GPIO.output(BI1_PIN, GPIO.LOW)
        GPIO.output(BI2_PIN, GPIO.HIGH)
        motorA_pwm.ChangeDutyCycle(SEARCH_SPEED)
        motorB_pwm.ChangeDutyCycle(SEARCH_SPEED)
        return "Searching Left..."

def detect_and_track_target(frame, hsv_min, hsv_max):
    """
    Improved target detection and tracking function
    """
    global lost_target_count, last_valid_position, search_direction
    
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    if len(hsv_min) == 2:
        mask1 = cv2.inRange(hsv_frame, hsv_min[0], hsv_max[0])
        mask2 = cv2.inRange(hsv_frame, hsv_min[1], hsv_max[1])
        mask = cv2.bitwise_or(mask1, mask2)
    else:
        mask = cv2.inRange(hsv_frame, hsv_min[0], hsv_max[0])

    kernel = np.ones((5,5), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=1)
    mask = cv2.dilate(mask, kernel, iterations=2)
    
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    valid_contours = [c for c in contours if cv2.contourArea(c) > AREA_MIN_THRESHOLD]
    
    frame_width = frame.shape[1]

    if valid_contours:
        lost_target_count = 0  # Reset lost count
        largest_contour = max(valid_contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)
        x, y, w, h = cv2.boundingRect(largest_contour)

        # Record target position
        target_center_x = x + w // 2
        last_valid_position = target_center_x

        x = max(x - PADDING, 0)
        y = max(y - PADDING, 0)
        w += 2 * PADDING
        h += 2 * PADDING

        section_width = frame_width // 5
        
        if area > DISTANCE_THRESHOLD:
            move_backward()
            status = "Moving Backward"
        else:
            if target_center_x < section_width:
                move_left()
                status = "Turning Right"
            elif target_center_x > (section_width * 4):
                move_right()
                status = "Turning Left"
            else:
                move_forward()
                status = "Moving Forward"

        # Visual feedback
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.circle(frame, (target_center_x, y + h // 2), 5, (0, 0, 255), -1)
        cv2.putText(frame, f"Area: {area}", (x, y - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(frame, status, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # Region dividing line
        for i in range(1, 5):
            cv2.line(frame, (section_width * i, 0), (section_width * i, frame.shape[0]), (255, 0, 0), 1)

        # Distance indicator bar
        distance_ratio = min(max((area - DISTANCE_FAR) / (DISTANCE_NEAR - DISTANCE_FAR), 0), 1)
        bar_width = 200
        bar_height = 20
        bar_x = 20
        bar_y = frame.shape[0] - 40
        cv2.rectangle(frame, (bar_x, bar_y), (bar_x + bar_width, bar_y + bar_height), (0, 0, 0, 255), -1)
        cv2.rectangle(frame, (bar_x, bar_y), (int(bar_x + bar_width * distance_ratio), bar_y + bar_height), (0, 255, 0), -1)

    else:
        lost_target_count += 1
        
        # Immediately start searching, no need to wait
        status = search_target(frame_width)
        
        # Change search direction every 30 frames
        if lost_target_count % 30 == 0:
            search_direction *= -1
            # If last position is recorded, use it to determine new search direction
            if last_valid_position is not None:
                if last_valid_position < frame_width // 2:
                    search_direction = -1
                else:
                    search_direction = 1
        
        # Display search status
        cv2.putText(frame, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.putText(frame, f"Searching for {lost_target_count} frames", (10, 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        # Display search direction arrow
        arrow_x = frame_width // 2
        arrow_y = frame.shape[0] // 2
        arrow_length = 30
        if search_direction < 0:
            cv2.arrowedLine(frame, (arrow_x, arrow_y), (arrow_x - arrow_length, arrow_y), 
                          (0, 0, 255), 2)
        else:
            cv2.arrowedLine(frame, (arrow_x, arrow_y), (arrow_x + arrow_length, arrow_y), 
                          (0, 0, 255), 2)

    return frame

def capture_with_gpio_exit(hsv_min, hsv_max):
    """
    Real-time tracking with Picamera2 and GPIO exit functionality
    """
    setup_gpio()

    pygame.init()
    # Swap width and height to adapt to 90 degree rotation
    screen = pygame.display.set_mode((240, 180), pygame.FULLSCREEN)

    # Initialize Picamera2
    picam2 = Picamera2()
    # Adjust camera capture size to match rotated screen
    picam2.configure(picam2.create_preview_configuration(main={"size": (240, 180)}))
    picam2.start()
    print("Camera Started... Press GPIO button to exit")

    try:
        while True:  # Main loop runs forever until exit button is pressed
            try:
                # Capture frame
                frame = picam2.capture_array()
                frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

                # Detect and track target
                tracked_frame = detect_and_track_target(frame_bgr, hsv_min, hsv_max)

                # Convert frame for Pygame display and rotate
                tracked_frame_rgb = cv2.cvtColor(tracked_frame, cv2.COLOR_BGR2RGB)
                # Rotate 180 degrees first
                tracked_frame_rotated = cv2.rotate(tracked_frame_rgb, cv2.ROTATE_180)
                # Then rotate 90 degrees counterclockwise
                tracked_frame_rotated = cv2.rotate(tracked_frame_rotated, cv2.ROTATE_90_COUNTERCLOCKWISE)
                # Flip horizontally
                tracked_frame_flipped = cv2.flip(tracked_frame_rotated, 1)
                
                pygame_surface = pygame.surfarray.make_surface(tracked_frame_flipped)
                
                screen.blit(pygame_surface, (0, 0))
                pygame.display.flip()

                # Exit only when GPIO button is pressed
                if gpio_exit_check():
                    print("Exit button pressed, program terminating")
                    break

            except Exception as e:
                # If any error occurs during processing, print error but continue running
                print(f"Error occurred: {str(e)}")
                continue  # Continue processing next frame

    finally:
        # Clean up
        screen.fill((0, 0, 0))
        pygame.display.flip()
        
        picam2.stop()
        pygame.quit()
        GPIO.cleanup()
        print("Program safely terminated")
        
if __name__ == "__main__":
    # Improved red color detection range (dual range support)
    hsv_min = [np.array([0, 120, 70]), np.array([170, 120, 70])]  # Two HSV ranges for red
    hsv_max = [np.array([10, 255, 255]), np.array([180, 255, 255])]
    capture_with_gpio_exit(hsv_min, hsv_max)
