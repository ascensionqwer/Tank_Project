import cv2
import pygame

from image_processing import *
from motor_control import *

def main():
    # Control Variables
    visual_ctrl = "Normal"
    turn_on_obj = False
    movement = "Neutral"
    motor_speed = [0.0, 0.0] # [left, right]
    max_speed = 100.0

    # Key pressed function initialize
    pygame.init()

    # Initialize the webcam
    cap = cv2.VideoCapture(0)

    # Check if camera opened successfully
    if not cap.isOpened():
        print("Error: Couldn't open the camera.")
        return

    # Intro Frame
    display_image("MonkiCorp.png")
    
    try:
        while True:
            # Capture frame-by-frame
            ret, frame = cap.read()
            if not ret:
                print("Failed to grab frame")
                break
              
            # Process pygame events
            pygame.event.pump()
            
            # Get the list of keys currently pressed
            keys = pygame.key.get_pressed()

            # Movement Process:
            movement, motor_speed, max_speed = movement_process(keys, movement, motor_speed)

            # Visual Image Processing Control
            visual_ctrl, turn_on_obj = input_control(keys, visual_ctrl, turn_on_obj)

            # Display the processed frame
            cv2.imshow("RC Visual", image_process(frame, device, visual_ctrl, turn_on_obj, movement, motor_speed, max_speed))

            # Press 'q' to break the main loop and end the program
            if keys[pygame.K_q]:
                break
    finally:
        # When everything done, release the video capture object
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
