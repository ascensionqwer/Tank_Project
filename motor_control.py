# Constants
INCREMENT_PERCENT = 0.2
ZERO_DEACCELERATION = 1
BRAKE_RATE = 30
ITER_TO_NEUTRAL = 15

# Control Variable(s)
neutral_iter = 0
stopped = True
brake = False
max_speed = 100.0


def movement_process(key, movement, motor_speed):
    # Declare control variables as global
    global neutral_iter
    global stopped
    global max_speed
    global brake

    brake = False
    stopped = False

    desired_speed = motor_speed

    # Process the movement inputs and translate it to the motors and return the string of the action
    if key == ord('w') or key == ord('W'):
        movement = "Forward"
        desired_speed = [max_speed, max_speed]
        neutral_iter = 0
        stopped = False
    elif key == ord('s') or key == ord('S'):
        movement = "Backward"
        desired_speed = [-max_speed, -max_speed]
        neutral_iter = 0
        stopped = False
    elif key == ord('d') or key == ord('D'):
        movement = "Turn Right"
        desired_speed = [max_speed, -max_speed]
        neutral_iter = 0
        stopped = False
    elif key == ord('a') or key == ord('A'):
        movement = "Turn Left"
        desired_speed = [-max_speed, max_speed]
        neutral_iter = 0
        stopped = False
    elif key == ord(' ') and motor_speed != [0, 0]:
        movement = "Brake"
        brake = True
        neutral_iter = 0
        desired_speed = [0, 0]
    elif (key == ord('i') or key == ord('I')) and motor_speed == [0, 0]:
        if max_speed < 100:
            max_speed += 10
    elif (key == ord('k') or key == ord('K')) and motor_speed == [0, 0]:
        if max_speed > 0:
            max_speed -= 10
    elif neutral_iter >= ITER_TO_NEUTRAL and motor_speed != [0, 0]:
        movement = "Neutral"
        desired_speed = [0, 0]  # Both motors going to a halt for neutral state

    if motor_speed == [0, 0]:
        stopped = True

    if stopped is True:
        movement = "Stop"
        

    motor_speed = gradual_motor_change(motor_speed, desired_speed)
    return movement, motor_speed, max_speed

def gradual_motor_change(motor_speed, desired_speed):
    global neutral_iter
    global brake

    for i in range(2):  # Iterate for both motors
        if brake is True:
            if abs(motor_speed[i] - desired_speed[i]) < BRAKE_RATE:
                motor_speed[i] = desired_speed[i]
                continue
            else:
                motor_speed[i] -= BRAKE_RATE * (1 if motor_speed[i] > 0 else -1)  # This makes sure the direction is right.
                continue

        if desired_speed == [0, 0]:
            if neutral_iter <= ITER_TO_NEUTRAL:
                continue
            if abs(motor_speed[i] - desired_speed[i]) < ZERO_DEACCELERATION:
                motor_speed[i] = desired_speed[i]
                continue
            else:
                motor_speed[i] -= ZERO_DEACCELERATION * (1 if motor_speed[i] > 0 else -1)  # This makes sure the direction is right.
                continue

        difference = desired_speed[i] - motor_speed[i]
        # If the difference is within increment, set the speed directly
        if abs(difference) <= INCREMENT_PERCENT * max_speed:  # Use MAX_SPEED here instead of desired_speed[i]
            motor_speed[i] = desired_speed[i]
        else:
            # Add or subtract the increment depending on whether we need to speed up or slow down
            motor_speed[i] += INCREMENT_PERCENT * max_speed * (1 if difference > 0 else -1)  # Again, use MAX_SPEED instead of desired_speed[i]

    neutral_iter += 1
    return motor_speed

def percentage_to_pwm(percentage, max_pwm_value):
    return (percentage / 100) * max_pwm_value
