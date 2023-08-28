import pygame

# Constant(s)
ITER_TO_NEUTRAL = 15

# Control Variable(s)
neutral_iter = 0
max_speed = 100.0

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.cumulative_error = 0
        self.prev_setpoint = None

    def compute(self, setpoint, current_value):
        # If the setpoint has changed, reset the internal states
        if self.prev_setpoint is not None and setpoint != self.prev_setpoint:
            self.reset()

        error = setpoint - current_value
        self.cumulative_error += error
        error_derivative = error - self.prev_error

        output = self.kp * error + self.ki * self.cumulative_error + self.kd * error_derivative
        self.prev_error = error
        self.prev_setpoint = setpoint
        
        return output

    def reset(self):
        self.prev_error = 0
        self.cumulative_error = 0
        self.prev_setpoint = None


def movement_process(keys, movement, motor_speed):
    # Declare control variables as global
    global neutral_iter
    global max_speed

    pid = PIDController(0.075, 0.075, 0.075)
    desired_speed = motor_speed.copy()  # Create a copy so the original list isn't mutated directly

    # Movement stopped check
    if motor_speed == [0, 0]:
        movement = "Stop"

    # Check for movement keys
    if keys[pygame.K_w]:  # Forward
        if keys[pygame.K_d]:
            desired_speed = [max_speed, 0]
            movement = "Forward Right"
        elif keys[pygame.K_a]:
            desired_speed = [0, max_speed]
            movement = "Forward Left"
        else:
            desired_speed = [max_speed, max_speed]
            movement = "Forward"
        neutral_iter = 0
    elif keys[pygame.K_s]:  # Backward
        if keys[pygame.K_d]:
            desired_speed = [-max_speed, 0]
            movement = "Backward Right"
        elif keys[pygame.K_a]:
            desired_speed = [0, -max_speed]
            movement = "Backward Left"
        else:
            desired_speed = [-max_speed, -max_speed]
            movement = "Backward"
        neutral_iter = 0
    elif keys[pygame.K_d]:  # Turn Right
        desired_speed = [max_speed, -max_speed]
        movement = "Turn Right"
        neutral_iter = 0
    elif keys[pygame.K_a]:  # Turn Left
        desired_speed = [-max_speed, max_speed]
        movement = "Turn Left"
        neutral_iter = 0
    elif keys[pygame.K_SPACE] and motor_speed != [0, 0]:  # Brake
        movement = "Brake"
        neutral_iter = 0
        desired_speed = [0, 0]

    # Check for speed adjustment keys
    if keys[pygame.K_i] and max_speed < 100:  # Increase speed
        max_speed += 10
    if keys[pygame.K_k] and max_speed > 0:  # Decrease speed
        max_speed -= 10

    # Check for no movement
    if neutral_iter >= ITER_TO_NEUTRAL and motor_speed != [0, 0]:
        movement = "Neutral"
        desired_speed = [0, 0]

    neutral_iter += 1

    # Apply PID control and adjust motor speeds
    for i in range(2):
        if abs(desired_speed[i] - motor_speed[i]) <= 1:
            motor_speed[i] = desired_speed[i]
        else:
            motor_speed[i] += pid.compute(desired_speed[i], motor_speed[i])
            motor_speed[i] = round(motor_speed[i], 2)  # Round to two decimal places

    return movement, motor_speed, max_speed


def percentage_to_pwm(percentage, max_pwm_value):
    counter_turn = False
    pwm_val = (percentage / 100) * max_pwm_value
    if pwm_val < 0:
        counter_turn = True
    return abs(pwm_val), counter_turn
