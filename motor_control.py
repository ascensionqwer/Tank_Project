# Constant(s)
ITER_TO_NEUTRAL = 15

# Control Variable(s)
neutral_iter = 0
stopped = True
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


def movement_process(key, movement, motor_speed):
    # Declare control variables as global
    global neutral_iter
    global stopped
    global max_speed

    stopped = False
    pid = PIDController(0.3, 0.03, 0.03)
    desired_speed = motor_speed

    # Process the movement inputs and translate it to the motors and return the string of the action
    if key == ord('w') or key == ord('W'):
        movement = "Forward"
        desired_speed = [max_speed, max_speed]
        neutral_iter = 0
    elif key == ord('s') or key == ord('S'):
        movement = "Backward"
        desired_speed = [-max_speed, -max_speed]
        neutral_iter = 0
    elif key == ord('d') or key == ord('D'):
        movement = "Turn Right"
        desired_speed = [max_speed, -max_speed]
        neutral_iter = 0
    elif key == ord('a') or key == ord('A'):
        movement = "Turn Left"
        desired_speed = [-max_speed, max_speed]
        neutral_iter = 0
    elif key == ord(' ') and motor_speed != [0, 0]:
        movement = "Brake"
        neutral_iter = 0
        desired_speed = [0, 0]
    elif (key == ord('i') or key == ord('I')):
        if max_speed < 100:
            max_speed += 10
    elif (key == ord('k') or key == ord('K')):
        if max_speed > 0:
            max_speed -= 10
    elif neutral_iter >= ITER_TO_NEUTRAL and motor_speed != [0, 0]:
        movement = "Neutral"
        desired_speed = [0, 0]  # Both motors going to a halt for neutral state
        if motor_speed == [0, 0]:
            stopped = True
    
    neutral_iter += 1
    if stopped is True:
        movement = "Stop"
    
    for i in range(2):
        # less than one percent, motor_speed =  desired_speed
        if abs((desired_speed[i] - motor_speed[i])) <= 1:
            motor_speed[i] = desired_speed[i]
            continue
            
        # PID controllers adjust the motor speeds
        motor_speed[i] += pid.compute(desired_speed[i], motor_speed[i])
        # Round to two decimal places
        motor_speed[i] = round(motor_speed[i], 2)

    return movement, motor_speed, max_speed

def percentage_to_pwm(percentage, max_pwm_value):
    return (percentage / 100) * max_pwm_value
