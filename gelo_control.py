import hub
import utime
import math


class MotorFunctionFactory():
    """A class that produces motor control functions.

    Usage:
        factory = MotorFunctionFactory()
        motor_a_fn = factory.linear(angular_v=25, time_delay=0.25, offset=270)
        motor_b_fn = factory.linear(angular_v=25, time_delay=0.25, offset=90)
        time = <code to get time>
        motor_a_position = motor_a_fn(time)
        motor_b_position = motor_b_fn(time)
    """

    def linear(self, angular_v, time_delay=0, offset=0):
        """Returns function angular_v * x - time_delay * angular_v + offset."""
        y0 = - time_delay * factor + offset

        def function(x):
            return x * factor + y0
        return function

    def sine_wave(self, amplitude=100, period=1000, offset=0):
        """Returns function amplitude * sine((x - offset) / period * 2 * pi)."""
        def function(x):
            return math.sin((x-offset)/period*2*math.pi) * amplitude
        return function

    def block_wave(self, amplitude=100, period=1000, offset=0):
        """Returns function sign(sine_wave(amplitude, period, offset)(x))."""
        def function(ticks):
            phase = ticks % period
            if offset < phase < offset + period//2:
                return amplitude
            else:
                return -amplitude
        return function


class AMHTimer():
    """A configurable timer which you can start, reverse, stop and pause.

    By default, it counts milliseconds, but you can speed it up,
    Slow it down or accelerate it!
    You can also set the time and reset it.
    You can even run it in reverse, so you can count down until 0.
    It always returns integers, even when you slow it way down.

    Author: 
        Anton's Mindstorms Hacks - https://antonsmindstorms.com

    Usage:
        my_timer = AMHTimer():
        my_timer.rate = 500    # set the rate to 500 ticks/s. That is half the normal rate
        my_timer.acceleration = 100    # Increase the rate by 100 ticks / second squared
        my_timer.reset()    # Reset to zero. Doesn't change running/paused state
        now = mytimer.time    # Read the time
        mytimer.time = 5000    #Set the time
    """

    def __init__(self, rate=1000, acceleration=0):
        self.running = True
        self.pause_time = 0
        self.reset_at_next_start = False
        self.__speed_factor = rate / 1000
        self.__accel_factor = acceleration / 1000000
        self.start_time = utime.ticks_ms()

    @property
    def time(self):
        if self.running:
            elapsed = utime.ticks_diff(utime.ticks_ms(), self.start_time)
            return int(
                self.__accel_factor * elapsed**2 +
                self.__speed_factor * elapsed +
                self.pause_time
            )
        else:
            return self.pause_time

    @time.setter
    def time(self, setting):
        self.pause_time = setting
        self.start_time = utime.ticks_ms()

    def pause(self):
        if self.running:
            self.pause_time = self.time
            self.running = False

    def stop(self):
        self.pause()

    def start(self):
        if not self.running:
            self.start_time = utime.ticks_ms()
            self.running = True

    def resume(self):
        self.start()

    def reset(self):
        self.time = 0

    def reverse(self):
        self.rate *= -1

    @property
    def rate(self):
        elapsed = utime.ticks_diff(utime.ticks_ms(), self.start_time)
        return (self.__accel_factor*elapsed + self.__speed_factor) * 1000

    @rate.setter
    def rate(self, setting):
        if self.__speed_factor != setting / 1000:
            if self.running:
                self.pause()
            self.__speed_factor = setting / 1000
            self.start()

    @property
    def acceleration(self):
        return self.__accel_factor * 1000000

    @acceleration.setter
    def acceleration(self, setting):
        if self.__accel_factor != setting / 1000000:
            if self.running:
                self.pause()
            self.__speed_factor = self.rate / 1000
            self.__accel_factor = setting / 1000000
            self.start()


class MotorAndFunction():
    """A class to hold a motor and its control function."""

    def __init__(self, motor, control_fn):
        """Constructor.

        Args:
            motor: a hub.port.X.motor object.
            control_fn: a callable taking a single float arg and returns motor positions.
        """
        # Allow for both hub.port.X.motor and Motor('X') objects:
        if hasattr(motor, '_motor_wrapper'):
            self.motor = motor._motor_wrapper.motor
        else:
            self.motor = motor.motor
        self.control_fn = control_fn


class MotorControl():
    """
    The class helps to control multiple motors in a tight loop python program.

    Author:
        Anton's Mindstorms Hacks - https://antonsmindstorms.com


    Usage:
        my_mechanism = Mechanism([Motor('A'), Motor('B')], [func_a, func_b])
        timer = AMHTimer()
        while True:
            my_mechanism.update_motor_pwms(timer.time)
    """

    def __init__(self, motors_and_functions, reset_zero=True, ramp_pwm=100, Kp=1.2):
        """ Constructor.

        Args:
            motors and functions: list of MotorAndFunction objects.
            reset_zero: whether to reset the 0 point of the relative encoder to the absolute encoder
                position.
            ramp_pwm: int, a number to limit maximum pwm per tick when starting. 0.5 is a good
                value for a slow ramp.
            Kp: float, proportional feedback factor for motor power.
        """

        self.motors_and_functions = motors_and_functions
        self.ramp_pwm = ramp_pwm
        self.Kp = Kp
        if reset_zero:
            self.relative_position_reset()

    def relative_position_reset(self):
        """Set degrees counted of all motors according to absolute 0"""
        for motor in self.motors:
            absolute_position = motor.get()[2]
            if absolute_position > 180:
                absolute_position -= 360
            motor.preset(absolute_position)

    @staticmethod
    def float_to_motorpower(f):
        """Clip any float into an integer between -100 and 100."""
        return min(max(int(f), -100), 100)

    def update_motor_pwms(self, ticks):
        """Proportional controller toward desired motor positions at ticks"""
        for m_fn in self.motors_and_functions:
            target_position = m_fn.control_fn(ticks)
            current_position = m_fn.motor.get()[1]
            power = self.float_to_motorpower(
                (target_position-current_position) * self.Kp)
            if self.ramp_pwm < 100:
                # Limit pwm for a smooth start
                max_power = int(self.ramp_pwm*(abs(ticks)))
                power = max(min(power, max_power), -max_power)
            m_fn.motor.pwm(power)

    def shortest_path_reset(self, ticks=0, speed=20):
        # Get motors in position smoothly before starting the control loop

        # Reset internal tacho to range -180,180
        self.relative_position_reset()

        # Run all motors to a ticks position with shortest path
        for m_fn in self.motors_and_functions:
            target_position = int(m_fn.control_fn(ticks))
            current_position = m_fn.motor.get()[1]
            # Reset internal tacho so next move is shortest path
            if target_position - current_position > 180:
                m_fn.motor.preset(current_position + 360)
            if target_position - current_position < -180:
                m_fn.motor.preset(current_position - 360)
            # Start the manouver
            m_fn.motor.run_to_position(target_position, speed)

        # Give the motors time to spin up
        utime.sleep_ms(50)
        # Check all motors pwms until all maneuvers have ended
        while True:
            pwms = []
            for m_fn in self.motors_and_functions:
                pwms += [m_fn.motor.get()[3]]
            if not any(pwms):
                break

    def stop(self):
        for m_fn in self.motors_and_functions:
            m_fn.motor.pwm(0)


factory = MotorFunctionFactory()
motor_a_fn = factory.linear(-0.36)
motor_b_fn = factory.linear(0.36, time_delay=500)
motor_c_fn = factory.linear(-0.36, time_delay=500)
motor_d_fn = factory.linear(0.36)

motor_and_fns = [MotorAndFunction(Motor('A'), motor_a_fn),
                 MotorAndFunction(Motor('B'), motor_b_fn),
                 MotorAndFunction(Motor('C'), motor_c_fn),
                 MotorAndFunction(Motor('D'), motor_d_fn)]

gelo_movement = MotorControl(motor_and_fns)
timer = AMHTimer()
while True:
    gelo_movement.update_motor_pwms(timer.time)
