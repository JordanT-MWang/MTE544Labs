from rclpy.time import Time
from utilities import Logger

# Controller types
P = 0    # Proportional
PD = 1   # Proportional + Derivative
PI = 2   # Proportional + Integral
PID = 3  # Proportional + Integral + Derivative


class PID_ctrl:

    def __init__(self, type_, kp=1.2, kv=0.8, ki=0.2,
                 history_length=10, filename_="errors.csv"):

        # Store controller settings and recent errors
        self.history_length = history_length
        self.history = []
        self.type = type_

        # Controller gains
        self.kp = kp  # Proportional gain
        self.kv = kv  # Derivative gain
        self.ki = ki  # Integral gain

        # Part 4: Create the CSV logger before using it
        self.logger = Logger(filename_)

    def update(self, stamped_error, status):

        if status == False:
            self.__update(stamped_error)
            return 0.0
        else:
            return self.__update(stamped_error)

    def __update(self, stamped_error):

        latest_error = stamped_error[0]
        stamp = stamped_error[1]

        self.history.append(stamped_error)

        if len(self.history) > self.history_length:
            self.history.pop(0)

        # Log startup samples too; derivative and integral are not available yet.
        if len(self.history) != self.history_length:
            self.logger.log_values([
                latest_error, 0.0, 0.0, Time.from_msg(stamp).nanoseconds
            ])
            return self.kp * latest_error

        # Calculate the time between measurements
        dt_avg = 0
        error_dot = 0

        for i in range(1, len(self.history)):
            t0 = Time.from_msg(self.history[i - 1][1])
            t1 = Time.from_msg(self.history[i][1])

            dt = (t1.nanoseconds - t0.nanoseconds) / 1e9
            dt_avg += dt

            # TODO Part 5: Calculate the error derivative.
            de = self.history[i][0] - self.history[i - 1][0]
            if dt > 0:
                error_dot += de / dt

        error_dot /= len(self.history)
        dt_avg /= len(self.history)

        # Calculate accumulated error
        sum_ = 0
        for hist in self.history:
            # TODO Part 5: Add errors for the integral calculation.
            sum_ += hist[0]
            

        error_int = sum_ * dt_avg

        # Part 4: Log error, derivative, integral and timestamp
        self.logger.log_values([
            latest_error,
            error_dot,
            error_int,
            Time.from_msg(stamp).nanoseconds
        ])

        # Part 4: Proportional controller
        if self.type == P:
            return self.kp * latest_error

        # TODO Part 5: Implement the remaining controllers.
        elif self.type == PD:
            return self.kp * latest_error + self.kv * error_dot


        elif self.type == PI:
            return self.kp * latest_error + self.ki * error_int


        elif self.type == PID:
             return self.kp * latest_error + self.kv * error_dot + self.ki * error_int
