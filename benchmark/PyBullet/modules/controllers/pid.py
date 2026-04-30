class PID:
    def __init__(self, kp, ki, kd, dt):
        self.kp = kp  # Proportional gain
        self.ki = ki  # Integral gain
        self.kd = kd  # Derivative gain
        self.dt = dt  # Time step

        self.prev_error = 0.0       # e(t-1)
        self.prev_prev_error = 0.0  # e(t-2)
        self.output = 0.0           # u(t-1)

    def reset(self):
        self.prev_error = 0.0
        self.prev_prev_error = 0.0
        self.output = 0.0

    def update(self, error):
        """
        error: current error e(t)
        returns: updated control output u(t)
        """
        # Incremental PID formula:
        delta_output = (
            self.kp * (error - self.prev_error)
            + self.ki * error * self.dt
            + self.kd * (error - 2 * self.prev_error + self.prev_prev_error) / self.dt
        )

        self.output += delta_output

        # Update errors history
        self.prev_prev_error = self.prev_error
        self.prev_error = error

        return self.output
