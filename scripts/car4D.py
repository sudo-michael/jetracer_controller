import math

class DubinsCar4D:
    def __init__(
        self,
        x=[0, 0, 0, 0],
        uMin=[-1.5, -math.pi / 12],
        uMax=[1.5, math.pi / 12],
        dMin=[0.0, 0.0],
        dMax=[0.0, 0.0],
        uMode="max",
        dMode="min",
    ):
        """Creates a Dublin Car with the following states:
           X position, Y position, acceleration, heading

           The first element of user control and disturbance is acceleration
           The second element of user control and disturbance is heading


        Args:
            x (list, optional): Initial state . Defaults to [0,0,0,0].
            uMin (list, optional): Lowerbound of user control. Defaults to [-1,-1].
            uMax (list, optional): Upperbound of user control.
                                   Defaults to [1,1].
            dMin (list, optional): Lowerbound of disturbance to user control, . Defaults to [-0.25,-0.25].
            dMax (list, optional): Upperbound of disturbance to user control. Defaults to [0.25,0.25].
            uMode (str, optional): Accepts either "min" or "max".
                                   * "min" : have optimal control reach goal
                                   * "max" : have optimal control avoid goal
                                   Defaults to "min".
            dMode (str, optional): Accepts whether "min" or "max" and should be opposite of uMode.
                                   Defaults to "max".
        """
        self.x = x
        self.uMax = uMax
        self.uMin = uMin
        self.dMax = dMax
        self.dMin = dMin
        assert uMode in ["min", "max"]
        self.uMode = uMode
        if uMode == "min":
            assert dMode == "max"
        else:
            assert dMode == "min"
        self.dMode = dMode

    def opt_ctrl(self, state, spat_deriv):
        """
        :param t: time t
        :param state: tuple of coordinates
        :param spat_deriv: tuple of spatial derivative in all dimensions
        :return:
        """
        # System dynamics
        # x_dot     = v * cos(theta) + d_1
        # y_dot     = v * sin(theta) + d_2
        # v_dot = a
        # theta_dot = v * tan(delta) / L

        # Graph takes in 4 possible inputs, by default, for now
        opt_a = self.uMax[0]
        opt_w = self.uMax[1]
        if self.uMode == "min":
            if spat_deriv[2] > 0:
                opt_a = self.uMin[0]
            if spat_deriv[3] > 0:
                opt_w = self.uMin[1]
        else:
            if spat_deriv[2] < 0:
                opt_a = self.uMin[0]
            if spat_deriv[3] < 0:
                opt_w = self.uMin[1]
        return opt_a, opt_w

    def dynamics(self, t, state, uOpt):
        L = 0.3
        x_dot = state[2] * math.cos(state[3])
        y_dot = state[2] * math.sin(state[3])
        v_dot = uOpt[0]
        theta_dot = state[2] * math.tan(uOpt[1]) / L

        return (x_dot, y_dot, v_dot, theta_dot)

