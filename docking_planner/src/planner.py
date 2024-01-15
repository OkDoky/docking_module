#! /usr/bin/python
import math

# import matplotlib.pyplot as plt
import numpy as np

# parameter
MAX_T = 10.0  # maximum time to the goal [s]
MIN_T = 5.0  # minimum time to the goal[s]

show_animation = False


class QuinticPolynomial:

    def __init__(self, xs, vxs, axs, xe, vxe, axe, time):
        # calc coefficient of quintic polynomial
        # See jupyter notebook document for derivation of this equation.
        self.a0 = xs
        self.a1 = vxs
        self.a2 = axs / 2.0

        A = np.array([[time ** 3, time ** 4, time ** 5],
                      [3 * time ** 2, 4 * time ** 3, 5 * time ** 4],
                      [6 * time, 12 * time ** 2, 20 * time ** 3]])
        b = np.array([xe - self.a0 - self.a1 * time - self.a2 * time ** 2,
                      vxe - self.a1 - 2 * self.a2 * time,
                      axe - 2 * self.a2])
        x = np.linalg.solve(A, b)

        self.a3 = x[0]
        self.a4 = x[1]
        self.a5 = x[2]

    def calc_point(self, t):
        return self.a0 + self.a1 * t + self.a2 * t ** 2 + \
             self.a3 * t ** 3 + self.a4 * t ** 4 + self.a5 * t ** 5

    def calc_first_derivative(self, t):
        return self.a1 + 2 * self.a2 * t + \
             3 * self.a3 * t ** 2 + 4 * self.a4 * t ** 3 + 5 * self.a5 * t ** 4

    def calc_second_derivative(self, t):
        return 2 * self.a2 + 6 * self.a3 * t + 12 * self.a4 * t ** 2 + 20 * self.a5 * t ** 3

    def calc_third_derivative(self, t):
        return 6 * self.a3 + 24 * self.a4 * t + 60 * self.a5 * t ** 2

def getTime(sx, sy, gx, gy, sv, gv, max_accel):
    prob_length = math.hypot(sy - gy, sx - gx)
    try:
        result_time = (gv - sv)/max_accel + (1/gv)*((prob_length)*1.2 - (gv - sv)*(gv + sv)/(2*max_accel))
    except ZeroDivisionError:
        result_time = prob_length * 5
    return result_time

def quintic_polynomials_planner(sx, sy, syaw, gx, gy, gyaw, dt, sv = 0.0, sa=0.02, gv=0.2, \
                                ga=0.05, max_accel=0.05, max_jerk=0.05):
    """
    quintic polynomial planner

    input
        s_x: start x position [m]
        s_y: start y position [m]
        s_yaw: start yaw angle [rad]
        sa: start accel [m/ss]
        gx: goal x position [m]
        gy: goal y position [m]
        gyaw: goal yaw angle [rad]
        ga: goal accel [m/ss]
        max_accel: maximum accel [m/ss]
        max_jerk: maximum jerk [m/sss]
        dt: time tick [s]

    return
        time: time result
        rx: x position result list
        ry: y position result list
        ryaw: yaw angle result list
        rv: velocity result list
        ra: accel result list

    """

    vxs = sv * math.cos(syaw)
    vys = sv * math.sin(syaw)
    vxg = gv * math.cos(gyaw)
    vyg = gv * math.sin(gyaw)

    axs = sa * math.cos(syaw)
    ays = sa * math.sin(syaw)
    axg = ga * math.cos(gyaw)
    ayg = ga * math.sin(gyaw)

    time, rx, ry, ryaw, rv, ra, rj = [], [], [], [], [], [], []

    T = getTime(sx, sy, gx, gy, sv, gv, max_accel)

    xqp = QuinticPolynomial(sx, vxs, axs, gx, vxg, axg, T)
    yqp = QuinticPolynomial(sy, vys, ays, gy, vyg, ayg, T)

    time, rx, ry, ryaw, rv, ra, rj = [], [], [], [], [], [], []

    for t in np.arange(0.0, T + dt, dt):
        time.append(t)
        rx.append(xqp.calc_point(t))
        ry.append(yqp.calc_point(t))

        vx = xqp.calc_first_derivative(t)
        vy = yqp.calc_first_derivative(t)
        v = np.hypot(vx, vy)
        yaw = math.atan2(vy, vx)
        rv.append(v)
        ryaw.append(yaw)

        ax = xqp.calc_second_derivative(t)
        ay = yqp.calc_second_derivative(t)
        a = np.hypot(ax, ay)
        if len(rv) >= 2 and rv[-1] - rv[-2] < 0.0:
            a *= -1
        ra.append(a)

        jx = xqp.calc_third_derivative(t)
        jy = yqp.calc_third_derivative(t)
        j = np.hypot(jx, jy)
        if len(ra) >= 2 and ra[-1] - ra[-2] < 0.0:
            j *= -1
        rj.append(j)

    if max([abs(i) for i in ra]) <= max_accel and max([abs(i) for i in rj]) <= max_jerk:
        print("find path!!")
        # break

    return time, rx, ry, ryaw, rv, ra, rj


def plot_arrow(x, y, yaw, length=0.1, width=0.05, fc="r", ec="k"):  # pragma: no cover
    """
    Plot arrow
    """

    if not isinstance(x, float):
        for (ix, iy, iyaw) in zip(x, y, yaw):
            plot_arrow(ix, iy, iyaw)
