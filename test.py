# This is code is just a test for using switch in python

import math

V_max = 9
a_long_max = 0
v_obs = 8
x = []
y = []
dyn_x = 0
dyn_y = 0
follow_dis = 0
v0 = 0
dyn1_x = 0
dyn1_y = 0
critical_dis = 0


def zero():
    v_max = V_max
    acc_long = a_long_max
    return v_max, acc_long


def one():
    v_max = min(v_obs, V_max)
    acc = 0.5 * (v_max ** 2 - v0 ** 2) / (math.sqrt((x[0] - dyn_x) ** 2 + (y[0] - dyn_y) ** 2) - follow_dis)
    acc_long = min(acc, a_long_max)
    return v_max, acc_long


def two():
    v_max = V_max
    acc_long = a_long_max
    return v_max, acc_long


def three():
    dis = math.sqrt((x[0] - dyn1_x) ** 2 + (y[0] - dyn1_y) ** 2) - critical_dis
    v_max = 0.0
    acc_long = max(-v0 ** 2 / (2 * abs(dis)), -V_max)
    return v_max, acc_long


def five():
    v_max = V_max
    acc_long = a_long_max
    return v_max, acc_long


def number(argument):
    switcher = {
        0: zero,
        1: one,
        2: two,
        3: three,
        5: five
    }
    func = switcher.get(argument, "Invalid month")
    return func()


v_max, acc_long = number(0)
print(v_max)


