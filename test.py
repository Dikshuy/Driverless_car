# # This is code is just a test for using switch in python
#
# import math
#
# V_max = 9
# a_long_max = 0
# v_obs = 8
# x = []
# y = []
# dyn_x = 0
# dyn_y = 0
# follow_dis = 0
# v0 = 0
# dyn1_x = 0
# dyn1_y = 0
# critical_dis = 0
#
#
# def zero():
#     v_max = V_max
#     acc_long = a_long_max
#     return v_max, acc_long
#
#
# def one():
#     v_max = min(v_obs, V_max)
#     acc = 0.5 * (v_max ** 2 - v0 ** 2) / (math.sqrt((x[0] - dyn_x) ** 2 + (y[0] - dyn_y) ** 2) - follow_dis)
#     acc_long = min(acc, a_long_max)
#     return v_max, acc_long
#
#
# def two():
#     v_max = V_max
#     acc_long = a_long_max
#     return v_max, acc_long
#
#
# def three():
#     dis = math.sqrt((x[0] - dyn1_x) ** 2 + (y[0] - dyn1_y) ** 2) - critical_dis
#     v_max = 0.0
#     acc_long = max(-v0 ** 2 / (2 * abs(dis)), -V_max)
#     return v_max, acc_long
#
#
# def five():
#     v_max = V_max
#     acc_long = a_long_max
#     return v_max, acc_long
#
#
# def number(argument):
#     switcher = {
#         0: zero,
#         1: one,
#         2: two,
#         3: three,
#         5: five
#     }
#     func = switcher.get(argument, "Invalid month")
#     return func()
#
#
# v_max, acc_long = number(0)
# print(v_max)
#
#
# state = 2
# global x, y, i
# e = 0
# i = 2
#
#
# def function_for_test(a, b):
#     new_a = a
#     new_b = b
#     return new_a, new_b
#
#
# while i > 0:
#     if state == 2:
#         x = 10
#         y = 13
#         (c, d) = function_for_test(x, y)
#         c = c + 1
#
#     for j in range(0, 2):
#         e = c
#         print(e)
#
#     i = i - 1
import numpy as np
# num_foc_t = 5
# xd = []
#
# td = 5
# a_coeff = np.array([[1], [2], [3] , [4] , [5], [6]])
# print([[1], [td], [td**2], [td**3], [td**4], [td**5]])
# print(a_coeff)
#
# xd = [[1], [td], [td**2], [td**3], [td**4], [td**5]]*a_coeff
# print(xd)
# print(len(xd))
#
# j = 5
# if td == 5:
#     j = j + 5
#
# print(j)
#
# a = np.random.randint(0, 20, 10)
# print(a)
# print(np.where(a > 5))

matrix = np.zeros((5,4))
# print(matrix)
# print(len(matrix))
a = np.zeros((len(matrix), 1))
# print(a)
print(a[-1])

d = 4 + 10 / 5
print(d ** 2)

b = [[2],
     [2],
     [3],
     [4]]
print(b)
c = max(b)
print(c)


