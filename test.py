# # This is code is just a test for using switch in python
#
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

# import math

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

# matrix = np.zeros((5, 4))
# print(matrix)
# print(len(matrix))
# a = np.zeros((len(matrix), 1))
# print(a)
# print(a[-1])

# d = 4 + 10 / 5
# print(d ** 2)

b = [[1],
     [2],
     [0],
     [4]]
# print(b)
# c = max(b)
# print(c)
#
# minElement = np.amin(b)
# print(minElement)
#
# print(np.argmin(b))
# print(np.subtract(b, -2))
#
# a = np.array([[2, 3, 2]]).T
# print(len(a))
# print(np.power(b, 1.5))
#
# print(b[3])
# print("hello")
print("hello world")

d = np.array([[2, 6, 4, 5, 1, 7]]).T
print(d)
#
# print(d[0])
# # g = []
g = np.zeros((len(d), 1))
# print(g)

s = 4
for i in range(0, s):
    g[i] = d[i]

# print(len(g))
print(g)


#
#
def function(aa, bb):
    for ii in range(0, 7):
        aa[ii] = aa[ii] + 2
        bb[ii] = bb[ii] + 3

    return aa, bb


#
#
# (q, w) = function(d, g)
# print("hello3")
# print(q)
# # print(np.subtract((q, 2)))
# # print(np.add(q,2))
#
# # for i in range(1, len(q)-1):
# #     print(np.add(q[i-1],q[i],q[i+1]))
# #     print('h')
# #     t = np.add(q[i-1],q[i])
# #     w = np.add(t, q[i+1])
# #     print(w)
#
# print(d)

# f = np.add(d[0], d[1], d[2])
# print(d[0])
# print(d[1])
# print(d[2])
# # print(f)
# f = np.add(d[0], d[1])
# print(f)
# g = np.add(f, d[2])
# h = 0.5 * d
# print(np.divide(d, h))
# k = np.zeros((len(d), 1))
# for i in range(1, len(d) - 1):
#     area1 = d[i - 1] * (g[i] - g[i + 1])
#     area2 = d[i] * (g[i + 1] - g[i - 1])
#     area3 = d[i + 1] * (g[i - 1] - g[i])
#     area = 0.5 * (area1 + area3 + area2)
#     s1 = math.sqrt((d[i - 1] - d[i]) ** 2 + (g[i - 1] - g[i]) ** 2)
#     s2 = math.sqrt((d[i + 1] - d[i]) ** 2 + (g[i + 1] - g[i]) ** 2)
#     s3 = math.sqrt((d[i - 1] - d[i + 1]) ** 2 + (g[i - 1] - g[i + 1]) ** 2)
#     s4 = s1 * s2 * s3
#     result = area / s4
#     k[i] = 4 * result
# k[0] = k[1]
# k[len(d) - 1] = k[len(d) - 2]
# print(k)
# print(len(k))
# print('h')

# def curvature_fn(x, y):
#     dx = np.gradient(x, axis=0)
#     ddx = np.gradient(dx, axis=0)
#     dy = np.gradient(y, axis=0)
#     ddy = np.gradient(dy, axis=0)
# A = np.multiply(dx, ddy)
# B = np.multiply(dy, ddx)
# C = np.subtract(A, B)
# D = np.multiply(dx, dx)
# E = np.multiply(dy, dy)
# F = np.add(D, E)
# G = np.power(F, 1.5)
# k_matrix = np.divide(C, G)
# for i in range(0, len(x)):
#     k[i] = (dx[i] * ddy[i] - dy[i] * ddx[i]) / ((dx[i] ** 2 + dy[i] ** 2) ** 1.5)
# return k


# print(curvature_fn(d, g))
# coeff = np.array([[1, 2, 3]]).T
# print(coeff)
#
# # print(np.roots(coeff)
P = np.array([[1, 2, 3],
              [4, 5, 6],
              [7, 8, 9]])
print(P)
Q = ([[1, 2, 3, 4],
      [4, 5, 6, 4],
      [7, 8, 9, 4]])
print(Q)
print(len(Q[0]))
# print(P[:, 0])
# G = P[:, 0]
# print(np.sqrt((np.square((G - 2)))))
# print(np.sum(G))
# print(3 * G)
#
# a = np.array([[2], [3], [4]])
# print(a)
# print(len(a))
#
# b = [1, 2, 3]
# print(b)
# print(len(b))
# print(np.dot(b, a))
# print(np.dot(2, b))
# print(np.add(2, b))
# print(np.zeros((1, 5)))
# x_store = [[1], [2], [3], [4], [5], [6], [7], [8], [9]]
# xd = np.add(x_store, 1)
# print(xd)
# print(len(x_store))
#
# print(x_store)
# v = np.zeros((7, 1))
# (c, d) = function(d, g)
# print(v[4, 0])
# cc = np.zeros((5, 1))
# print(cc)
#
# var = 0
# for h in range(0, 1):
#     var = cc[h]
#     print('h')
#
# print(var)
#
# print(len(np.array([[2, 3, 4]])))
#
#
a = 1
f = np.dot(np.array([1, a, a ** 2, a ** 3, a ** 4, a ** 5]), d)
print(f)
v = np.array([1, 2, 3, 4, 5, 6])

print(len(v))
r = np.roots(v)
print(r)
td_n = 2
z = np.array([1, td_n, td_n ** 2, td_n ** 3, td_n ** 4, td_n ** 5])
print(z)
v = np.array([1, a, a, a, a, a])
print(v)
print(a*v)
