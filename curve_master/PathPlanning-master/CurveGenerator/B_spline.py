
"""
Path Planner with clamped B-spline curve
author: Atsushi Sakai (@Atsushi_twi)
modified: Zhihai Bi
"""
import numpy as np
import matplotlib.pyplot as plt
import scipy.interpolate as scipy_interpolate

def approximate_b_spline_path(x, y, n_path_points, degree=3):
    # 6个控制点，阶数是3，所以需要的节点数目是 10个，所以t的取值和x相关
    t = range(len(x))
    x_tup = scipy_interpolate.splrep(t, x, k=degree)  # 插值函数
    y_tup = scipy_interpolate.splrep(t, y, k=degree)

    x_list = list(x_tup)
    x_list[1] = x + [0.0, 0.0, 0.0, 0.0]

    y_list = list(y_tup)
    y_list[1] = y + [0.0, 0.0, 0.0, 0.0]

    ipl_t = np.linspace(0.0, len(x) - 1, n_path_points)
    rx = scipy_interpolate.splev(ipl_t, x_list)
    ry = scipy_interpolate.splev(ipl_t, y_list)

    return rx, ry


def test():
    print(__file__ + " start!!")
    # 6个控制点
    way_point_x = [0, -2, 2.0, 3.5, 5.5, 6.0, 8.0]
    way_point_y = [-2, 0, 2.7, -0.5, 0.5, 3.0, 4.0]  #
    n_course_point = 100  # sampling number
    rax, ray = approximate_b_spline_path(way_point_x, way_point_y,
                                         n_course_point)

    # show results
    plt.plot(way_point_x, way_point_y, '-og', label="Control Points")
    plt.plot(rax, ray, '-r', label="Approximated B-Spline path")
    plt.grid(True)
    plt.title("B-spline clamped Curves")
    plt.legend()
    plt.axis("equal")
    plt.show()


if __name__ == '__main__':
    test()
