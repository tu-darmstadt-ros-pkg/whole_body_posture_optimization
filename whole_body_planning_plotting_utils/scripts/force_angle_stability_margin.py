import numpy as np


def segments(p):
    return zip(p, p[1:] + [p[0]])


def normalized(a, axis=-1, order=2):
    l2 = np.atleast_1d(np.linalg.norm(a, order, axis))
    l2[l2==0] = 1
    return a / np.expand_dims(l2, axis)


def norm(a):
    return np.atleast_1d(np.linalg.norm(a, 2, 0))


def force_angle_stability_margin(vertices, com):
    vertices_np = [np.array(vertex).reshape(3, 1) for vertex in vertices]
    I = np.eye(3)
    f_r = np.array([0, 0, -9.81]).reshape(3, 1)
    stabilities = []
    for (p1, p2) in segments(vertices_np):
        axis = p2 - p1
        axis_norm = normalized(axis, axis=0)
        l_i = (I - axis_norm.dot(axis_norm.transpose())).dot(p2 - com)
        l_i_norm = normalized(l_i, axis=0)
        f_i = (I - axis_norm.dot(axis_norm.transpose())).dot(f_r)
        f_i_norm = normalized(f_i, axis=0)
        d_i = -l_i + l_i.transpose().dot(f_i_norm) * f_i_norm

        if np.cross(f_i_norm, l_i_norm, axis=0).transpose().dot(axis_norm) > 0:
            sign = 1
        else:
            sign = -1
        theta_i = sign * np.arccos(f_i_norm.transpose().dot(l_i_norm))

        distance = norm(d_i)
        weight_force = norm(f_i)
        stability = theta_i * distance * weight_force * 1/np.abs(f_i[2])
        stabilities.append(stability)

    return stabilities


def main():
    vertices = [
        (1.72127, -0.262612, 0.106133),
        (1.71987, -0.262603, 0.157293),
        # (1.71745, 0.129414, 0.138245),
        # (1.7174, 0.209414, 0.138229),
        # (1.7211, 0.262624, 0.164542),
        # (2.6552, 0.263204, 0.0354168),
        # (2.65551, -0.21702, 0.0355132),
        # (2.60026, -0.26206, 0.0141214),
        # (2.50031, -0.262126, 0.0109553),
        # (2.35039, -0.262224, 0.00620616),
        # (1.83579, -0.262555, 0.0198238),
        # (1.7856, -0.262586, 0.0298429),
        # (1.74473, -0.262606, 0.0606462),
    ]
    # Stabilities: [0.0412742, 5.73714, 5.68681, 4.50367, 4.33073, 8.29936, 6.23878, 2.73369, 2.73369, 2.86917, 2.51777,
    # 1.54764, 0.735709, ]
    com = np.array([2.06912, -0.0172914, 0.110774]).reshape(3, 1)
    stabilities = force_angle_stability_margin(vertices, com)
    print(stabilities)


if __name__ == "__main__":
    main()
