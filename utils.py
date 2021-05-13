import numpy as np

# function for damped pseudo-inverse
def damped_pseudoinverse(jac, l = 0.01):
    m, n = jac.shape
    if n >= m:
        return jac.T @ np.linalg.inv(jac @ jac.T + l*l*np.eye(m))
    return np.linalg.inv(jac.T @ jac + l*l*np.eye(n)) @ jac.T

# function for skew symmetric
def skew_symmetric(v):
    mat = np.zeros((3, 3))
    mat[0, 1] = -v[2]
    mat[1, 0] = v[2]
    mat[0, 2] = v[1]
    mat[2, 0] = -v[1]
    mat[1, 2] = -v[0]
    mat[2, 1] = v[0]

    return mat

# function for Adjoint
def AdT(tf):
    R = tf.rotation()
    T = tf.translation()
    tr = np.zeros((6, 6))
    tr[0:3, 0:3] = R
    tr[3:6, 0:3] = skew_symmetric(T) @ R
    tr[3:6, 3:6] = R

    return tr

# angle wrap to [-pi,pi)
def angle_wrap(theta):
    while theta < -np.pi:
        theta += 2 * np.pi
    while theta > np.pi:
        theta -= 2 * np.pi
    return theta

# angle wrap multi
def angle_wrap_multi(theta):
    if isinstance(theta, list):
        th = theta
        for i in range(len(th)):
            th[i] = angle_wrap(th[i])
        return th
    elif type(theta) is np.ndarray:
        th = theta
        for i in range(theta.shape[0]):
            th[i] = angle_wrap(th[i])
        return th
    return angle_wrap(theta)
