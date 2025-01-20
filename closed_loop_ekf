
def sgn(x):
    if x > 0:
        return 1
    elif x < 0:
        return -1
    else:
        return 0


def stribeck_friction(v):
    """
    Stribeck friction model.

    Parameters:
    v (float): velocity
    F_s (float): static friction force
    F_c (float): Coulomb friction force
    v_s (float): characteristic sliding velocity
    K_v (float): viscous damping term

    Returns:
    float: friction force at given velocity v
    """
    F_s = mu_s2 * normal_force2
    F_c = mu_c2 * normal_force2

    # Stribeck curve
    f_stribeck = F_c + (F_s - F_c) * math.exp(-(np.abs(v / v_s)))

    # Total friction (with sign of velocity and viscous damping)
    f_total = f_stribeck * sgn(v) + K_v2 * v

    # Ensure friction is between F_s and F_c
    if sgn(v) > 0:
        return min(f_total, F_s)
    elif sgn(v) < 0:
        return max(f_total, F_c)#-F_s
    else:
        return f_total


# State transition function (prediction step)
def state_transition(state, dt, u):
    p1, v1, p2, v2 = state

    offset = (p1-p2)**2
    mag_func = mag_force-((3/2)*offset*(mag_force**2))
    # mag_func = CONS * mag_func
    friction_top = stribeck_friction(v2)

    # bottom magnet
    p1_new = p1 + dt * v1
    v1_new = v1 + dt * (u / m1 - (K_v1 * v1)/m1 - mag_func/m1)

    # top magnet
    p2_new = p2 + dt * v2
    v2_new = v2 + dt * (mag_func/m2 - friction_top/m2)

    return np.array([p1_new, v1_new, p2_new, v2_new])

# Measurement update function (correction step)
def measurement_update1(state_mean, state_covariance, measurement, measurement_noise_covariance):
    H = np.array([[1, 0, 0, 0], [0, 0, 1, 0]])  # Measurement matrix (only measuring positions)
    R = measurement_noise_covariance

    # Kalman gain calculation
    S = H @ state_covariance @ H.T + R
    S = S + np.eye(S.shape[0]) * 1e-6
    K = state_covariance @ H.T @ np.linalg.inv(S)

    # Measurement residual
    y = measurement - H @ state_mean

    # State update
    state_mean_new = state_mean + K @ y

    # Covariance update (Joseph form to ensure positive semi-definiteness)
    I = np.eye(4)
    state_covariance_new = (I - K @ H) @ state_covariance @ (I - K @ H).T + K @ R @ K.T

    return state_mean_new, state_covariance_new

def measurement_update2(state_mean, state_covariance, measurement, measurement_noise_covariance):
    H = np.array([[1, 0, 0, 0]])  # Measurement matrix (only measuring positions)
    R = measurement_noise_covariance

    # Kalman gain calculation
    S = H @ state_covariance @ H.T + R
    S = S + np.eye(S.shape[0]) * 1e-6
    K = state_covariance @ H.T @ np.linalg.inv(S)

    # print('R {}, s {}, K {}: '.format(R, S, K))
    # Measurement residual
    # print(measurement, state_mean)
    y = measurement - H @ state_mean

    # State update
    state_mean_new = state_mean + K @ y


    # Covariance update (Joseph form to ensure positive semi-definiteness)
    I = np.eye(4)
    state_covariance_new = (I - K @ H) @ state_covariance @ (I - K @ H).T + K @ R @ K.T

    # print('y {}, state_mean_new {}, state_covariance_new {}'.format(y, state_mean_new, state_covariance_new))

    return state_mean_new, state_covariance_new

def jacob_state(f, x, dt, u):
    # Calculate the Jacobian of the state transition function f at state x and control input u
    epsilon = 1e-6
    F = np.zeros((len(x), len(x)))

    for i in range(len(x)):
        x_plus = x.copy()
        x_minus = x.copy()
        x_plus[i] += epsilon
        x_minus[i] -= epsilon
        F[:, i] = (f(x_plus, dt, u) - f(x_minus, dt, u)) / (2 * epsilon)

    return F
