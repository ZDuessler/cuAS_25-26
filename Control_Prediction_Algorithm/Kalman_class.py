import numpy as np
import matplotlib.pyplot as plt
from typing import Any


class KalmanFilter:
    """Kalman Filter
    kf = KalmanFilter(A, B, C, Q, R)
    kf.set_init_values(x_init, P_init)
    xhat, P, K = kf.update(obs)

    Let A and C be, respectively, an n by n matirx and an m by n matrix. Then,
    xhat is the n by 1 state estimate, P will be the n by n state covariance matrix,
    and K will be the n by m Kalman gain.
    """

    def __init__(
        self,
        A: np.ndarray[Any, Any],
        B: np.ndarray[Any, Any],
        C: np.ndarray[Any, Any],
        Q: np.ndarray[Any, Any],
        R: np.ndarray[Any, Any],
    ) -> None:
        """Class constructor
        kf = KalmanFilter(A, B, C, Q, R) creates an LTI Kalman Filter.
        Args:
            A (_numpy array_): _System transition matrix_
            B (_numpy array): _Input matrix_
            C (_numpy array): _Observation matrix_
            Q (_numpy array_): _System covariance_
            R (_numpy array_): _Sbservation covariance_
        """

        if not isinstance(A, np.ndarray):
            raise TypeError("A must be a numpy ndarray.")
        if A.shape[0] != A.shape[1]:  # A must be an n by n matrix
            raise ValueError("A must be a square matrix (n x n).")

        if B.shape[0] != A.shape[0]:  # B must be an n by m matrix
            raise ValueError("B must be an n by m matrix (n x n).")

        if (
            Q.shape[0] != Q.shape[1] or Q.shape[0] != A.shape[0]
        ):  # Q must be an n by m matrix
            raise ValueError("Q must be a square matrix (n x n).")

        if A.shape[0] != C.shape[1]:  # C must be a k by n matrix
            raise ValueError("C must be a k by n matrix (k x n).")

        if (
            R.shape[0] != R.shape[1] or R.shape[0] != C.shape[0]
        ):  # C must be a k by n matrix
            raise ValueError("R must be a square matrix (k x k).")

        self.A = A
        self.B = B
        self.C = C
        self.Q = Q
        self.R = R
        self.xhat = np.array([])
        self.P = np.array([])
        self.K = np.array([])

    def set_init_values(self, x_init, Pinit) -> None:
        """_summary_

        Args:
            x_init (_numpy array_): _initial state estimate_
            Pinit (_numpy array_): _initial state covariance_
        """
        self.xhat = x_init
        self.P = Pinit

    def update(self, obs, u):
        """Update the optimal estimate
        xhat, P, K = kf.update(obs, u) updates the Kalman filter

        Args:
            obs (_numpy array_): _observation (measurement)_
            u (_numpy array_): _input_

        Returns:
            xhat (_numpy array_): n by 1 state estimate vector
            P (_numpy array_): n by n state covariance matrices,
            Pp (_numpy array_): n by n predicted state covariance matrices,
            K (_numpy array_): n by k Kalman gains

            where n is the state dimension, m is the input dimension, and k is the measurment dimesion.
        """

        """
        TODO: Implement the update function
        """
        # predict state estimate
        # xhat(k|k-1) = A*xhat(k-1|k-1) + B*u(k)
        xpred = (self.A @ self.xhat) + self.B * u

        # state prediction covariance matrix
        # P(k|k-1) = A*P(k-1|k-1)*A' + Q
        Ppred = self.A @ self.P @ np.transpose(self.A) + self.Q

        # innovation
        # z_tilde(k) = z(k) - C*xhat(k|k-1)
        innovation = obs - self.C @ xpred

        # innovation covariance
        # S = Cov[X(:,k) | y(:,1:k)] = C*P(k|k-1)*C' + R
        S = self.C @ Ppred @ np.transpose(self.C) + self.R

        # Kalman gain
        # K(k) = P(k|k-1)*C'*inv(S)
        self.K = Ppred @ np.transpose(self.C) @ np.linalg.inv(S)

        # update state estimate
        # xhat(k|k) = xhat(k|k-1) + K*z_tilde(k)
        self.xhat = xpred + self.K @ innovation

        # update state covariance matrix
        # P(k|k) = P(k|k-1) - K*C*P(k|k-1)
        self.P = Ppred - self.K @ self.C @ Ppred

        return self.xhat, self.P, Ppred, self.K


def kalman_function1(path_array, Ts):
    ###### Step 0. Incorportate Desired Path

    # Convert desired path from coordinates to a matrix
    x = []
    y = []
    for coords in path_array:
        x.append(coords[0])
        y.append(coords[1])


    path = np.row_stack((x, y))




    ####################### Step 1. Set Time Step ################################
    #Ts = 10 # Time step is every 10 seconds

    # Set other variables
    mu, sigma = 0, 4
    speed = 4.2  # m/s 
    radius = 120  # 120m
    z = path + np.random.normal(mu, sigma, path.shape)

    ####################### Step 2. Construct A, B, and C matricies ##############
    # TODO
    A = np.array([[1, 0, Ts, 0], [0, 1, 0, Ts], [0, 0, 1, 0], [0, 0, 0, 1]])
    B = np.array([[0], [0], [0], [0]])
    C = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])

    ####################### Step 3. Select System Covariance, Q ##################
    # TODO
    q_variance = sigma**2
    Q = np.eye(4)  # 4 by 4 identity matrix
    Q[2, 2] = q_variance
    Q[3, 3] = q_variance

    ####################### Step 4. Select Measurement Covariance, R #############
    # TODO
    r_variance = sigma**2
    R = np.array([[1, 0], [0, 1]])

    ####################### Step 5. Initialize Kalman Filter #####################
    # TODO: initialize Kalman filter
    kf = KalmanFilter(A, B, C, Q, R)
    x_init = np.array([[radius], [speed], [0], [0]])
    P_init = [[q_variance, 0, 0, 0], [0, q_variance, 0, 0], [0, 0, q_variance, 0], [0, 0, 0, q_variance]]

    kf.set_init_values(x_init, P_init)

    u = np.zeros_like(x_init)

    ####################### Step 6. Execute Kalman Filter ########################

    xhats = []  # python list, not numpy array
    state_covs = []  # python list, not numpy array
    state_pred_covs = []  # python list, not numpy array
    kalman_gains = []  # python list, not numpy array

    ####################### Step 7. Loop through observations to update ##########
    # TODO
    for obs in z.T:
        # obs must be a 2 by 1 vector
        xhat, state_cov, state_pred_cov, kalman_gain = kf.update(obs.reshape(2, 1), u)

        xhats.append(xhat)
        state_covs.append(state_cov)
        state_pred_covs.append(state_pred_cov)
        kalman_gains.append(kalman_gain)
        
    ####################### Step 8. Calculate RMSE Values ########################
    # TODO





    y0 = 60
    v0 = 20
    g = -9.8067
    Ts = 0.25

    t = np.linspace(0, 6, 25)
    mu, sigma = 0, 2
    np.random.seed(20)
    noise = np.random.normal(mu, sigma, len(t))

    y_true = 1 / 2 * g * t**2 + v0 * t + y0
    v_true = g * t + v0

    # observations (measurements)
    z = y_true + noise


    Ts = 0.25
    g = np.array([[-9.81]])

    A = np.array([[1, Ts], [0, 1]])
    B = np.array([[0], [Ts]])
    C = np.array([[1, 0]])

    # The dimensions of {A,B,C} are important
    print(f"Dimension of A: {A.shape}")
    print(f"Dimension of B: {B.shape}")
    print(f"Dimension of C: {C.shape}")

    """ 
    TODO: Select a system covariance that results in a RMSE of position estimates less than 2.0
    """
    # Select a system variance
    q_variance = sigma**2
    Q = np.eye(2) * q_variance

    # measurment variance
    R = np.array([[4]])

    print(f"Dimension of Q: {Q.shape}")
    print(f"Dimension of R: {R.shape}")

    # TODO: initialize Kalman filter
    kf = KalmanFilter(A, B, C, Q, R)
    x_init = np.array([[y0], [v0]])
    #P_init = np.array([[50, 50], [50, 50]])
    P_init = [[q_variance, 0], [0, q_variance]]
    kf.set_init_values(x_init, P_init)

    # Data collection
    xhats = []  # python list, not numpy array
    state_covs = []  # python list, not numpy array
    kalman_gains = []  # python list, not numpy array

    for obs in z:
        xhat, state_cov, _, kalman_gain = kf.update(obs, g)

        xhats.append(xhat)
        state_covs.append(state_cov)
        kalman_gains.append(kalman_gain)


    xhats = np.array(xhats)  # convert python list to numpy array
    #print(xhats)
    # reshape 25 x 2 X 1 array to 25 x 2 array
    # print(x_init.shape[0])
    # print(x_init.shape)
    xhats = xhats.reshape((len(z), x_init.shape[0]))

    kalman_gains = np.array(kalman_gains)  # convert python list to numpy arry
    # reshape 25 x 2 X 1 array to 25 x 2 array
    kalman_gains = kalman_gains.reshape((len(z), x_init.shape[0]))


    # numerical velocity
    v_numeric = np.append(v0, np.diff(z) / Ts)

    mse_pos_obs = np.square(z - y_true).mean()
    # print("Ytrue: ", y_true)
    # print("xhats: ", xhats[:, 0])
    mse_pos_est = np.square(xhats[:, 0] - y_true).mean()

    mse_vel_obs = np.square(v_numeric - v_true).mean()
    mse_vel_est = np.square(xhats[:, 1] - v_true).mean()

    # Return a path of predicted values
    print(xhats)
    return xhats
