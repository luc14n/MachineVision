import math

import numpy as np


class TrackingKalmanFilter:
    """
    Extended Kalman Filter (EKF) using a Constant Turn Rate and Velocity (CTRV) model.
    This allows the predictive engine to accurately follow and predict curving/arcing
    trajectories for human movements instead of just straight lines.
    """

    def __init__(self, dt, initial_state):
        """
        Initializes the EKF from the SVD "Cold Start".

        :param dt: Time step in seconds (e.g., 0.033 for 30 FPS).
        :param initial_state: Dictionary containing initial kinematics.
        """
        self.dt = dt

        # Extract initial Cartesian values from the SVD cold start
        px = initial_state["x"]
        py = initial_state["y"]
        vx = initial_state["vx"]
        vy = initial_state["vy"]

        # Convert to CTRV state space: [x, y, v, yaw, yaw_rate]^T
        v = math.sqrt(vx**2 + vy**2)
        yaw = math.atan2(vy, vx)
        yaw_rate = 0.0  # Initial turn rate is unknown, assume 0 (straight line)

        self.X = np.array(
            [
                [px],
                [py],
                [v],
                [yaw],
                [yaw_rate],
            ],
            dtype=float,
        )

        # Observation Matrix (H) - We only measure [x, y] from the camera
        self.H = np.array(
            [
                [1.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0, 0.0],
            ]
        )

        # Measurement Noise (R) - Camera Jitter
        self.R = np.array(
            [
                [5.0, 0.0],
                [0.0, 5.0],
            ]
        )

        # Process Noise (Q) - The Human Factor
        # High variance for velocity (v) and turn rate (yaw_rate) allows the filter
        # to react instantly to erratic human arcs and changes in speed.
        self.Q = np.diag([0.5, 0.5, 500.0, 5.0, 100.0])

        # Covariance Matrix (P) - Filter Confidence
        self.P = np.eye(5) * 10.0

    def _normalize_angle(self, angle):
        """Normalize angle to be between -pi and pi."""
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def _predict_state_and_jacobian(self, X, dt):
        """
        Computes the non-linear state transition f(X) and its Jacobian F_j.
        This handles the mathematical rotation of the velocity vector over time.
        """
        px = X[0, 0]
        py = X[1, 0]
        v = X[2, 0]
        yaw = X[3, 0]
        yaw_rate = X[4, 0]

        # Light damping to prevent infinite spinning loops when tracking is lost
        v_damped = v * 0.99
        yaw_rate_damped = yaw_rate * 0.95

        X_next = np.zeros((5, 1))
        F_j = np.eye(5)

        # Avoid division by zero when moving in a straight line
        if abs(yaw_rate_damped) < 0.001:
            # Basic kinematic prediction (straight line limit)
            X_next[0, 0] = px + v_damped * math.cos(yaw) * dt
            X_next[1, 0] = py + v_damped * math.sin(yaw) * dt
            X_next[2, 0] = v_damped
            X_next[3, 0] = yaw
            X_next[4, 0] = yaw_rate_damped

            # Jacobian Matrix for straight line (Taylor expansion limit)
            F_j[0, 2] = math.cos(yaw) * dt
            F_j[0, 3] = -v_damped * math.sin(yaw) * dt
            F_j[0, 4] = -0.5 * v_damped * math.sin(yaw) * (dt**2)

            F_j[1, 2] = math.sin(yaw) * dt
            F_j[1, 3] = v_damped * math.cos(yaw) * dt
            F_j[1, 4] = 0.5 * v_damped * math.cos(yaw) * (dt**2)
        else:
            # CTRV curving prediction
            X_next[0, 0] = px + (v_damped / yaw_rate_damped) * (
                math.sin(yaw + yaw_rate_damped * dt) - math.sin(yaw)
            )
            X_next[1, 0] = py + (v_damped / yaw_rate_damped) * (
                -math.cos(yaw + yaw_rate_damped * dt) + math.cos(yaw)
            )
            X_next[2, 0] = v_damped
            X_next[3, 0] = yaw + yaw_rate_damped * dt
            X_next[4, 0] = yaw_rate_damped

            # Jacobian Matrix for curve (Partial derivatives)
            F_j[0, 2] = (
                math.sin(yaw + yaw_rate_damped * dt) - math.sin(yaw)
            ) / yaw_rate_damped
            F_j[0, 3] = (v_damped / yaw_rate_damped) * (
                math.cos(yaw + yaw_rate_damped * dt) - math.cos(yaw)
            )
            F_j[0, 4] = (
                v_damped * dt * math.cos(yaw + yaw_rate_damped * dt)
            ) / yaw_rate_damped - (
                v_damped * (math.sin(yaw + yaw_rate_damped * dt) - math.sin(yaw))
            ) / (yaw_rate_damped**2)

            F_j[1, 2] = (
                -math.cos(yaw + yaw_rate_damped * dt) + math.cos(yaw)
            ) / yaw_rate_damped
            F_j[1, 3] = (v_damped / yaw_rate_damped) * (
                math.sin(yaw + yaw_rate_damped * dt) - math.sin(yaw)
            )
            F_j[1, 4] = (
                v_damped * dt * math.sin(yaw + yaw_rate_damped * dt)
            ) / yaw_rate_damped - (
                v_damped * (-math.cos(yaw + yaw_rate_damped * dt) + math.cos(yaw))
            ) / (yaw_rate_damped**2)

        F_j[3, 4] = dt

        X_next[3, 0] = self._normalize_angle(X_next[3, 0])

        return X_next, F_j

    def predict(self, dt=None):
        """
        Step 1: Predict the future state based purely on non-linear physics.
        """
        if dt is not None:
            self.dt = dt

        # Update State and Jacobian
        self.X, F_j = self._predict_state_and_jacobian(self.X, self.dt)

        # Update Covariance: P_new = F_j * P * F_j^T + Q
        self.P = F_j @ self.P @ F_j.T + self.Q

        return self.get_state()

    def check_gate(self, measurement, threshold=1e5):
        """
        Checks if a camera measurement is statistically valid using Mahalanobis distance.
        """
        Z = np.array(measurement, dtype=float).reshape(2, 1)

        # Innovation (Error between measurement and prediction)
        Y = Z - (self.H @ self.X)

        # Innovation Covariance
        S = self.H @ self.P @ self.H.T + self.R

        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return False

        md_sq = float((Y.T @ S_inv @ Y)[0, 0])
        return md_sq < threshold

    def update(self, measurement):
        """
        Step 2: Correct the prediction using a validated camera measurement.
        """
        Z = np.array(measurement, dtype=float).reshape(2, 1)

        # Innovation
        Y = Z - (self.H @ self.X)

        # Innovation Covariance
        S = self.H @ self.P @ self.H.T + self.R
        S_inv = np.linalg.inv(S)

        # Kalman Gain: K = P * H^T * S^-1
        K = self.P @ self.H.T @ S_inv

        # Update State: X_new = X + K * Y
        self.X = self.X + (K @ Y)
        self.X[3, 0] = self._normalize_angle(self.X[3, 0])

        # Update Confidence: P_new = (I - K * H) * P
        I = np.eye(5)
        self.P = (I - K @ self.H) @ self.P

        return self.get_state()

    def get_state(self):
        """
        Translates the CTRV polar state back into Cartesian coordinates
        so the main UI and SVD systems don't break.
        """
        px = float(self.X[0, 0])
        py = float(self.X[1, 0])
        v = float(self.X[2, 0])
        yaw = float(self.X[3, 0])

        vx = v * math.cos(yaw)
        vy = v * math.sin(yaw)

        return {
            "x": px,
            "y": py,
            "vx": vx,
            "vy": vy,
            "ax": 0.0,
            "ay": 0.0,
        }

    def get_covariance(self):
        """Returns the current covariance matrix P."""
        return self.P.copy()

    def predict_trajectory(self, steps=10, dt=None):
        """
        Predicts the future trajectory of the tracked object over a number of steps,
        yielding curving paths when yaw_rate != 0.
        Returns the predicted positions and their estimated error.
        """
        if dt is None:
            dt = self.dt

        trajectory = []
        X_sim = self.X.copy()
        P_sim = self.P.copy()

        for _ in range(steps):
            X_sim, F_j = self._predict_state_and_jacobian(X_sim, dt)
            P_sim = F_j @ P_sim @ F_j.T + self.Q

            trajectory.append(
                {
                    "x": float(X_sim[0, 0]),
                    "y": float(X_sim[1, 0]),
                    "cov_x": float(P_sim[0, 0]),
                    "cov_y": float(P_sim[1, 1]),
                }
            )

        return trajectory
