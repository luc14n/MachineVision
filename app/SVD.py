import numpy as np


class SVDKinematicFitter:
    """
    Fits a physical kinematic model to a short buffer of noisy camera frames
    using Singular Value Decomposition (SVD).

    This is used as the "Cold Start" (Phase 3) for the tracking pipeline to
    extract a clean initial position, velocity, and acceleration before
    handing off to the Kalman filter.
    """

    def __init__(self, min_points=3):
        """
        Initialize the SVD fitter.

        :param min_points: Minimum number of frames required to perform a fit.
                           We need at least 3 points to solve for [p, v, a].
                           For human-wand tracking, a small buffer (e.g., 5-8) is ideal.
        """
        self.min_points = min_points

    def fit_axis(self, times_sec, positions):
        """
        Solves for [Position_0, Velocity_0, Acceleration] for a single axis (X or Y)
        using Singular Value Decomposition (A*c = b).

        :param times_sec: A 1D numpy array of timestamps (e.g., [0.0, 0.03, 0.06...])
        :param positions: A 1D numpy array of pixel coordinates for those times.
        :return: Tuple of (initial_position, initial_velocity, acceleration)
        """
        if len(times_sec) < self.min_points:
            raise ValueError(
                f"Need at least {self.min_points} points to perform SVD fit, got {len(times_sec)}."
            )

        n = len(times_sec)

        # 1. Build the Design Matrix 'A'
        # Kinematic Equation: p(t) = p_0*(1) + v_0*(t) + a*(0.5 * t^2)
        A = np.zeros((n, 3))
        A[:, 0] = 1.0  # Coefficient for p_0
        A[:, 1] = times_sec  # Coefficient for v_0
        A[:, 2] = 0.5 * times_sec**2  # Coefficient for a

        # 2. Perform Singular Value Decomposition: A = U * S * V^T
        # 'full_matrices=False' gives us the "economy" SVD, which is faster and
        # mathematically sufficient for overdetermined systems.
        U, S, Vt = np.linalg.svd(A, full_matrices=False)

        # 3. Calculate the pseudo-inverse of the Singular Values matrix (S)
        # S is returned as a 1D array of singular values. We invert them.
        # To avoid division by zero (ill-conditioned matrices), we add a tiny threshold.
        threshold = 1e-10
        S_inv = np.array([1.0 / s if s > threshold else 0.0 for s in S])
        S_inv_matrix = np.diag(S_inv)

        # 4. Solve for our coefficients: c = V * S_inv * U^T * b
        # (Where b is our observed positions)
        c = Vt.T @ S_inv_matrix @ U.T @ positions

        p0 = c[0]  # The "true" starting position, smoothed out (pixels)
        v0 = c[1]  # Initial velocity (pixels/sec)
        a = c[2]  # Constant acceleration over the buffer (pixels/sec^2)

        return p0, v0, a

    def fit_2d_trajectory(self, times_sec, x_positions, y_positions):
        """
        Takes the buffer of X and Y data and returns the full 2D state
        needed to initialize the Kalman Filter.

        :param times_sec: List or array of timestamps.
        :param x_positions: List or array of X pixel coordinates.
        :param y_positions: List or array of Y pixel coordinates.
        :return: Dictionary containing the initial state vector components.
        """
        # Convert inputs to numpy arrays
        t_arr = np.array(times_sec, dtype=float)
        x_arr = np.array(x_positions, dtype=float)
        y_arr = np.array(y_positions, dtype=float)

        # Normalize time so the first frame in the buffer is t=0
        t_norm = t_arr - t_arr[0]

        # Fit X axis
        x0, vx, ax = self.fit_axis(t_norm, x_arr)

        # Fit Y axis
        y0, vy, ay = self.fit_axis(t_norm, y_arr)

        return {"x": x0, "y": y0, "vx": vx, "vy": vy, "ax": ax, "ay": ay}
