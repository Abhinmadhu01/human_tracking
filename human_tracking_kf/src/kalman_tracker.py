"""
kalman_tracker.py
-----------------
Self-contained 2-D Kalman Filter for pixel-space target tracking.

State vector  : [x, y, vx, vy]  (position + velocity in image pixels)
Measurement   : [x, y]           (bounding-box centre from YOLO)

Improvements over the original Kalman_Filter_Custom_2D.py
----------------------------------------------------------
1. Joseph stabilised covariance update  – prevents P going non-positive-definite
   during long runs (floating-point error accumulates in the naive form).
2. Measurement shape guard              – accepts both (2,) and (2,1) arrays.
3. Innovation gating                    – rejects wild YOLO detections that are
   statistically inconsistent with the current estimate (Mahalanobis distance).
4. Velocity accessor                    – expose vx, vy so the drone node can
   feed predicted pixel velocity into feed-forward control if needed.
5. All magic numbers removed            – constructor parameters only.
6. No matplotlib / os imports           – zero unused dependencies in production.
"""

import numpy as np


class KalmanTracker:
    """
    Linear Kalman Filter for 2-D pixel-space tracking.

    WHY linear and not EKF/UKF?
    The camera-image plane is a flat 2-D space with no coordinate transforms.
    The motion is well approximated by constant velocity for the durations a
    drone needs to predict (< 1 s occlusion).  A linear KF is optimal for this
    model and is significantly cheaper to run than EKF/UKF on an embedded board.

    Parameters
    ----------
    dt          : float  – control loop period in seconds (e.g. 1/20 = 0.05 s)
    init_pos_std: float  – initial position uncertainty in pixels (e.g. 10.0)
    init_vel_std: float  – initial velocity uncertainty in px/s  (e.g. 50.0)
    accel_std   : float  – process noise (acceleration std) in px/s²
                           LOW  (~1)   → trust the motion model more (smooth but lags turns)
                           HIGH (~40)  → trust measurements more   (responsive but noisy)
                           Typical outdoor drone value: 15–25
    meas_std    : float  – measurement noise in pixels (YOLO box jitter, ~3–8 px)
    gate_sigma  : float  – Mahalanobis gate (reject if chi² > gate_sigma²).
                           Set to 0 to disable gating.
    """

    def __init__(
        self,
        dt: float         = 0.05,
        init_pos_std: float = 10.0,
        init_vel_std: float = 50.0,
        accel_std: float   = 20.0,
        meas_std: float    = 5.0,
        gate_sigma: float  = 4.0,
    ) -> None:

        self.dt         = dt
        self.gate_chi2  = gate_sigma ** 2   # pre-square once

        # ── State vector [x, y, vx, vy] ─────────────────────────────────────
        self.x = np.zeros((4, 1), dtype=np.float64)

        # ── State covariance P ───────────────────────────────────────────────
        # WHY diagonal initialisation?
        # At startup we know nothing about velocity – high variance.
        # We assume position is roughly at image centre – lower variance.
        self.P = np.diag([
            init_pos_std ** 2,
            init_pos_std ** 2,
            init_vel_std ** 2,
            init_vel_std ** 2,
        ]).astype(np.float64)

        # ── State transition matrix F ────────────────────────────────────────
        # Constant-velocity model:  x(t+dt) = x(t) + vx(t)*dt
        self.F = np.array([
            [1.0, 0.0, dt,  0.0],
            [0.0, 1.0, 0.0, dt ],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ], dtype=np.float64)

        # ── Process noise covariance Q ───────────────────────────────────────
        # Discrete white-noise acceleration model.
        # L maps 2-D acceleration noise into the 4-D state space.
        L = np.array([
            [0.5 * dt ** 2, 0.0          ],
            [0.0,           0.5 * dt ** 2],
            [dt,            0.0          ],
            [0.0,           dt           ],
        ], dtype=np.float64)

        q = np.diag([accel_std ** 2, accel_std ** 2]).astype(np.float64)
        self.Q = L @ q @ L.T

        # ── Measurement matrix H ─────────────────────────────────────────────
        # We only observe position, not velocity.
        self.H = np.array([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
        ], dtype=np.float64)

        # ── Measurement noise covariance R ───────────────────────────────────
        # Diagonal: YOLO x-error and y-error are uncorrelated.
        self.R = np.diag([meas_std ** 2, meas_std ** 2]).astype(np.float64)

        # ── Book-keeping ─────────────────────────────────────────────────────
        self._initialised = False   # first measurement seeds x directly

    # ── Public API ────────────────────────────────────────────────────────────

    def predict(self) -> tuple[float, float]:
        """
        Propagate state forward by one time step using the motion model.

        Returns
        -------
        (x_pred, y_pred) : predicted pixel position (float, float)

        WHY call predict every loop tick, even when YOLO has a detection?
        The predict step must always run first to advance the state covariance.
        Skipping it then calling update gives a stale P, producing a wrong
        Kalman gain K.  Correct pattern: predict() → update() every cycle.
        """
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        return float(self.x[0, 0]), float(self.x[1, 0])

    def update(self, z) -> tuple[float, float] | None:
        """
        Fuse a YOLO bounding-box centre measurement into the state estimate.

        Parameters
        ----------
        z : array-like of shape (2,) or (2,1)  – measured [x_pixel, y_pixel]

        Returns
        -------
        (x_updt, y_updt) : updated position estimate, or None if gated out.

        WHY return None instead of raising?
        A gated measurement should be treated as a missed detection – the
        calling node simply skips the update step and continues with the
        predicted state.  Raising would crash the node.
        """
        z = np.asarray(z, dtype=np.float64).reshape(2, 1)

        # Seed state on first detection so we don't track from (0,0)
        if not self._initialised:
            self.x[0, 0] = z[0, 0]
            self.x[1, 0] = z[1, 0]
            self._initialised = True
            return float(self.x[0, 0]), float(self.x[1, 0])

        # Innovation  y = z - H*x_pred
        y_innov = z - self.H @ self.x

        # Innovation covariance  S = H*P*Hᵀ + R
        S = self.H @ self.P @ self.H.T + self.R

        # ── Mahalanobis gate ─────────────────────────────────────────────────
        # WHY gate?  YOLO occasionally produces wildly wrong detections
        # (e.g. a bird mis-classified as a person).  If the measurement is
        # statistically far from our prediction (large Mahalanobis distance)
        # we reject it.  This prevents the filter from "jumping" to a false
        # detection.  chi² = yᵀ * S⁻¹ * y
        if self.gate_chi2 > 0.0:
            S_inv = np.linalg.inv(S)
            chi2  = float(y_innov.T @ S_inv @ y_innov)
            if chi2 > self.gate_chi2:
                return None   # reject this measurement; keep predicted state

        # Kalman gain  K = P*Hᵀ * S⁻¹
        K = self.P @ self.H.T @ np.linalg.inv(S)

        # State update
        self.x = self.x + K @ y_innov

        # ── Joseph stabilised covariance update ──────────────────────────────
        # WHY Joseph form instead of  P = (I - K*H)*P  ?
        # The naive form is algebraically equivalent but NOT numerically so.
        # Floating-point rounding makes P gradually lose symmetry and positive-
        # definiteness, causing the Kalman gain to diverge after ~minutes.
        # The Joseph form  P = (I-KH)*P*(I-KH)ᵀ + K*R*Kᵀ  is always symmetric
        # positive semi-definite regardless of numerical errors.
        I   = np.eye(4, dtype=np.float64)
        IKH = I - K @ self.H
        self.P = IKH @ self.P @ IKH.T + K @ self.R @ K.T

        return float(self.x[0, 0]), float(self.x[1, 0])

    def reset(self, x: float = 0.0, y: float = 0.0) -> None:
        """
        Reset filter state – call when the drone re-acquires a lost target
        at a very different position (e.g. after a long occlusion).
        """
        self.x          = np.zeros((4, 1), dtype=np.float64)
        self.x[0, 0]    = x
        self.x[1, 0]    = y
        self._initialised = False

    # ── Properties ────────────────────────────────────────────────────────────

    @property
    def position(self) -> tuple[float, float]:
        """Current estimated pixel position (x, y)."""
        return float(self.x[0, 0]), float(self.x[1, 0])

    @property
    def velocity(self) -> tuple[float, float]:
        """
        Current estimated pixel velocity (vx, vy) in pixels/second.
        Useful for feed-forward: if the person is moving right at 20 px/s,
        pre-emptively strafe right rather than waiting for position error.
        """
        return float(self.x[2, 0]), float(self.x[3, 0])

    @property
    def uncertainty(self) -> tuple[float, float]:
        """1-sigma position uncertainty in pixels (sqrt of P diagonal)."""
        return float(np.sqrt(self.P[0, 0])), float(np.sqrt(self.P[1, 1]))
