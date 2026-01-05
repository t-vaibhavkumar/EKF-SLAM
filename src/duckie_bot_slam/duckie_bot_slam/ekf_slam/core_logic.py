import numpy as np

def angle_normalizer(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

class EKFSLAM:
    def __init__(self, Q, R):
        self.x = None #state vector 
        self.P = None #covarience
        self.Q = Q # motion noise covarience
        self.R = R # measurement noise covarience
        self.landmark_ids = {}

    def initialize(self, x0, p0): #no landmarks yet thus covarience is high
        self.x = np.array(x0, dtype=float) # initiall robot pose
        self.P = p0 # initial covarience
    
    def predict(self, v, w, dt): #prediction step
        x, y, theta = self.x[0:3]

        #motion model
        x_new  = x + v * dt * np.cos(theta)
        y_new  = y + v * dt * np.sin(theta)
        theta_new  = angle_normalizer(theta+w*dt)

        self.x[0:3] = x_new, y_new, theta_new

        #Motion Jacobian
        n = len(self.x)
        F = np.eye(n)

        F[0, 2] = -v * dt * np.sin(theta)
        F[1, 2] =  v * dt * np.cos(theta)

        #Covariance prediction

        Q_full = np.zeros((n,n))
        Q_full [0:3,0:3] = self.Q #noise matrix

        self.P = F @ self.P @ F.T + Q_full

    def initialize_landmark(self, landmark_id, r, phi): #when a landmark is read
        x_r, y_r, theta = self.x[0:3]

        lx = x_r + r * np.cos(theta + phi)
        ly = y_r + r * np.sin(theta + phi)

        idx = len(self.x)
        self.landmark_ids[landmark_id] = idx

        self.x = np.hstack([self.x, lx, ly]) #expanding the state vector
        
        #expanding the covarience 
        n = len(self.x)
        P_new = np.zeros((n, n))
        P_new[:n-2, :n-2] = self.P

        P_new[n-2:n, n-2:n] = np.eye(2) * 1.0  # large uncertainty

        self.P = P_new

    def update(self, landmark_id, r, phi):
        """
        EKF-SLAM Update Step with Nearest-Neighbor Data Association.
        """
        # --- STEP 1: PRE-ASSOCIATION ---
        # Compute the predicted global position for this measurement using the robot's current pose
        x_r, y_r, theta = self.x[0:3]
        lx_meas = x_r + r * np.cos(theta + phi)
        ly_meas = y_r + r * np.sin(theta + phi)

        best_match_id = None
        min_dist = 1.5  # Threshold: If further than 1.5m, consider it a new landmark

        # --- STEP 2: NEAREST-NEIGHBOR DATA ASSOCIATION ---
        # Search through all existing landmarks in the EKF state vector
        for existing_id, idx in self.landmark_ids.items():
            lx_est = self.x[idx]
            ly_est = self.x[idx + 1]
            
            # Calculate Euclidean distance between the measurement and the estimated landmark
            dist = np.sqrt((lx_meas - lx_est)**2 + (ly_meas - ly_est)**2)
            
            if dist < min_dist:
                min_dist = dist
                best_match_id = existing_id

        # --- STEP 3: DECISION LOGIC ---
        if best_match_id is None:
            # NO MATCH FOUND: This is a brand new landmark. 
            # Expand the state vector and map.
            self.initialize_landmark(landmark_id, r, phi)
            return
        else:
            # MATCH FOUND: We re-observed an existing landmark.
            # Use its index in the state vector for the EKF update.
            idx = self.landmark_ids[best_match_id]

        # --- STEP 4: STANDARD EKF UPDATE ---
        lx, ly = self.x[idx:idx+2]
        dx = lx - x_r
        dy = ly - y_r
        q = dx**2 + dy**2
        
        # Predicted range and bearing from current robot estimate
        r_hat = np.sqrt(q)
        phi_hat = angle_normalizer(np.arctan2(dy, dx) - theta)

        z_hat = np.array([r_hat, phi_hat])
        z = np.array([r, phi])

        # Innovation (Residual): difference between real and predicted measurement
        y = z - z_hat
        y[1] = angle_normalizer(y[1])

        # --- STEP 5: MEASUREMENT JACOBIAN (H) ---
        # H represents how the measurement changes relative to the robot and landmark states.
        n = len(self.x)
        H = np.zeros((2, n))
        
        # Derivatives w.r.t robot pose (x, y, theta)
        H[0, 0] = -dx / r_hat
        H[0, 1] = -dy / r_hat
        H[1, 0] =  dy / q
        H[1, 1] = -dx / q
        H[1, 2] = -1

        # Derivatives w.r.t landmark position (lx, ly)
        H[0, idx]     =  dx / r_hat
        H[0, idx + 1] =  dy / r_hat
        H[1, idx]     = -dy / q
        H[1, idx + 1] =  dx / q

        # --- STEP 6: KALMAN GAIN (K) ---
        # S is the innovation covariance
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)

        # --- STEP 7: FINAL CORRECTION ---
        # Correct the state vector (robot + all landmarks)
        self.x = self.x + K @ y
        self.x[2] = angle_normalizer(self.x[2])

        # Update state covariance matrix
        self.P = (np.eye(n) - K @ H) @ self.P



        