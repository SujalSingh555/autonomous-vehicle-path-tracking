import numpy as np
from scipy.linalg import solve_continuous_are


class LQR:
    def __init__(self, wheelbase, Q, R,max_speed):
        self.L = wheelbase
        self.Q=Q
        self.R=R
        self.max_speed=max_speed
        self.K = None
        self.prev_v = None
        self.i=1


    def control(self, x, y,v, psi, path_x, path_y,dt):

        error=np.hypot((x-path_x),(y-path_y))
        closest_idx=np.argmin(error)


        if closest_idx < len(path_x) - 1:
            path_heading = np.arctan2(
                path_y[closest_idx+1] - path_y[closest_idx],
                path_x[closest_idx+1] - path_x[closest_idx]
            )
        else:
            path_heading = np.arctan2(
                path_y[closest_idx] - path_y[closest_idx-1],
                path_x[closest_idx] - path_x[closest_idx-1]
            )

        if 0 < closest_idx < len(path_x)-1:

            x1,y1 = path_x[closest_idx-1], path_y[closest_idx-1]
            x2,y2 = path_x[closest_idx],   path_y[closest_idx]
            x3,y3 = path_x[closest_idx+1], path_y[closest_idx+1]

            heading1 = np.arctan2(y2-y1, x2-x1)
            heading2 = np.arctan2(y3-y2, x3-x2)

            dpsi = heading2 - heading1
            dpsi = (dpsi + np.pi) % (2*np.pi) - np.pi

            ds = np.hypot(x3-x2, y3-y2)

            curvature = abs(dpsi)/(ds + 1e-6)

        else:
            curvature = 0.0
        
        gain=10
        target_speed=np.clip(self.max_speed/(1+gain*curvature),1,self.max_speed)

        v_safe = max(abs(v), 0.5)

        A=np.array(
        [[0,v_safe,0],
        [0,0,     0],
        [0,0,     0]
        ])

        B=np.array(
        [[0,           0],
        [(v_safe/self.L),0],
        [0,            1]
        ]
        )

        dx = path_x[closest_idx] - x
        dy = path_y[closest_idx] - y
        tangent_x = np.cos(path_heading)
        tangent_y = np.sin(path_heading)
        cross =-(tangent_x * dy - tangent_y * dx)
        e = np.hypot(dx, dy) * np.sign(cross)

        psi_e = (-path_heading + psi)
        psi_e = (psi_e + np.pi) % (2 * np.pi) - np.pi

        v_error = v-target_speed 

        state = np.array([e,psi_e,v_error])

        if self.K is None or abs(v - self.prev_v) > 0.5:
                try:
                    P = solve_continuous_are(A, B, self.Q, self.R)
                    self.K = np.linalg.inv(self.R) @ B.T @ P
                    self.prev_v = v
                except np.linalg.LinAlgError:
                    if self.K is None:
                        return 0.0, 0.0
        
        u = -self.K @ state
        self.i+=1
        #if self.i%10==0:
            #print("state:",state," u:",u)
        
        delta = float(u[0])
        a = float(u[1])

        return delta,a