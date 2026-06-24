import numpy as np


class PurePursuit:
    def __init__(self, wheelbase, lookahead_dist):
        self.L = wheelbase
        self.Ld = lookahead_dist
        


    def control(self, x, y, psi, path_x, path_y):
        distances = np.sqrt((path_x - x)**2 +(path_y - y)**2)
        closest_idx = np.argmin(distances)
        # Default: last point in path
        target_x = path_x[-1]
        target_y = path_y[-1]

        n = len(path_x)
        if n > 1:
            segment_distances = np.hypot(np.diff(path_x), np.diff(path_y))
            closed_path = np.hypot(path_x[-1] - path_x[0], path_y[-1] - path_y[0]) < np.mean(segment_distances) * 1.5
        else:
            closed_path = False

        if closed_path:
            for i in range(closest_idx, closest_idx + n):
                j = i % n
                dist = np.sqrt((path_x[j] - x)**2 +(path_y[j] - y)**2)

                if dist >= self.Ld:
                    target_x = path_x[j]
                    target_y = path_y[j]
                    break
            else:
                # Fallback: use closest point if no lookahead point found
                target_x = path_x[closest_idx]
                target_y = path_y[closest_idx]
        else:
            for i in range(closest_idx, n):
                dist = np.sqrt((path_x[i] - x)**2 +(path_y[i] - y)**2)

                if dist >= self.Ld:
                    target_x = path_x[i]
                    target_y = path_y[i]
                    break
            else:
                # Fallback: use closest point if no lookahead point found
                target_x = path_x[closest_idx]
                target_y = path_y[closest_idx]

        
        target_heading = np.arctan2(target_y - y,target_x - x)

        alpha = target_heading - psi

        # Pure Pursuit steering law
        delta = np.arctan((2 * self.L * np.sin(alpha))/ self.Ld)

        return delta,target_x,target_y