import numpy as np
class Stanley:
    def __init__(self,k,k_psi):
        self.k=k
        self.k_psi = k_psi
    
    def control(self, x, y, psi, path_x, path_y,v,L):

        distances=np.hypot((x-path_x),(y-path_y))
        closest_idx=np.argmin(distances)
        crosstrack_error=distances[closest_idx]
    

        if closest_idx < len(path_x) - 1:
            dx = path_x[closest_idx + 1] - path_x[closest_idx]
            dy = path_y[closest_idx + 1] - path_y[closest_idx]
        else:
            dx = path_x[closest_idx] - path_x[closest_idx - 1] 
            dy = path_y[closest_idx] - path_y[closest_idx - 1]      
        

        x_front = x + L*np.cos(psi)
        y_front = y + L*np.sin(psi)

        dx_car = x_front - path_x[closest_idx]
        dy_car = y_front - path_y[closest_idx]

        cross=np.cross([dx,dy],[dx_car,dy_car])  #To get the direction of vehicle wrt the path
        direction=-1 if cross>0 else 1

        path_heading = np.arctan2(dy, dx)

        heading_error = path_heading - psi
        heading_error = np.arctan2(np.sin(heading_error),np.cos(heading_error)) #restraining angles betweeen pi and -pi

        delta = (
        self.k_psi*heading_error+
        np.arctan(
            (self.k*direction*crosstrack_error)/(v + 1e-6)))

        debug = {
            "front_x": x_front,
            "front_y": y_front,
            "nearest_x": path_x[closest_idx],
            "nearest_y": path_y[closest_idx],
            "path_heading": path_heading,
            "cte": crosstrack_error,
            "heading_error": heading_error
        }

        return delta, debug