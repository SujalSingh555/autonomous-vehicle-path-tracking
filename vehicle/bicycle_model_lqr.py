import numpy as np

class BicycleModelLQR:
    def __init__(self, L, x=0, y=0, psi=0,v=1,delta=0):
        if L <= 0:
            raise ValueError("Wheelbase must be positive")
        self.L = L
        self.x = x
        self.y = y
        self.psi = psi
        self.v=v
        self.delta=delta

    def update(self, a, delta_dot, dt):
        self.delta+=delta_dot*dt
        self.delta = np.clip(self.delta, -np.radians(35), np.radians(35))
         # Steering limit
        #self.delta = np.clip(self.delta,-np.radians(35),np.radians(35))
        self.v = max(self.v + a * dt, 0.0)
        # Prevent negative velocity
        self.x += self.v * np.cos(self.psi) * dt                  #x_dot=v*cos(psi)
        self.y += self.v * np.sin(self.psi) * dt                  #y_dot=v*sin(psi)
        self.psi += (self.v / self.L) * np.tan(self.delta) * dt   #psi_dot=(v/L) tan (delta)
          

    def get_state(self):
        return self.x, self.y, self.psi,self.v,self.delta