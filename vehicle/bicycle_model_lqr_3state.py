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

    def update(self, a, delta, dt):          # delta directly, not delta_dot
        self.delta = np.clip(delta, -np.radians(50), np.radians(50))
        self.v = max(self.v + a * dt, 0.0)
        self.x += self.v * np.cos(self.psi) * dt                  #x_dot=v*cos(psi)
        self.y += self.v * np.sin(self.psi) * dt                  #y_dot=v*sin(psi)
        self.psi += (self.v / self.L) * np.tan(self.delta) * dt   #psi_dot=(v/L) tan (delta)

    def get_state(self):
        return self.x, self.y, self.psi,self.v,self.delta