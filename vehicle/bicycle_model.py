import numpy as np

class BicycleModel:
    def __init__(self, L, x=0, y=0, psi=0,v=0):
        if L <= 0:
            raise ValueError("Wheelbase must be positive")
        self.L = L
        self.x = x
        self.y = y
        self.psi = psi
        self.v=v

    def update(self, a, delta, dt):

        self.x += self.v * np.cos(self.psi) * dt             #x_dot=v*cos(psi)
        self.y += self.v * np.sin(self.psi) * dt             #y_dot=v*sin(psi)
        delta = np.clip(delta, -np.radians(35), np.radians(35))
        self.psi += (self.v / self.L) * np.tan(delta) * dt
        self.v+=a*dt   #psi_dot=(v/L) tan (delta)

    def get_state(self):

        return self.x, self.y, self.psi