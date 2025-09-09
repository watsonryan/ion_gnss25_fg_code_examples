import numpy as np
from examples.helpers.constants import mu_earth, R_earth, C_D, A, mass, p_srp, AU, Cr, rho_0

# RKF78 coefficients
A_rkf = np.array([0, 2/27, 1/9, 1/6, 5/12, 1/2, 5/6, 1/6, 2/3, 1/3, 1, 0, 1])
B_rkf = [
    [2/27],
    [1/36, 1/12],
    [1/24, 0, 1/8],
    [5/12, 0, -25/16, 25/16],
    [1/20, 0, 0, 1/4, 1/5],
    [-25/108, 0, 0, 125/108, -65/27, 125/54],
    [31/300, 0, 0, 0, 61/225, -2/9, 13/900],
    [2, 0, 0, -53/6, 704/45, -107/9, 67/90, 3],
    [-91/108, 0, 0, 23/108, -976/135, 311/54, -19/60, 17/6, -1/12],
    [2383/4100, 0, 0, -341/164, 4496/1025, -301/82, 2133/4100, 45/82, 45/164, 18/41],
    [3/205, 0, 0, 0, 0, -6/41, -3/205, -3/41, 3/41, 6/41],
    [-1777/4100, 0, 0, -341/164, 4496/1025, -289/82, 2193/4100, 51/82, 33/164, 12/41, 0]
]

C7 = np.array([41/840, 0, 0, 0, 34/105, 9/35, 9/35, 9/280, 9/280, 41/840])
C8 = np.array([0, 0, 0, 0, 34/105, 9/35, 9/35, 9/280, 9/280, 41/840, 1/80])

def rkf78_step(f, t, y, h):
    """Perform one step of the RKF78 method."""
    k = np.zeros((13, len(y)))  # Store k_i values
    
    # Calculate k_i values
    k[0] = h * f(t, y)
    for i in range(1, 13):
        y_temp = y + np.sum(np.array(B_rkf[i-1])[:, np.newaxis] * k[:i], axis=0)
        k[i] = h * f(t + A_rkf[i] * h, y_temp)
    
    # 7th and 8th order solutions
    y_7th = y + np.dot(C7, k[:10])
    y_8th = y + np.dot(C8, k)
    
    # Estimate error and adaptive step size
    error = np.linalg.norm(y_8th - y_7th, ord=np.inf)
    tol = 1e-6  # Desired tolerance
    if error == 0:
        s = 2  # Prevent division by zero
    else:
        s = 0.9 * (tol / error)**(1/8)
    h_next = s * h
    
    return y_8th, t + h, h_next, error

def gravity(r):
    """Compute gravitational acceleration."""
    r_mag = np.linalg.norm(r)
    return -mu_earth * r / r_mag**3

def atmospheric_density(altitude):
    """Exponential model for atmospheric density."""
    H = 8500  # Scale height in meters
    rho = rho_0 * np.exp(-altitude / H)
    return rho

def drag(r, v):
    """Compute drag force."""
    altitude = np.linalg.norm(r) - R_earth
    rho = atmospheric_density(altitude)
    v_mag = np.linalg.norm(v)
    drag_acc = -0.5 * rho * C_D * A / mass * v_mag * v
    return drag_acc

def srp(r, r_sun):
    """Compute solar radiation pressure force."""
    r_sun_mag = np.linalg.norm(r_sun)
    r_rel = r - r_sun
    r_rel_mag = np.linalg.norm(r_rel)
    F_srp = p_srp * Cr * A / mass * (AU / r_rel_mag)**2 * r_rel / r_rel_mag
    return F_srp

def acceleration(t, state, r_sun):
    """Compute total acceleration due to gravity, drag, and SRP."""
    r = state[:3]
    v = state[3:]
    a_gravity = gravity(r)
    a_drag = drag(r, v)
    a_srp = srp(r, r_sun)
    return np.concatenate((v, a_gravity + a_drag + a_srp))

def propagate_orbit(r0, v0, t_span, dt, r_sun):
    """Propagate the orbit using RKF78."""
    t = 0
    state = np.concatenate((r0, v0))
    trajectory = [state]
    
    # Lambda function to pass r_sun to the acceleration function
    acceleration_with_sun = lambda t, state: acceleration(t, state, r_sun)
    
    while t < t_span:
        state, t, dt, error = rkf78_step(acceleration_with_sun, t, state, dt)
        trajectory.append(state)
    
    return np.array(trajectory)

# Example usage:
if __name__ == "__main__":
    r0 = np.array([7000e3, 0, 0])  # Initial position (m)
    v0 = np.array([0, 7.5e3, 0])   # Initial velocity (m/s)
    r_sun = np.array([AU, 0, 0])   # Simplified Sun position
    t_span = 24 * 3600  # 1 day in seconds
    dt = 60  # Initial time step in seconds
    
    trajectory = propagate_orbit(r0, v0, t_span, dt, r_sun)
    print(trajectory)
