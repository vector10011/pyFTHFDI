import cmsisdsp as dsp
import numpy as np
from scipy import signal

# Buck converter parameters
R_L     = 10e-3     # Inductor resistance   [Ohm]
L       = 5e-4      # Inductance            [H]
C       = 5.8e-4    # Capacitance           [F]
f_sw    = 10e3      # Switching frequency   [Hz]
V_in    = 13        # Input voltage         [V]
V_out   = 5         # Output voltage        [V]
R       = 3.5       # Load resistance       [Ohm]
mu      = 8e3       # Observer gain         [-]

# State-space representation of the buck converter
A = np.array([[-R_L/L, -1/L], [1/C, 0]])
B = np.array([[1/L, 0], [0, -1/C]])
C = np.array([[1, 1]])
D = np.array([[0]])

x = np.array([[0], [0]])        # State
u = np.array([[0], [0]])        # Input
y = np.array([[0], [0]])        # Output

x_next = np.array([[0], [0]])  # Next state

print("Continuous-time state-space matrices:")
print("A:\n", A)
print("B:\n", B)
print("C:\n", C)
print("D:\n", D)

# Discretize the system using zero-order hold
T_s = 1 / f_sw / 1e2

A_d, B_d, C_d, D_d, _ = signal.cont2discrete((A, B, C, D), T_s, method='zoh')

print("Discrete-time state-space matrices:")
print("A_d:\n", A_d)
print("B_d:\n", B_d)
print("C_d:\n", C_d)
print("D_d:\n", D_d)


