import cmsisdsp as dsp
import os

os.environ.setdefault("MPLCONFIGDIR", ".matplotlib")

import matplotlib.pyplot as plt
import numpy as np
from scipy import signal

# Buck converter parameters
R_L     = np.float32(10e-3)     # Inductor resistance   [Ohm]
L       = np.float32(5e-4)      # Inductance            [H]
C       = np.float32(5.8e-4)    # Capacitance           [F]
f_sw    = np.float32(10e3)      # Switching frequency   [Hz]
V_in    = np.float32(13)        # Input voltage         [V]
V_out   = np.float32(5)         # Output voltage        [V]
R       = np.float32(3.5)       # Load resistance       [Ohm]
mu      = np.float32(8e3)       # Observer gain         [-]

# State-space representation of the buck converter
A = np.array([
    [-R_L / L, -1 / L],
    [1 / C, 0]
    ], dtype=np.float32)
B = np.array([
    [1 / L, 0],
    [0, -1 / C]
    ], dtype=np.float32)
C = np.array([
    [1, 1]
    ], dtype=np.float32)
D = np.array([[0]], dtype=np.float32)

x = np.array([
    [0],
    [0]
    ], dtype=np.float32)        # State
u = np.array([
    [V_in],
    [V_out / R]
    ], dtype=np.float32)        # Input
y = np.array([
    [0],
    [0]
    ], dtype=np.float32)        # Output

x_next = np.array([
    [0],
    [0]
    ], dtype=np.float32)  # Next state

print("Continuous-time state-space matrices:")
print("A:\n", A)
print("B:\n", B)
print("C:\n", C)
print("D:\n", D)

# Discretize the system using zero-order hold
T_s = np.float32(1 / f_sw / 1e2)
switch_period = np.float32(1 / f_sw)
switch_half_period = np.float32(switch_period / 2)

A_d, B_d, C_d, D_d, _ = signal.cont2discrete((A, B, C, D), T_s, method='euler')
A_d = A_d.astype(np.float32)
B_d = B_d.astype(np.float32)
C_d = C_d.astype(np.float32)
D_d = D_d.astype(np.float32)

B_on = B.copy()
B_on[0, 0] = 1 / L
B_off = B.copy()
B_off[0, 0] = 0

_, B_d_on, _, _, _ = signal.cont2discrete((A, B_on, C, D), T_s, method='euler')
_, B_d_off, _, _, _ = signal.cont2discrete((A, B_off, C, D), T_s, method='euler')
B_d_on = B_d_on.astype(np.float32)
B_d_off = B_d_off.astype(np.float32)

print("Discrete-time state-space matrices:")
print("A_d:\n", A_d)
print("B_d:\n", B_d)
print("B_d_on:\n", B_d_on)
print("B_d_off:\n", B_d_off)
print("C_d:\n", C_d)
print("D_d:\n", D_d)

def check_status(status, operation):
    if status != 0:
        raise RuntimeError(f"{operation} failed with status {status}")


t_final = 200e-3
n_steps = int(t_final / T_s)
t = np.arange(n_steps + 1, dtype=np.float32) * T_s
x_history = np.zeros((n_steps + 1, x.shape[0]), dtype=np.float32)
y_history = np.zeros((n_steps + 1, C_d.shape[0]), dtype=np.float32)
switch_history = np.zeros(n_steps + 1, dtype=np.float32)

x_history[0] = x[:, 0]

for k in range(n_steps):
    time_in_switch_period = np.float32(t[k] % switch_period)
    switch_state = np.float32(time_in_switch_period < switch_half_period)
    B_d_k = B_d_on if switch_state else B_d_off
    switch_history[k] = switch_state

    status, A_d_MULT_x = dsp.arm_mat_mult_f32(A_d, x)
    check_status(status, "arm_mat_mult_f32(A_d, x)")

    status, B_d_MULT_u = dsp.arm_mat_mult_f32(B_d_k, u)
    check_status(status, "arm_mat_mult_f32(B_d_k, u)")

    status, x_next = dsp.arm_mat_add_f32(A_d_MULT_x, B_d_MULT_u)
    check_status(status, "arm_mat_add_f32(A_d_MULT_x, B_d_MULT_u)")

    x = x_next
    x_history[k + 1] = x[:, 0]

    status, y = dsp.arm_mat_mult_f32(C_d, x)
    check_status(status, "arm_mat_mult_f32(C_d, x)")
    y_history[k + 1] = y[:, 0]

switch_history[-1] = switch_history[-2]

print(f"Simulation step T_s: {T_s} s")
print(f"Simulation time: {t_final} s")
print("Final x:\n", x)
print("Final y:\n", y)

fig, (ax_x, ax_y, ax_switch) = plt.subplots(3, 1, sharex=True, figsize=(10, 8))

for state_index in range(x_history.shape[1]):
    ax_x.plot(t, x_history[:, state_index], label=f"x{state_index + 1}")
ax_x.set_ylabel("State")
ax_x.grid(True)
ax_x.legend()

for output_index in range(y_history.shape[1]):
    ax_y.plot(t, y_history[:, output_index], label=f"y{output_index + 1}")
ax_y.set_xlabel("Time [s]")
ax_y.set_ylabel("Output")
ax_y.grid(True)
ax_y.legend()

ax_switch.step(t, switch_history, where="post", label="B[0,0] switch")
ax_switch.set_xlabel("Time [s]")
ax_switch.set_ylabel("Switch")
ax_switch.set_yticks([0, 1])
ax_switch.grid(True)
ax_switch.legend()

plt.tight_layout()
plt.show()
