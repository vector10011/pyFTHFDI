## Abstract

We propose a fault diagnosis method for power electronics systems that extends classical observer-based fault-sensitive detection filters for linear time-invariant systems to switched-linear systems commonly encountered in power electronics. The result is a piecewise-linear detection filter, which in the absence of faults, works the same way as an observer—it predicts the system states exactly. If a fault occurs, the state predicted by the filter differs from the true state of the system, and by appropriately choosing the filter gain, the filter residual has certain geometrical characteristics that makes the fault identifiable. An experimental platform to verify the feasibility of the proposed method is presented along with simulation and experimental results illustrating the feasibility and effectiveness of the method.

# I. Introduction

Fault-tolerance may be defined as the ability of a system to adapt and compensate in a planned, systematic way to random component faults and keep delivering completely or partially the functionality for which it was designed. Fault-tolerant electric power supply is paramount in many applications, ranging from safety- and mission-critical systems for aircraft, spacecraft and automobiles, to communication systems, computers supporting financial markets, and industrial control equipment. In all these applications, power electronics systems are the essential building blocks of the electric power supply.

In a power electronics system, key elements to achieving fault tolerance are: component redundancy, a fault diagnosis system, and a reconfiguration system that, upon information provided by the diagnosis system, removes faulty components and usually substitutes them with redundant ones. A fault diagnosis system executes three tasks: i) detection makes a binary decision whether or not a fault has occurred, ii) isolation determines the location of the faulty component, and iii) a severity assessment determines the extent of the fault.

In general, methods for fault diagnosis can be broadly classified into three categories: i) model-based, which includes observer-based detection filters and fault knowledge-based methods that can point to specific faults by measuring certain variables; ii) artificial intelligence, which uses neural networks and fuzzy logic to develop expert systems that, once trained, can point to specific faults; and iii) empirical and signal processing methods, which use spectral analysis to identify specific signatures of a certain fault. In the context of power electronics, there has been work on fault knowledge-based methods. There has also been substantial work on the application of artificial intelligence and signal processing methods.

In this paper, we focus on developing observer-based fault diagnosis methods for power electronics systems. In particular, we extend the method for fault diagnosis of linear time-invariant (LTI) systems developed by Beard and Jones, to switched-linear systems commonly encountered in power electronics. In the Beard-Jones method, a detection filter with the same structure of a Luenberger observer is constructed. In the absence of faults, the filter works exactly the same way as the Luenberger observer, and the state estimates obtained from the filter converge to the actual values asymptotically. If a fault occurs, the state predicted by the detection filter differs from the true state of the system. For particular faults, by appropriately choosing the filter gain, the filter residuals have certain geometrical characteristics that make the fault identifiable.

In power electronics, it is common to use an averaged model of the system to design the control. The use of averaged models together with the Beard-Jones approach might look like a feasible solution for fault diagnosis in power electronics systems. Careful analysis of certain types of faults, e.g., degradation in the output filter capacitor of a buck converter, shows that this is not the case, i.e., a detection filter based on the averaged model of the converter cannot capture this type of fault. To overcome the limitations of averaged models for fault detection filter design, we use the converter switching model and develop fault sensitive detection filters based on piecewise linear observers. We show analytical and simulation results demonstrating the feasibility of this approach. A testbed has been developed to experimentally verify the feasibility and effectiveness of the proposed diagnosis method.

The remainder of this paper is organized as follows. In Section II, we formulate the fault diagnosis problem in power electronics systems, and by using observer-based detection filters, a solution is provided in Section III. In Section IV, we develop detection filters for a fault-tolerant buck converter and show their performance through simulation. Section V illustrates the implementation of the proposed detection filters in an experimental setup. Concluding remarks are discussed in Section VI.

# II. Problem Formulation

To motivate the problem, consider the redundant converter of Fig. 1. Before any fault occurrence, its behavior is described by a linear switched state-space model:

$$
\frac{d}{dt}
\begin{bmatrix}
i_1 \\
i_2 \\
v
\end{bmatrix}
=
\begin{bmatrix}
-\frac{R_{L1}}{L_1} & 0 & -\frac{1}{L_1} \\
0 & -\frac{R_{L2}}{L_2} & -\frac{1}{L_2} \\
\frac{1}{C_1+C_2} & \frac{1}{C_1+C_2} & 0
\end{bmatrix}
\begin{bmatrix}
i_1 \\
i_2 \\
v
\end{bmatrix}
+
\begin{bmatrix}
\frac{\sigma_1(t)}{L_1} & 0 & 0 \\
0 & \frac{\sigma_2(t)}{L_2} & 0 \\
0 & 0 & -\frac{1}{C_1+C_2}
\end{bmatrix}
\begin{bmatrix}
V_1 \\
V_2 \\
i_{load}
\end{bmatrix}.
\tag{1}
$$

where $\sigma_1, \sigma_2 : [0,\infty) \to \{0,1\}$ are the binary switching signals governing $SW_1$ and $SW_2$ respectively. The switching signals governing $SW_3$ and $SW_4$ are given by the binary complement of $\sigma_1$ and $\sigma_2$ respectively. The description in (1) can be completed with $y = Cx$, where $C$ is the identity matrix and $x = [i_1, i_2, v]'$, describing the measurements available to the controller.

Now, suppose a fault has occurred in the system, causing a change in the matrices of any or all subsystems in (1). Without loss of generality, consider a fault in the output filter capacitor $C_1$. This fault may cause the capacitor to degrade slowly over time, which would result in a gradual decrease in capacitance (soft fault), or it may cause a sudden fail open or short (hard fault). Thus, the capacitance of a capacitor in the output filter capacitor bank can be described by $C_1(t) = C_1 + \lambda_C(t)$, where $C_1$ is the nominal capacitance, with $\lambda_C(t) \le 0$ describing the fault magnitude. Then, after the fault occurrence, the converter behavior is described by

$$
\frac{d}{dt}
\begin{bmatrix}
i_1 \\
i_2 \\
v
\end{bmatrix}
=
\begin{bmatrix}
0 \\
0 \\
-\frac{1}{C_1(t)+C_2}\frac{d\lambda_C}{dt}v
\end{bmatrix}
+
\begin{bmatrix}
-\frac{R_{L1}}{L_1} & 0 & -\frac{1}{L_1} \\
0 & -\frac{R_{L2}}{L_2} & -\frac{1}{L_2} \\
\frac{1}{C_1(t)+C_2} & \frac{1}{C_1(t)+C_2} & 0
\end{bmatrix}
\begin{bmatrix}
i_1 \\
i_2 \\
v
\end{bmatrix}
+
\begin{bmatrix}
\frac{\sigma_1(t)}{L_1} & 0 & 0 \\
0 & \frac{\sigma_2(t)}{L_2} & 0 \\
0 & 0 & -\frac{1}{C_1(t)+C_2}
\end{bmatrix}
\begin{bmatrix}
V_1 \\
V_2 \\
i_{load}
\end{bmatrix}.
\tag{2}
$$

Note that the matrices in (2) are time-varying and result from perturbing the last row of the matrices in (1). However, it is possible to rewrite (2) as the original dynamics (1) plus an additional disturbance input which captures the effect of the fault on the original dynamics:

$$
\frac{d}{dt}
\begin{bmatrix}
i_1 \\
i_2 \\
v
\end{bmatrix}
=
\begin{bmatrix}
-\frac{R_{L1}}{L_1} & 0 & -\frac{1}{L_1} \\
0 & -\frac{R_{L2}}{L_2} & -\frac{1}{L_2} \\
\frac{1}{C_1+C_2} & \frac{1}{C_1+C_2} & 0
\end{bmatrix}
\begin{bmatrix}
i_1 \\
i_2 \\
v
\end{bmatrix}
+
\begin{bmatrix}
\frac{\sigma_1(t)}{L_1} & 0 & 0 \\
0 & \frac{\sigma_2(t)}{L_2} & 0 \\
0 & 0 & -\frac{1}{C_1+C_2}
\end{bmatrix}
\begin{bmatrix}
V_1 \\
V_2 \\
i_{load}
\end{bmatrix}
+
\phi_{\sigma(t)} f,
\tag{3}
$$

where

$$
\phi_{\sigma(t)} =
-\frac{1}{C_1(t)+C_2}
\left(
\frac{\lambda_C(t)}{C_1+C_2}(i_1+i_2-i_{load})
+ v \frac{d\lambda_C}{dt}
\right),
\qquad
f = [0,0,1]'.
$$

**Fault diagnosis problem:** Consider (3), the fault diagnosis problem consists of designing a detection filter that takes $u$, $y$, $\sigma_1$, and $\sigma_2$ as inputs and generates a residual vector with the following properties: i) when there is no fault, the residual is identical to zero, and ii) when a fault occurs, the residual asymptotically converges to a vector parallel to $f$.

The generalization of the ideas presented in this section, as well as the solution to the diagnosis problem are discussed next.

> **Fig. 1.** Dual-redundant buck converter architecture.

# III. Solution to the Fault Diagnosis Problem

By generalizing (1), the behavior of a non-faulty power electronics system can be described by

$$
\dot{x} = A_\sigma x + B_\sigma u, \qquad y = Cx,
\tag{4}
$$

where $x \in \mathbb{R}^n$, $u(t) \in \mathbb{R}^m$, $y(t) \in \mathbb{R}^l$, the function $\sigma : [0,\infty) \to \mathcal{P}$, called the switching signal, indicates the active subsystem at every time, $\mathcal{P}$ is called “index set”, and $A_p$, $B_p$, $C$, with $p \in \mathcal{P}$ define the subsystems in (4). Note that $C$ is assumed to be the same for all subsystems.

Now to generalize (3), suppose a fault has occurred in the system described by (4), causing a change in the $A_p$ and $B_p$ matrices of any or all subsystems and/or causing a change in the switches implementing $\sigma$. It is assumed the system measurements are not subject to faults and therefore the observation matrix $C$ remains unchanged. It is assumed that a single fault will only affect the derivative of a single state variable, which is equivalent to assuming that only the elements of a particular row of the matrices $A_p$ and $B_p$ change after the fault. As it turns out, this is not a very restrictive condition in power electronics systems, where, for example, a change in the derivative of a voltage across a capacitor is usually associated with the degradation of a capacitor. In this scenario, it can be shown that the faulty dynamics of a power electronics system can be described by

$$
\dot{x} = A_\sigma x + B_\sigma w + \phi_\sigma(t) f, \qquad y = Cx,
\tag{5}
$$

where $\phi_\sigma(t) \in \mathbb{R}$ captures the change in the system due to the fault, and $f \in \mathbb{R}^n$ is constant.

## A. Detection Filter

In (5), the fault described by $\phi_\sigma(t)f$ can be detected with a filter of the form

$$
\dot{\hat{x}} = A_\sigma \hat{x} + B_\sigma w + L_\sigma(y-\hat{y}), \qquad \hat{y} = C\hat{x},
\tag{6}
$$

where $\sigma : [0,\infty) \to \mathcal{P}$ is known and each $L_p$, with $p \in \mathcal{P}$ is chosen in such a way that the vector $f$ associated with the particular fault will manifest in the filter residual. Let $e = x - \hat{x}$ and $\gamma = y - C\hat{x}$. If we assume full-state measurement, i.e., $C = I$, and choose $L_p = \mu I + A_p$, with $\mu > 0$, the filter residual $\gamma$ is

$$
\dot{e} = -\mu e + \phi_\sigma(t) f, \qquad \gamma = e,
\tag{7}
$$

$$
\gamma(t) = e^{-\mu t}\gamma(0) +
\left[\int_0^t e^{-\mu(t-\tau)}\phi(\tau)d\tau\right]f.
\tag{8}
$$

As $t$ increases, the exponential term in (8) vanishes and the solution $\gamma(t)$ remains parallel to $f$. Also, since $\gamma(t)$ is a function of $\phi_\sigma(t)$, which is a function of the magnitude of the fault, observing $\gamma(t)$ allows us to quantify the magnitude of the fault. It is important to note that while (4) and (5) are switched-linear systems, the error dynamics given in (7) correspond to a LTI system. Thus, by imposing $\mu > 0$, filter stability is ensured.

## Table I. Dual-Redundant Buck Converter Parameters

| Parameter | Value | Parameter | Value |
|---|---:|---|---:|
| $R_{L1}, R_{L2}$ [$\Omega$] | $10^{-3}$ | $V_1, V_2$ [V] | $12$ |
| $L_1, L_2$ [H] | $10^{-3}$ | $V_{out}$ [V] | $5$ |
| $C$ [F] | $10 \cdot 10^{-5}$ | $i_{out}$ [A] | $10$ |
| $f$ [kHz] | $200$ | $\mu$ | $100$ |

# IV. Fault Diagnosis in a Dual-Redundant Buck Converter

Consider again the dual-redundant buck converter of Fig. 1. In this system, the components most vulnerable to faults are the switches and the output capacitor. In this section, we develop a detection filter that allows the diagnosis of faults in these components. We study the effectiveness of the filter by analyzing the asymptotic behavior of the filter residual. We additionally test their performance in a simulation environment using MATLAB/Simulink®, where both the converter dynamics and the FDI filter are simulated, and random faults are injected. The converter model parameters and the detection filter gain used in the simulations are given in Table I.

## A. Detection Filter

Following the notation of Section III, a fault detection filter for the system in Fig. 1 is given by

$$
\frac{d}{dt}
\begin{bmatrix}
\hat{i}_1 \\
\hat{i}_2 \\
\hat{v}
\end{bmatrix}
=
-\mu
\begin{bmatrix}
\hat{i}_1 \\
\hat{i}_2 \\
\hat{v}
\end{bmatrix}
+
\begin{bmatrix}
\frac{\sigma_1(t)}{L_1} & 0 & 0 \\
0 & \frac{\sigma_2(t)}{L_2} & 0 \\
0 & 0 & -\frac{1}{2C}
\end{bmatrix}u
+
\begin{bmatrix}
\mu-\frac{R_{L1}}{L_1} & 0 & -\frac{1}{L_1} \\
0 & \mu-\frac{R_{L2}}{L_2} & -\frac{1}{L_2} \\
\frac{1}{2C} & \frac{1}{2C} & \mu
\end{bmatrix}x,
\tag{9}
$$

where $u = [V_1,V_2,i_{load}]'$, $x = [i_1,i_2,v]'$, and $\mu > 0$.

## B. Capacitor Fault

The effect on the converter performance of a fault in the output filter was discussed in Section II, where we argued that the converter dynamics in the presence of this fault can be described by (3). Let $e_{i1} = i_1 - \hat{i}_1$, $e_{i2} = i_2 - \hat{i}_2$ and $e_v = v - \hat{v}$. Then, by subtracting (9) from (3), it follows that $\dot{e} = -\mu e + \phi_\sigma(t)f$, where $e = [e_{i1}, e_{i2}, e_v]'$. For $t$ large enough, $e(t)$ remains in a fixed direction parallel to $f = [0,0,1]'$ and therefore a fault in one of the capacitors (whether is “soft” or “hard”) can be easily identified. It is important to note that due to the symmetry of the circuit, it is impossible to distinguish which capacitor in the filter bank is faulty, i.e., the fault cannot be isolated even if it has been detected.

> **Fig. 2.** Detection filter residual for a capacitor fault occurring at $t = 0.01$ s.

Figure 2 shows the detection filter residual for a simulated fault injected at $t = 0.01$ s in one of the output filter capacitors. As shown in the figure, after $t = 0.01$ s, the fault manifests in $e_v$ while $e_{i1}$ and $e_{i2}$ remain fixed at zero, consistent with the analysis above.

## C. Switch Faults

Suppose the detection filter for this system, specified in (9), is once again used to detect open- and short circuit faults in switches $SW_1$ and $SW_2$.

### 1. $SW_1$ open-circuit fault

The faulty dynamics are

$$
\frac{d}{dt}
\begin{bmatrix}
i_1 \\
i_2 \\
v
\end{bmatrix}
=
\begin{bmatrix}
-\frac{R_{L1}}{L_1} & 0 & -\frac{1}{L_1} \\
0 & -\frac{R_{L2}}{L_2} & -\frac{1}{L_2} \\
\frac{1}{2C} & \frac{1}{2C} & 0
\end{bmatrix}
\begin{bmatrix}
i_1 \\
i_2 \\
v
\end{bmatrix}
+
\begin{bmatrix}
0 & 0 & 0 \\
0 & \frac{\sigma_2(t)}{L_2} & 0 \\
0 & 0 & -\frac{1}{2C}
\end{bmatrix}
\begin{bmatrix}
V_1 \\
V_2 \\
i_{load}
\end{bmatrix},
\tag{10}
$$

and the fault detection filter residual reduces to

$$
\dot{e} = -\mu e + \sigma_1(t)\frac{V_1}{L_1}
\begin{bmatrix}
-1 \\
0 \\
0
\end{bmatrix}.
\tag{11}
$$

For $t$ large enough, the solution of (11) will remain in a fixed direction parallel to $[-1,0,0]'$.

In the simulation environment, a fault causing $SW_1$ to remain open was injected into the converter model at $t = 0.01$ s. The filter residual is shown in Fig. 3, where it can be seen that after $t = 0.01$ s, the fault manifests in $e_{i1}$ while $e_v$ and $e_{i2}$ remain fixed at zero, which is consistent with the analysis in the previous paragraph.

> **Fig. 3.** Detection filter residual for a switch open-circuit fault occurring at $t = 0.01$ s.

### 2. $SW_2$ short-circuit fault

The faulty dynamics are

$$
\frac{d}{dt}
\begin{bmatrix}
i_1 \\
i_2 \\
v
\end{bmatrix}
=
\begin{bmatrix}
-\frac{R_{L1}}{L_1} & 0 & -\frac{1}{L_1} \\
0 & -\frac{R_{L2}}{L_2} & -\frac{1}{L_2} \\
\frac{1}{2C} & \frac{1}{2C} & 0
\end{bmatrix}
\begin{bmatrix}
i_1 \\
i_2 \\
v
\end{bmatrix}
+
\begin{bmatrix}
\frac{\sigma_1(t)}{L_1} & 0 & 0 \\
0 & 0 & 0 \\
0 & 0 & -\frac{1}{2C}
\end{bmatrix}
\begin{bmatrix}
V_1 \\
V_2 \\
i_{load}
\end{bmatrix},
\tag{12}
$$

and the filter residual reduces to

$$
\dot{e} = -\mu e + \sigma_2(t)\frac{V_2}{L_2}
\begin{bmatrix}
0 \\
1 \\
0
\end{bmatrix}.
\tag{13}
$$

For $t$ large enough, the solution of (13) will remain in a fixed direction parallel to $[0,1,0]'$.

In the simulation environment, a fault causing $SW_2$ to remain closed was injected into the converter model at $t = 0.01$ s. The filter residual is shown in Fig. 4, where it can be seen that after $t = 0.01$ s, the fault manifests in $e_{i2}$ while $e_v$ and $e_{i1}$ remain fixed at zero. This is again consistent with the analysis above.

> **Fig. 4.** Detection filter residual for a switch short-circuit fault occurring at $t = 0.01$ s.

# V. Experimental Verification

In order to experimentally verify the feasibility and effectiveness of the proposed detection filters, we developed a testbed for fault diagnosis in a (non-redundant) buck converter. A block diagram of the testbed is displayed in Fig. 5. Testbed implementation details and experimental results are discussed next.

> **Fig. 5.** Buck converter/DSP fault diagnosis testbed.

## A. Implementation Details

The testbed hardware implementation is illustrated in Fig. 6(a) and Fig. 6(b). The diagnosis system consists of the DSP board eZdsp™ F28335, where the detection filter is implemented and where the controller of the power electronics circuit also resides. Allegro ACS714 Hall-effect sensors are used to measure inductor currents with a bandwidth of 80 kHz. Capacitor voltages are measured through a resistor divider and conditioned before being passed to the DSP. These measurements are passed to the DSP for both control and fault diagnosis.

All parameters and sensors connected to the DSP were calibrated using an external oscilloscope and voltmeter. Furthermore all passive components were characterized on an LCR meter and programmed into the simulation on the real time target at compile time. As shown in Fig. 5 and Fig. 6(b), an automated fault injection mechanism disables part of the output capacitance. This is accomplished by a FET which is connected to the ground lead of a standard capacitor and controlled from the DSP allowing the fault to be generated and recorded in a predictable manner. Similar mechanisms can be implemented to enable the injection of other faults.

To record data from the model of the DSP all internal parameters are stored in an external memory attached to the DSP which allows for all channels to be recorded at the DSP sampling frequency (and model step frequency) of 100 kHz. Up to 16 channels are recorded as 32 bit floating point numbers which include both the measurements of all analog channels (voltage in, current in, inductor current, voltage out and current out) and also the corresponding parameters of the discrete-time model running in software.

As the DSP has no external communication interface that can be connected to a computer that can transfer all this data in realtime (about 32 MB of data per second) snapshots are recorded. These are then transmitted to the computer in approximately 0.02 s windows. The programming of the DSP synchronizes fault injection with the trigger of the recording in software. All 16 channels of data are then transmitted over a an rs232 connection to the computer where a MATLAB® script deserializes the data and stores the results for later analysis.

> **Fig. 6.** Hardware platform.  
> (a) Fault Diagnosis testbed.  
> (b) Buck converter interfacing with DSP.

## B. Experimental Results

We verified the effectiveness of the diagnosis system for detecting capacitor faults. We also verified the robustness of the system against false positives after sudden changes in converter load and output voltage setpoint. The detection filter formulation is based on the buck converter model of Fig. 5. The model parameters as well as the detection filter gain $\mu$ are given in Table II.

### Capacitor fault

The fault injector implemented in the DSP injects a fault at approximately $t = 0.01$ s, causing the output filter capacitance to suddenly drop. The filter residual, calculated in real time by the DSP, is shown in Fig. 7, where it can be seen that after $t = 0.01$ s, the fault manifests in $e_v$ while $e_i$ remains fixed. We can also see that both the output voltage ripple increases dramatically causing the error between the observer and actual voltage to increase dramatically. To further filter this output for computer processing, we pass the samples through a four pole FIR high pass filter, and then take the absolute value of the samples. By low-pass filtering this output, an envelope detector can be developed, which gives an average output proportional to the size of the fault. By using a software-implemented threshold detector, the DSP can estimate the extent of the fault within a few cycles of its occurrence.

> **Fig. 7.** Capacitor voltage $v_c$, and filter residuals $e_i$ and $e_v$ for capacitor fault injected around $t = 0.01$ s.

### Load switching

This type of converter is easy to characterize under static load, however most power electronics converters operate to provide power to varying loads. Since this algorithm relies on the switching characteristics of the converter to provide fault detection, it is vitally important that sudden changes in load do not interfere with the detection algorithm and cause false positives to happen. While the converter is running, half of its load is switched out at $t = 0.01$ s, which results resulting in the the load resistor changing from $3.5\ \Omega$ to $7\ \Omega$. Figure 8 shows the inductor current $i_L$, the estimate inductor current $\hat{i}_L$ and the filter residual $e_i$. In this figure, it can be seen that the inductor current drops after $t = 0.01$ s, and how the filter accurately predicts this change, which results in the filter residual $e_i$ not significantly changing.

> **Fig. 8.** Inductor current $i_L$, estimated inductor current $\hat{i}_L$, and filter residual $e_i$ for a 50% load resistor at $t = 0.01$ s.

## Table II. Dual-Redundant Buck Converter Parameters

| Parameter | Value | Parameter | Value |
|---|---:|---|---:|
| $R_L$ [$\Omega$] | $10^{-3}$ | $V$ [V] | $13$ |
| $L$ [H] | $5 \cdot 10^{-4}$ | $V_{out}$ [V] | $5$ |
| $C$ [F] | $5.8 \cdot 10^{-4}$ | $R$ [$\Omega$] | $3.5, 7$ |
| $f$ [kHz] | $10$ | $\mu$ | $8 \cdot 10^3$ |

### Voltage output switching

The robustness of the proposed detection filters also becomes apparent for the case when the output voltage setpoint suddenly changes. In order to determine the capacitor health, it is easy to measure the ripple of the capacitor voltage under a static load. However, if the output voltage needs to be suddenly changed, this simplistic measurement will not work as the rapidly changing output voltage would overwhelm the ripple voltage causing a false positive to occur. With the proposed detection filter, it is possible to monitor capacitor health under varying conditions with high accuracy. In Fig. 9, we can see that the capacitor voltage error $e_v$ stays within the same bounds as under a static load for an output voltage setpoint change that causes the voltage capacitor to change from 5 V to 6 V.

> **Fig. 9.** Capacitor voltage $v_c$, estimated capacitor voltage $\hat{v}_c$, and filter residual $e_v$ for a voltage input change of 20% at $t = 0.01$ s.

# VI. Concluding Remarks

We develop fault-sensitive detection filters for power electronics systems based upon piecewise linear observers. We provided the theoretical foundations to design these filters, and showed simulations and experimental results proving the feasibility and effectiveness of the proposed diagnosis approach.

Observer-based detection filters provide fast fault detection times, which allows a fault to be flagged as soon as it occurs. They also allow a fault to be detected while not flagging transient source, load, or converter behavior as faults, i.e., they are robust against false positive events. This makes them very useful in detecting faults systems where the converter powers dynamic loads such as the electric motor in an automobile. Furthermore, fast detection of hard faults allows the converter to be shut down or having its operating setpoint changed to be within tolerable operating limits, allowing operation without damage even if the converter has partly failed.
