# l-dominance-mimo-control
A MATLAB/Simulink project for designing a decentralized PI controller for a gas turbine model using the L-dominance sequential design method.
# Design and Evaluation of a MIMO Controller for a Gas Turbine System using L-dominance

This project evaluates a multi-input-multi-output (MIMO) gas turbine system using the L-dominance method [1, 2] for diagonal dominance analysis. Based on this analysis, a pre-compensator is designed to reduce system interactions, followed by the implementation of decentralized PI controllers for a gas turbine model [3].

---

## 📝 Description

In the control of multivariable systems, interactions between different control loops often degrade performance. Diagonal dominance is a key concept for simplifying controller design by treating a complex MIMO system as a set of nearly independent single-input-single-output (SISO) systems.

This project focuses on the **L-dominance** method [1, 2], a sequential design technique for achieving diagonal dominance. Unlike traditional methods that require a full redesign if a compensator element changes, L-dominance allows for a sequential loop-closing approach. This simplifies the design process, as each loop can be designed and analyzed sequentially without altering previously designed loops.

The methodology is applied to a linearized model of a two-input, two-output gas turbine system [3]. A pre-compensator is designed to establish L-dominance, and then decentralized PI controllers are tuned for the resulting system. The final design is validated through simulation in Simulink.

---

## 📖 Methodology: The L-dominance Method

The L-dominance method provides a measure for assessing diagonal dominance based on the LU decomposition of a system's transfer function matrix. The following theory is based on the work presented in [1] and [2]. Consider a system with a pre-compensator $K(s)$ and controller $K_d(s)$ in a unity feedback configuration.

<p align="center">
  <img src="/figures/f1.png" width="600" alt="System Block Diagram">
  <br>
  <em>Figure 1: System Block Diagram [1]</em>
</p>

### Key Definitions and Theorems

1.  **L-dominance Condition for a Complex Matrix**
    A complex matrix $Z$ is considered L-dominant if, for its LU decomposition ($Z=LU$), the following condition holds for $i = 1, ..., n$:
    $|u_{ii}| \ge \sum_{j=i+1}^{n} |l_{ij}u_{jj}|$
    The dominance is strict if the inequality is strict.

2.  **Sequential Nyquist Theorem**
    A sequentially closed feedback system (closing loops from 1 to i-1 to analyze loop i) is stable if and only if:
    $\sum_{i=1}^{n} N_i = -p_0$
    where $p_0$ is the number of unstable open-loop poles and $N_i$ is the number of counter-clockwise encirclements of the critical point by the Nyquist plot of the i-th loop. This theorem allows a MIMO system to be analyzed with sequential Nyquist diagrams.

3.  **L-dominance and Sequential Stability**
    The closed-loop system is asymptotically stable if, for each loop $i=1,...,n$:
    * The Nyquist plot of $q_{ii}$ does not pass through the critical point $(-1/f_i, 0)$.
    * The Nyquist locus of $q_{ii}$ and its L-dominance circles (centered at $q_{ii}(j\omega)$ with radius $\delta_i^L(j\omega)$) exclude the critical point.
    * The number of counter-clockwise encirclements of the critical point by the Nyquist plot of $q_{ii}$ equals $p_0$.

---
# L-dominance Sequential Design Algorithm

This is a structured controller synthesis algorithm ensuring L-dominance and frequency-based stability margins.

## Step 0: Initialization

- Select a set of frequencies:  
  Ω = {ω₁, ω₂, ..., ωₘ}
  
- Define:  
  Kⁱ ≜ [K₁ | K₂ | ... | Kᵢ | eᵢ₊₁ | ... | eₙ],  
  where eⱼ is a unit column vector.

- Set:  
  L⁰(ωᵣ) = I  
  P⁰(ωᵣ) = I  
  K⁰ = I  
  δ₁ᴸ(ωᵣ) = 0  

- Initialize: i = 1

---

## Step 1: Design of Kᵢ(s)

1.1 Define the open-loop transfer function:  
  tᵢᵢ(s) = fᵢ⁻¹ + qᵢᵢ(s), where  
  qᵢᵢ(s) = Gᵢᵀ·Kᵢ(s) = Σ gᵢₗ(s)·kₗᵢ(s)

1.2 Use Nyquist design to compute Kᵢ(s):  
 - If qᵢᵢ(s) satisfies stability margins → continue  
 - Else → redesign this stage

1.3 Plot tᵢᵢ(jω) on the Nyquist plane, check winding number Nᵢ

1.4 For each ωᵣ ∈ Ω:  
 - Compute δᵢᴸ(T(jωᵣ))  
 - Draw a circle of radius δᵢᴸ(jωᵣ) centered at qᵢᵢ(jωᵣ)  
 - Choose fᵢ outside the L-dominance band

1.5 If all checks pass → go to Step 2  
  Else → repeat Step 1

---

## Step 2: Update

Update T⁽ⁱ⁻¹⁾ with new Kⁱ and compute eliminator matrix:

  Pₖⱼ⁽ⁱ⁾ = – tₖⱼ⁽ⁱ⁻¹⁾ / tᵢᵢ⁽ⁱ⁻¹⁾

---

## Step 3: Gauss Elimination

- T⁽ⁱ⁾(jωᵣ) = Pⁱ(jωᵣ) · T⁽ⁱ⁻¹⁾(jωᵣ)  
- L⁽ⁱ⁾(jωᵣ) = Pⁱ(jωᵣ) · L⁽ⁱ⁻¹⁾(jωᵣ)

---

## Step 4: Loop

- Increment i  
- If i = n → **STOP**  
- Else → **return to Step 1**
  
----

## 💨 The System Model: Gas Turbine [3]

The project uses a linearized model of a gas turbine operating at 80% gas generator speed and 85% power turbine speed.
* **Inputs:** Fuel pump excitation, Nozzle actuator.
* **Outputs:** Gas generator speed, Inter-turbine temperature.

The transfer function matrix $G(s)$ is:

$$
G(s) = \begin{bmatrix} \frac{0.806s + 0.264}{s^2+1.15s+0.202} & \frac{-(15.0s+1.42)}{s^3+12.8s^2+13.6s+2.36} \\\\ \frac{1.95s^2+2.12s+4.90}{s^3+9.15s^2+9.39s+1.62} & \frac{7.14s^2+25.8s+9.35}{s^4+20.8s^3+116.4s^2+111.6s+188} \end{bmatrix}
$$

Initial analysis using Gershgorin bands shows that the system has significant cross-coupling (interaction).

<p align="center">
  <img src="/figures/f2.png" width="800" alt="Gershgorin Bands for the Original System">
  <br>
  <em>Figure 2: Gershgorin Bands for the Original System</em>
</p>

---

## 🛠️ Controller Design & Implementation

The goal is to design a pre-compensator matrix $K$ such that the compensated plant $Q(s) = G(s)K(s)$ is diagonally dominant.

### 1. Pre-Compensator Design

The design follows a sequential process based on the L-dominance methodology.

**Loop 1 Design:**
The first column of $K$, which is $k_1 = [k_{11}, k_{21}]^T$, is designed to ensure the numerator of $t_{11}$ is minimum phase. This leads to a feasible region for the gains.

<p align="center">
  <img src="/figures/f3.png" width="800" alt="Acceptable Region for First Loop Gains">
  <br>
  <em>Figure 3: Acceptable Region for First Loop Gains (k11, k21)</em>
</p>

Based on this region, the following gains were chosen:
$$
k_{11} = 2, \quad k_{21} = -6
$$

**Loop 2 Design:**
The second column of $K$, $k_2 = [k_{12}, k_{22}]^T$, is chosen via trial and error while observing the L-dominance circles to ensure stability. The chosen values are:
$$
k_{12} = 2.5, \quad k_{22} = 5.5
$$

The final compensator matrix is:

$$
K = \begin{bmatrix} 2 & 2.5 \\\\ -6 & 5.5 \end{bmatrix}
$$

**Verification:**
The Nyquist plots for the diagonal elements of the compensated system, $q_{11}$ and $q_{22}$, along with the L-dominance circles for the second loop, confirm that the system is stable and diagonally dominant. The plots do not encircle the -1 point.

<p align="center">
  <img src="/figures/f4.png" width="800" alt="Nyquist Diagram of q11">
  <br>
  <em>Figure 4: Nyquist Diagram of q11</em>
</p>

<p align="center">
  <img src="/figures/f5.png" width="800" alt="Nyquist Diagram of q22 with L-dominance Circles">
  <br>
  <em>Figure 5: Nyquist Diagram of q22 with L-dominance Circles</em>
</p>

### 2. Decentralized PI Controller Design

With diagonal dominance achieved, the MIMO system can be treated as two independent SISO loops. Two PI controllers, $C_1(s)$ and $C_2(s)$, are tuned for the diagonal elements $t_{11}$ and $t_{22}$ respectively using MATLAB's `pidtune`.

$$
C(s) = \begin{bmatrix} C_1(s) & 0 \\\\ 0 & C_2(s) \end{bmatrix} = \begin{bmatrix} \frac{0.1807s + 0.7815}{s} & 0 \\\\ 0 & \frac{0.7251}{s} \end{bmatrix}
$$

---

## 🚀 Simulation & Results

The complete closed-loop system was simulated in Simulink.

<p align="center">
  <img src="/figures/f6.png" width="700" alt="Main Simulink Block Diagram">
  <br>
  <em>Figure 6: Main Simulink Block Diagram</em>
</p>



The simulation results below show the system's response (red line) to reference inputs (blue line).

<p align="center">
  <img src="/figures/f10.png" width="800" alt="System Response to Reference Inputs">
  <br>
  <em>Figure 7: System Response to Reference Inputs</em>
</p>

<p align="center">
  <img src="/figures/f11.png" width="800" alt="Control Signals">
  <br>
  <em>Figure 8: Control Signals (u)</em>
</p>

The results demonstrate that the controller successfully tracks the reference signals with minimal overshoot and significantly reduced interaction between the loops, as expected from the design.

---

## 🏁 Conclusion

The L-dominance method proved to be a powerful and straightforward technique for designing a controller for a complex MIMO system. Its main advantage is the sequential design process, which avoids the need for complete redesigns when parameters are adjusted.

By first designing a pre-compensator to achieve diagonal dominance and then applying simple decentralized PI controllers, effective control of the gas turbine system was achieved. The simulation confirmed that interactions between channels were drastically reduced and the control objectives were met.

---

## 📚 References

[1] L. Yeung and G. Bryant, "The L-dominance concept and its applications to multivariable system designs," *International Journal of Control*, vol. 56, no. 5, pp. 1079-1102, 1992.

[2] G. F. Bryant and L.-F. Yeung, *Multivariable control system design techniques*. John Wiley & Sons, Inc., 1996.

[3] N. Munro, O. Winterbone, and P. Lourtie, "Design of a multivariable controller for an automotive gas turbine," *IFAC Proceedings Volumes*, vol. 7, no. 1, pp. 315-323, 1974.

[4] M. Denham, "Computer Aided Control System Design. Bv HH ROSENBROCK.(Academic Press, 1974.)," *International Journal of Control*, vol. 22, no. 3, pp. 442-443, 1975.

---

## 👨‍💻 Author

* **Sohrab Rezaei**

---
