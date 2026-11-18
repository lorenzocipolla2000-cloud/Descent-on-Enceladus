# Descent-on-Enceladus
# Enceladus Landing GNC — Powered descent simulator for Enceladus

**English** · [Italiano](README.it.md)

Closed-loop Simulink simulator of the powered descent of a lander onto the surface of Enceladus. The **guidance – navigation – control** chain is complete: no onboard block uses the true state, only the estimates produced by the navigation filters.

> Personal study project, inspired by the public studies of the ESA L4 (Enceladus) mission. **Not affiliated with ESA**; no restricted data are used.

<!-- Insert a figure here: 3D trajectory or altitude/velocity vs time -->
<!-- ![Nominal descent](docs/img/nominal_descent.png) -->

---

## Contents

- [Features](#features)
- [Models](#models)
- [Requirements](#requirements)
- [How to run](#how-to-run)
- [Assumptions and limitations](#assumptions-and-limitations)
- [Future work](#future-work)
- [References](#references)
- [Author](#author)

---

## Features

- **Translational dynamics** in the Hill rotating frame centred on Enceladus (Coriolis, Saturn tidal term, centrifugal term, Keplerian gravity), with variable mass. Validated with the Jacobi constant (relative drift ~2.5e-8 over 9000 s).
- **Free-final-time optimal guidance** (D'Souza, 1997): $t_{go}$ is obtained at each step from a quartic solved with Newton's method.
- **Attitude control**: main engine pointed along the commanded thrust, PD on the error quaternion with angular-velocity *feedforward*.
- **Attitude actuators**: 12 RCS thrusters of 22 N (4 clusters of 3) with PWPF modulation and torque allocation.
- **Navigation**:
  - sensors: accelerometer and gyroscope (white noise + random-walk bias), radar altimeter, TRN camera, star tracker;
  - 9-state **translational EKF** $[\mathbf p;\ \mathbf v;\ \mathbf b_a]$;
  - 6-error-state **attitude MEKF** $[\delta\boldsymbol\theta;\ \delta\mathbf b_g]$;
  - consistency checks with $\pm3\sigma$ bounds and NEES.

## Models

### Translational dynamics (Hill frame)

$\hat x$ points towards Saturn, $\hat z$ is normal to the orbital plane, $\hat y$ completes the triad. The coefficients $k_C, k_S, k_\omega$ switch the Coriolis, tidal and centrifugal terms on and off.

$$
\begin{aligned}
\ddot x &= 2k_C n\dot y + k_\omega n^2x + 2k_S n^2x - \frac{\mu x}{\|\mathbf p\|^3} + \frac{T_x}{m}\\
\ddot y &= -2k_C n\dot x + k_\omega n^2y - k_S n^2y - \frac{\mu y}{\|\mathbf p\|^3} + \frac{T_y}{m}\\
\ddot z &= -k_S n^2z - \frac{\mu z}{\|\mathbf p\|^3} + \frac{T_z}{m}
\end{aligned}
\qquad
\dot m = -\frac{\|\mathbf T\|}{I_{sp}g_0}
$$

### Guidance (D'Souza, free final time)

Minimises $J=\Gamma t_f+\tfrac12\int\mathbf a^T\mathbf a\,dt$. The time-to-go is the largest real root of

$$
\left(\Gamma+\tfrac{g^2}{2}\right)t_{go}^4-2(\mathbf v^T\mathbf v+\mathbf v^T\mathbf v_f+\mathbf v_f^T\mathbf v_f)\,t_{go}^2+12\,\mathbf D^T(\mathbf v+\mathbf v_f)\,t_{go}-18\,\mathbf D^T\mathbf D=0,
\qquad \mathbf D=\mathbf p_f-\mathbf p
$$

and the commanded acceleration takes the ZEM/ZEV form:

$$
\mathbf a_{cmd}=\frac{6\,\mathbf{ZEM}}{t_{go}^2}-\frac{2\,\mathbf{ZEV}}{t_{go}},\qquad
\mathbf{ZEM}=\mathbf p_f-\mathbf p-\mathbf v\,t_{go}-\tfrac12\mathbf g\,t_{go}^2,\qquad
\mathbf{ZEV}=\mathbf v_f-\mathbf v-\mathbf g\,t_{go}
$$

The target is placed half a metre below the surface with $\mathbf v_f=\mathbf 0$, and $t_{go}$ is bounded below by $t_{go,min}=10$ s to avoid coupling between the guidance and attitude bandwidths in the last seconds.

### Attitude

The main engine is fixed along the body axis $\hat b_0=[0;0;1]$: the body rotates to align it with $\hat{\mathbf a}_{cmd}$.

$$
J\dot{\boldsymbol\omega}=\mathbf M-\boldsymbol\omega\times J\boldsymbol\omega,\qquad
\mathbf M_{des}=k_p\,\mathrm{sign}(\delta q_0)\,\delta\mathbf q_v+k_d(\boldsymbol\omega_{des}-\boldsymbol\omega)
$$

The *feedforward* term $\boldsymbol\omega_{des}=A(q)\,(\hat{\mathbf a}\times\dot{\hat{\mathbf a}})$ removes the tracking lag of the thrust direction. Diagnostics: pointing error $\varepsilon$ and thrust efficiency $\eta=\cos\varepsilon$.

### Navigation

Navigation estimates the lander's position, velocity and attitude from noisy measurements. It is organised in three layers:

1. the **sensors** take the true state from the plant and corrupt it (noise, bias);
2. at every IMU step ($\Delta t=0.02$ s) the **filters predict** the state by integrating accelerometer and gyroscope data and propagate the uncertainty $P$;
3. when an absolute measurement arrives (altimeter, camera, star tracker) the filter **corrects** the prediction, weighting the prediction uncertainty against the sensor uncertainty.

Sensors and filters are separate blocks: the sensor sees the true state, the filter sees only the measurements.

#### Sensors

| Sensor | Rate | Measures | Model |
|---|---|---|---|
| Accelerometer | 50 Hz | specific force in body axes | $\tilde{\mathbf a}_b=A(q)\,\mathbf T/m+\mathbf F_{rcs,b}/m+\mathbf b_a+\boldsymbol\eta_a$ |
| Gyroscope | 50 Hz | angular velocity in body axes | $\tilde{\boldsymbol\omega}=\boldsymbol\omega+\mathbf b_g+\boldsymbol\eta_v$ |
| Radar altimeter | 10 Hz | altitude | $\tilde h=\|\mathbf p\|-R+\nu_h$, $\ \sigma_h=\sigma_{h0}+k_h h$ |
| TRN camera | 1 Hz | 3D position | $\tilde{\mathbf p}_c=\mathbf p+\boldsymbol\nu_c$, $\ \sigma_c=\max(\sigma_{c0},k_c h)$ |
| Star tracker | 5 Hz | attitude | $\tilde q=\delta q(\delta\boldsymbol\theta)\otimes q$, $\ \sigma_{st}\approx10''$ |

Biases evolve as random walks, $\mathbf b_{k+1}=\mathbf b_k+\sigma_b\sqrt{\Delta t}\,\mathbf w_k$. The accelerometer only senses contact forces (main thrust and RCS): it does not see gravity or the Hill terms, which the filter recomputes from the model.

#### Translational EKF (9 states)

State $\hat{\mathbf x}=[\hat{\mathbf p};\ \hat{\mathbf v};\ \hat{\mathbf b}_a]$ in the Hill frame.

**Prediction** with the accelerometer and the Hill model:

$$
\hat{\mathbf a}=\mathbf g(\hat{\mathbf p},\hat{\mathbf v})+A(\hat q)^T(\tilde{\mathbf a}_b-\hat{\mathbf b}_a),\qquad
\hat{\mathbf p}_{k+1}=\hat{\mathbf p}_k+\hat{\mathbf v}_k\Delta t+\tfrac12\hat{\mathbf a}\Delta t^2,\qquad
\hat{\mathbf v}_{k+1}=\hat{\mathbf v}_k+\hat{\mathbf a}\Delta t
$$

$$
F=\begin{bmatrix}0&I&0\\ G_p&G_v&-A^T\\ 0&0&0\end{bmatrix},\qquad
\Phi=I+F\Delta t+\tfrac12F^2\Delta t^2,\qquad
P_{k+1}=\Phi P_k\Phi^T+Q_d
$$

where $G_p$ and $G_v$ are the Jacobians of the Hill acceleration with respect to position and velocity.

**Correction** with the altimeter ($H=[\hat{\mathbf u}^T\ 0\ 0]$, $\hat{\mathbf u}=\hat{\mathbf p}/\|\hat{\mathbf p}\|$) and the camera ($H_c=[I\ 0\ 0]$):

$$
\mathbf y=\tilde{\mathbf z}-h(\hat{\mathbf x}^-),\qquad
K=P^-H^T(HP^-H^T+R)^{-1},\qquad
\hat{\mathbf x}^+=\hat{\mathbf x}^-+K\mathbf y,\qquad
P^+=(I-KH)P^-(I-KH)^T+KRK^T
$$

The covariance update uses the Joseph form, which keeps $P$ symmetric and positive definite. When two measurements arrive in the same step, the corrections are applied sequentially.

#### Attitude MEKF (6 error states)

The quaternion has 4 components tied by the unit-norm constraint, so it cannot be used directly as an EKF state. The multiplicative filter instead estimates a **small angle error** $\delta\boldsymbol\theta$ around the estimate, $q=\delta q(\delta\boldsymbol\theta)\otimes\hat q$, together with the gyroscope bias: error state $[\delta\boldsymbol\theta;\ \delta\mathbf b_g]$, $6\times6$ covariance.

**Prediction** with the gyroscope:

$$
\hat{\boldsymbol\omega}=\tilde{\boldsymbol\omega}-\hat{\mathbf b}_g,\qquad
\hat q_{k+1}=\begin{bmatrix}\cos\frac{\|\boldsymbol\phi\|}{2}\\ \frac{\boldsymbol\phi}{\|\boldsymbol\phi\|}\sin\frac{\|\boldsymbol\phi\|}{2}\end{bmatrix}\otimes\hat q_k,\quad \boldsymbol\phi=\hat{\boldsymbol\omega}\Delta t,\qquad
F=\begin{bmatrix}-[\hat{\boldsymbol\omega}\times]&-I\\ 0&0\end{bmatrix}
$$

**Correction** with the star tracker: the innovation is the small rotation between measurement and estimate,

$$
\delta q_{res}=\tilde q_{st}\otimes\hat q^{-1},\qquad \mathbf y=2\,\delta\mathbf q_{res,v},\qquad H=[I\ 0]
$$

After the update the estimated error is folded into the quaternion (*reset*), $\hat q^+=[1;\ \delta\hat{\boldsymbol\theta}/2]\otimes\hat q^-$, and $\delta\boldsymbol\theta$ returns to zero.

Quaternion convention: scalar first, $A(p\otimes q)=A(p)A(q)$, $\dot q=\tfrac12[0;\ \boldsymbol\omega]\otimes q$.

#### Filter verification

- **$\pm3\sigma$ bounds**: for each component the true error $\hat x_i-x_i$ is compared with $\pm3\sqrt{P_{ii}}$. A consistent filter stays within the bounds.
- **NEES** (*Normalized Estimation Error Squared*), $\varepsilon=\mathbf e^TP^{-1}\mathbf e$: for a consistent filter it follows a $\chi^2$ distribution with as many degrees of freedom as states (mean 9 for the translational EKF). Values well above indicate an optimistic filter, well below a pessimistic one.
- **Incremental testing**: each sensor and each correction was added and verified one at a time (prediction only first, then altimeter, camera, star tracker) before closing the loop.

<!-- ![EKF error and ±3σ bounds](docs/img/ekf_bounds.png) -->
<!-- ![NEES](docs/img/nees.png) -->

#### Closed-loop integration

| Block | Uses |
|---|---|
| Guidance | $\hat{\mathbf p},\hat{\mathbf v}$ from the EKF |
| Attitude control and feedforward | $\hat q,\hat{\boldsymbol\omega}$ from the MEKF |
| Translational EKF | $\hat q$ from the MEKF (to rotate the accelerometer measurement) |
| Sensors, plant, thrust projection | true state |

The path guidance → thrust → accelerometer → EKF → guidance forms an algebraic loop within the same time step. It is broken by a one-IMU-step delay on the estimates fed to the guidance, representing onboard computation time. No delay is needed on the attitude side, since the MEKF depends only on the sensors.

## Requirements

- MATLAB and Simulink (developed with R20XXx — *to be completed*)
- Aerospace Toolbox (`quatmultiply`, `quatinv` in the attitude PD)

## How to run

```matlab
cd scripts
run_encelado          % or: init_encelado; sim('encelado')
```

Solver: fixed-step `ode4`, 0.01 s. All sensor and filter sample times are multiples of 0.01 s. The simulation stops when the true altitude reaches 0 (StopTime 500 s as a safety limit).

## Assumptions and limitations

- Spherical, homogeneous Enceladus (no $J_2$); smooth spherical terrain for altimeter and camera.
- Camera modelled as an ideal 3D position fix (TRN), with no image processing.
- Mass known onboard.
- The Hill frame is treated as inertial for attitude (the gyroscope does not sense $n\approx5.3\cdot10^{-5}$ rad/s).
- MEKF discretisation error during fast manoeuvres (≤0.045°), not modelled in the covariance.

## Future work

- Disturbances: thrust magnitude and direction errors, non-spherical gravity, torque from centre-of-mass offset.
- Minimum main-engine thrust (10–20% throttle).
- Adaptation to the L4 mission profile: south-pole landing site, release from NRHO, engine *cutoff* and final free fall, *hazard detection & avoidance* with in-flight *retargeting*.
- Convex guidance (G-FOLD) and Monte Carlo analysis.

## References

- C. D'Souza, *An Optimal Guidance Law for Planetary Landing*, AIAA 97-3709, 1997.
- B. Açıkmeşe, S. Ploen, *Convex Programming Approach to Powered Descent Guidance for Mars Landing*, JGCD, 2007.
- F. L. Markley, J. L. Crassidis, *Fundamentals of Spacecraft Attitude Determination and Control*, Springer, 2014.
- J. Holtkamp, *Enceladus lander GNC*, MSc thesis, TU Delft, 2014.
- K. Konstantinidis, *Enceladus Explorer*, Universität der Bundeswehr München, 2019.
- ESA, *Voyage 2050 – Expert Committee report on the L4 mission (Moons of the Giant Planets)*.

## Author

**Lorenzo [Surname]** — Aerospace Engineering, Politecnico di Torino
[LinkedIn](#) · [email](#)

## License

Released under the MIT License (see `LICENSE`).
