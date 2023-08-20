<script
  src="https://cdn.mathjax.org/mathjax/latest/MathJax.js?config=TeX-AMS-MML_HTMLorMML"
  type="text/javascript">
</script>

# Visual Inertial SLAM

## Dead Reckoning
This uses purely control inputs to estimate where our agent is. This is a naive approach that accumulates error in motion noise.
<img src='03_CombinedDR.gif'>
<img src='10_CombinedDR.gif'>



## SLAM
This uses stereo camera observations to estimate where our agent is and where our observed landmarks are. This utilizes the Extended Kalman Filter (EKF) to estimate the pose of our agent and the position of our landmarks.

- The left visual shows the left camera frame with features overlayed.
- The center visual shows the landmarks as they are seen in each frame.
- The right visual shows all the landmarks that have been seen thus far.

<img src='03_CombinedSLAMAnimation.gif'>
<p>
    <img src='10_animation.gif' width=33%>
    <img src='10_CombinedSLAM.gif' width=66%>
</p>


# Mathematical Approach
## Variables
### States
$\text{IMU Pose}: T_t \in SE(3)$

$\text{Landmark Positions}: \mathbf{m} := \begin{bmatrix}
\mathbf{m}_1^\top, ..., \mathbf{m}_M^\top
\end{bmatrix}^\top \in \mathbb{R}^{3M}$

### Control
$\text{Linear Velocity}: \mathbf{v} \in \mathbb{R}^3$

$\text{Angular Velocity}: \mathbf{\omega} \in \mathbb{R}^3$

$\text{Twist}: \mathbf{u} = \begin{bmatrix}
\mathbf{v}\\
\mathbf{\omega}
\end{bmatrix} \in \mathbb{R}^6$

### Observation
$\text{Stereo Camera Features}: \mathbf{z}_t := \begin{bmatrix}
\mathbf{z}_{t,1}^\top, ..., \mathbf{z}_{t,N_t}^\top
\end{bmatrix}^\top \in \mathbb{R}^{4N_t}$

## Models
### Kinematics/Motion Model
$\text{Motion Noise}: \mathbf{w}_t \sim \mathcal{N}(\mathbf{0}, W)$

$\text{Continuous Time Kinematics}:\dot{T} = T(\hat{\mathbf{u}} + \hat{\mathbf{w}})$

$\text{Pose as a Nominal + Perturbation} : T = \mathbf{\mu}\exp(\hat{\delta\mathbf{\mu}}) \approx \mu\left(I + \hat{\delta\mathbf{\mu}}\right)$

$\text{Nominal Kinematics Model}: \mathbf{\mu}_{t+1} = \mathbf{\mu}_t\exp(\tau_t\hat{\mathbf{u}}_t)$

$\text{Perturbation Kinematics Model}:\delta\mathbf{\mu}_{t+1}=\exp(-\tau_t\overset{\curlywedge}{\mathbf{u}}_t)\delta\mathbf{\mu}_t + \mathbf{w}_t$

### Observation/Measurement Model
$\text{Measurement Noise}: \mathbf{v}_t \sim \mathcal{N}(\mathbf{0}, V)$

$\text{Observation Model}: \mathbf{z}_{t+1,i} = K_s\pi\left(_oT_I T_{t+1}^{-1} \mathbf{\underbar{m}}_j\right) + \mathbf{v}_{t+1,i}$

$K_s := \begin{bmatrix}
fs_u & 0 & c_u & 0\\
0 & fs_v & c_v & 0\\
fs_u & 0 & c_u & -fs_u b\\
0 & fs_v & c_v & 0
\end{bmatrix} : \begin{bmatrix}
f: \text{focal length (m)}\\
s_u, s_v: \text{pixel scaling (pixels/m)}\\
c_u, c_v: \text{principal point (pixels)}\\
b: \text{stereo baseline (m)}
\end{bmatrix}$

$\pi(\mathbf{q}) := \frac{1}{q_3}\mathbf{q} \in \mathbb{R}^4$

$\text{Data Association}: \Delta_t(j) = i$

## Extended Kalman Filter (EKF)
### Prior Knowledge
$T_t|\mathbf{z}_{0:t},\mathbf{u}_{0:t-1} \sim \mathcal{N}(\mathbf{\mu}_{t|t,T}, \Sigma_{t|t,T})$

$\mathbf{m}_t|\mathbf{z}_{0:t},\mathbf{u}_{0:t-1} \sim \mathcal{N}(\mathbf{\mu}_{t|t,m}, \Sigma_{t|t,m})$

$\text{Joint Prior}: \begin{bmatrix}
\mathbf{m}_t|\mathbf{z}_{0:t},\mathbf{u}_{0:t-1} \\
T_t|\mathbf{z}_{0:t},\mathbf{u}_{0:t-1}
\end{bmatrix} \sim \mathcal{N}(\mu_{t|t,SLAM}, \Sigma_{t|t,SLAM})$

$\mu_{t|t,SLAM} = \begin{bmatrix}
\mathbf{\mu}_{t|t,m}\\
\mathbf{\mu}_{t|t,T}
\end{bmatrix}$

$\Sigma_{t|t,SLAM} = \begin{bmatrix}
\Sigma_{t|t,m} & \Sigma_{t|t,mT}\\
\Sigma_{t|t,Tm} & \Sigma_{t|t,T}
\end{bmatrix}$

### Prediction Step
$\mathbf{\mu}_{t+1|t,m} = \mathbf{\mu}_{t|t,m}: \text{Assume landmarks do not move}$

$\mathbf{\mu}_{t+1|t,T} = \mathbf{\mu}_{t|t,T}\exp(\tau_t \hat{\mathbf{u}}_t)$

$\Sigma_{t+1|t,SLAM} = \begin{bmatrix}
\Sigma_{t|t,m} & \Sigma_{t|t,mT}F^\top\\
F\Sigma_{t|t,Tm} & F\Sigma_{t|t,T}F^\top + W
\end{bmatrix}$

$F = \exp(\tau_t\overset{\curlywedge}{\mathbf{u}}_t)$

### Update Step

#### Measurement Jacobians
$H_{t+1,m,i,j} = \begin{cases}
K_s \frac{d\pi}{d\mathbf{q}}\left(_oT_I \mathbf{\mu}_{t+1|t,T}^{-1}\underline{\mathbf{\mu}}_{t+1|t,m,j}\right) {}_oT_I \mathbf{\mu}_{t+1|t,T}^{-1}P^\top & :\text{if } \Delta_t(j)=i\\
\mathbf{0} & :\text{otherwise}
\end{cases}$

$P := \begin{bmatrix}I & \mathbf{0}\end{bmatrix} \in \mathbb{R}^{3\times4}$

$H_{t+1,T,i} = -K_s \frac{d\pi}{d\mathbf{q}}\left(_oT_I \mathbf{\mu}_{t+1|t,T}^{-1}\underline{\mathbf{\mu}}_{t+1|t,m,j}\right)_oT_I \left(\mathbf{\mu}_{t+1|t,T}^{-1}\underline{\mathbf{\mu}}_{t+1|t,m,j}\right)^\odot$

$\hat\xi \underline{\mathbf{s}} = \underline{\mathbf{s}}^\odot \xi$

$\begin{bmatrix}\mathbf{s} \\ 1 \end{bmatrix}^\odot := \begin{bmatrix}
I  & -\hat{\mathbf{s}}\\
0 & 0
\end{bmatrix} \in \mathbf{R}^{4\times 6}$

$H_{t+1,SLAM} = \begin{bmatrix}H_{t+1,m} & H_{t+1,T}\end{bmatrix}$

#### Update Rule

$K_{t+1,SLAM} = \Sigma_{t+1|t,SLAM} H_{t+1,SLAM}^\top \left(H_{t+1,SLAM}\Sigma_{t+1|t,SLAM} H_{t+1,SLAM}^\top + I \otimes V\right)^{-1}$

$\mathbf{\mu}_{t+1|t+1, m} = \mathbf{\mu}_{t+1|t, m} + K_{t+1, m}(\mathbf{z}_{t+1} - \tilde{\mathbf{z}}_{t+1})$

$\mathbf{\mu}_{t+1|t+1, T} = \mathbf{\mu}_{t+1|t, T} + K_{t+1, T}(\mathbf{z}_{t+1} - \tilde{\mathbf{z}}_{t+1})$

$\Sigma_{t+1|t+1, SLAM} = (I - K_{t+1, SLAM}H_{t+1,SLAM})\Sigma_{t+1|t,SLAM}$