# Drone-Pose-Estimation
1. Velocity update equation:
   
$$ z_t = c_v W = c_p W = g(x_2, x_3, B \omega_B^W) + \eta \quad \eta \sim N(0, R)$$

3. Transformation matrix equation:
   
$$\begin{bmatrix}
c_p W \\
c_\omega W
\end{bmatrix}
=$$ $$\begin{bmatrix}
R_C^B & -R_B^C S(r_{BC}^B) \\
0 & R_B^C
\end{bmatrix}
\begin{bmatrix}
B \omega_B^W \\
B \omega_B^W
\end{bmatrix}$$
 
4. Inverse transformation matrix equation:
   
$$\begin{bmatrix}
B \omega_B^W \\
B \omega_B^W
\end{bmatrix} =$$
$$\begin{bmatrix}
R_C^B & -R_B^C S(r_{CB}^C) \\
0 & R_B^C
\end{bmatrix}
\begin{bmatrix}
C_p W \\
C_\omega W
\end{bmatrix}
$$


