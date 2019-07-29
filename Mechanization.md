## Formula

$$
\begin{aligned}
C ^\alpha _\beta &\approx \begin{bmatrix}
1 & \psi_{\beta \alpha} & -\theta_{\beta \alpha} \\
-\psi_{\beta \alpha} & 1 & \phi_{\beta \alpha} \\
\theta_{\beta \alpha}& - \phi_{\beta \alpha} & 1
\end{bmatrix}
\\
 & = \mathbf{I_3 + [\Psi_{\beta \alpha}\times]}

\end{aligned}
$$

$$
\begin{aligned}
P_{gps} &= P_{ins} + C^n _b l_g \\
V_{gps} &= V_{ins} + C^n _b [\Omega ^b _{nb}\times]C^n _b l_g
\end{aligned}
$$
$$
\begin{aligned}
H_{ins} = \begin{bmatrix}
I_3& O& [C^n _b l_g\times]& O& O& C^n _b \\
O& I_3& C^n_b([w_{nb}^b\times][l_g \times]-[l_g \times][w_{nb}^b\times]) {C^n_b}^T& O &-C^n_b [l_g \times] & C^n_b [w_{nb}^b\times]
\end{bmatrix}
\end{aligned}
$$

$$
G = \begin{bmatrix}
O&O&O&O\\C_b^n&O&O&O\\O&-C_b^n&O&O\\
O&O&I_3&O\\O&O&O&I_3\\O&O&O&O
\end{bmatrix}
\begin{bmatrix}
\sigma _a&O&O&O\\ O&\sigma _w&O&O\\O&O&\sigma _{b_a}&O\\O&O&O&\sigma _{b_w}\end{bmatrix}
\begin{bmatrix}
O&O&O&O\\C_b^n&O&O&O\\O&-C_b^n&O&O\\
O&O&I_3&O\\O&O&O&I_3\\O&O&O&O
\end{bmatrix}^T
$$

$$
\begin{aligned}
\hat{V}_{nhc}&=[I-\gamma \times]C_b^a(C_n^b V_{ins} + w_{nb}^b \times l_o)\\
&= V_{nhc} + C_b^a [w_{nb}^b \times] \delta l_o + [C_b^a(C_n^b V_{ins} + [w_{nb}^b \times]l_o)\times]\gamma
\end{aligned} 
$$

$$
\begin{aligned}
V_{nhc}& =\begin{bmatrix}
    V_{gps}\\0\\0
\end{bmatrix} = C_b^a C_n^b V_{ins} + C_b^a[w_{nb}^b \times]l_o 
\\
H_{nhc} &= \begin{bmatrix}
    [C_b^a * (C_n^b V_{ins} + [w_{nb}^b \times]l_o) \times] & C_b^a [w_{nb}^b \times]
\end{bmatrix}
\end{aligned}
$$

%md
Displayed matrix works:
$$ \begin{bmatrix}
    a & b \\ b & c
\end{bmatrix}$$

Non-displayed matrix does not work due to some backslash escaping we do not want touching tex:
$\begin{bmatrix}
    a & b \\ b & c
\end{bmatrix}$

Proof that this is the problem:
$\begin{bmatrix}
    a & b \\\\ b & c
\end{bmatrix}$

