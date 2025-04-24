# Embodied-Teleoperation-System

## Data processing

Task completion time was defined as the interval from the commencement of insertion by the subject to the elimination of the final target in Phase 3.

Assuming that the instrument maintained a constant curvature, the tip position ($x_{tip}$, $y_{tip}$, $z_{tip}$) was computed using the measured flexion and rolling angles, in conjunction with the position of the robot end-effector.
An ideal trajectory was defined along the elliptical surface of the bladder to quantify deviations from the actual movement. The center ($x_{c}$, $y_{c}$, $z_{c}$) and the radii ($r_{x}$, $r_{y}$, $r_{z}$) of the elliptical bladder were uniformly defined for each subject.

$$\frac{(x-x_c)^2}{r_x^2} + \frac{(y-y_c)^2}{r_y^2} + \frac{(z-z_c)^2}{r_z^2} = 1$$

The target plane for each phase was defined such that it remains parallel to the y-axis (i.e., with a zero y-coefficient).
This was achieved by selecting two distinct points, $P_1 = (x_1, y_1, z_1)$ and $P_2 = (x_2, y_2, z_2)$, and constructing a plane whose normal vector is orthogonal to the y-axis direction vector, $\mathbf{v_2} = (0, 1, 0)$.
The direction vector between the two points is  $\mathbf{v_1} = P_2 - P_1 = (x_2 - x_1, y_2 - y_1, z_2 - z_1)$, and the cross product  $\mathbf{n} = \mathbf{v_1} \times \mathbf{v_2}$ yields a normal vector that lies in the xz-plane, thereby ensuring that the resulting plane is parallel to the y-axis.


$$
\begin{equation}
\mathbf{n} = 
\begin{vmatrix}
\hat{\imath} & \hat{\jmath} & \hat{k} \\
x_{2} - x_{1} & y_{2} - y_{1} & z_{2} - z_{1} \\
0 & 1 & 0 \\
\end{vmatrix}
= (-(z_{2} - z_{1}),\quad 0,\quad x_{2} - x_{1})
\end{equation}
$$

Using this normal vector and the point $P_1$, the equation of the plane can be expressed in point-normal form:

$$ -(z_2 - z_1)(x - x_1) + (x_2 - x_1)(z - z_1) + d = 0$$

where the constant $d$ is determined by substituting one of the points.
Thus, the ideal trajectory was obtained as the intersection curve of the phase-specific target plane and the elliptical bladder surface.
The objective function, $f$, is defined as the squared Euclidean distance between the calculated position of the tip and an arbitrary point $(x, y, z)$ trajectory.

$$f(x, y, z) = (x - x_{\text{tip}})^2 + (y - y_{\text{tip}})^2 + (z - z_{\text{tip}})^2$$

The equation of the elliptical curve, determined by the positions of the targets within the same phase, is imposed as a constraint function $g$.

$$g(x, y, z) = \frac{(x - x_c)^2}{r_x^2} + \frac{(y - y_c)^2}{r_y^2} + \frac{(z - z_c)^2}{r_z^2} - 1 = 0$$

Equations below were derived by substituting the plane equation into the above two equations.

$$f(x, y) = (x - x_{\text{tip}})^2 + (y - y_{\text{tip}})^2 + \left( \frac{(z_2 - z_1)(x - x_1) - d}{x_2 - x_1} + z_1 - z_{\text{tip}} \right)^2$$

$$g(x, y) = \frac{(x - x_c)^2}{r_x^2} + \frac{(y - y_c)^2}{r_y^2} + \frac{\left( \frac{(z_2 - z_1)(x - x_1) - d}{x_2 - x_1} + z_1 - z_c \right)^2}{r_z^2} - 1 = 0$$

The Lagrange multiplier method was employed to compute the tip position deviation while minimizing $f$ and satisfying $g$ [1].

### References

1. A. Y. Uteshev, M. V. Yashina, Metric problems for quadrics in multidimensional space. Journal of Symbolic Computation 68, 287–315 (2015), doi:https://doi.org/10.1016/j.jsc.2014.09.021, https://www.sciencedirect.com/science/article/pii/S0747717114000893.
