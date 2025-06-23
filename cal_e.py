import re
import numpy as np
import argparse

def parse_txt(filename):
    """
    Parse a text log file with lines like:
    I (...) MPU6050: g_total(raw)=1.033 g (ax=1.029, ay=-0.019, az=0.090)
    Returns numpy arrays ax, ay, az.
    """
    pattern = re.compile(
        r"g_total\(raw\)=[0-9]*\.?[0-9]+\s*g\s*\(ax=([-\d\.]+),\s*ay=([-\d\.]+),\s*az=([-\d\.]+)\)"
    )
    ax_list, ay_list, az_list = [], [], []
    with open(filename, 'r') as f:
        for line in f:
            m = pattern.search(line)
            if m:
                ax_list.append(float(m.group(1)))
                ay_list.append(float(m.group(2)))
                az_list.append(float(m.group(3)))
    return np.array(ax_list), np.array(ay_list), np.array(az_list)

def fit_ellipsoid(ax, ay, az):
    """
    Fit general ellipsoid: A x^2 + B y^2 + C z^2 + D xy + E xz + F yz + G x + H y + I z + J = 0
    Returns bias (center), eigenvectors Q, and scales.
    """
    x, y, z = ax, ay, az
    M = np.column_stack([
        x*x, y*y, z*z,
        x*y, x*z, y*z,
        x,   y,   z,
        np.ones_like(x)
    ])
    # Solve by SVD
    _, _, Vt = np.linalg.svd(M, full_matrices=False)
    params = Vt[-1]
    A, B, C, D, E, F, G, H, I, J = params

    # Build quadratic form matrix and linear part
    M3 = np.array([[A, D/2, E/2],
                   [D/2, B, F/2],
                   [E/2, F/2, C]])
    L = np.array([G, H, I])

    # Center of ellipsoid
    b = -0.5 * np.linalg.inv(M3).dot(L)

    # Constant term at center
    c = J + b.dot(M3.dot(b))
    if c > 0:
        # Flip sign if needed
        params *= -1
        A, B, C, D, E, F, G, H, I, J = params
        M3 = np.array([[A, D/2, E/2],
                       [D/2, B, F/2],
                       [E/2, F/2, C]])
        L = np.array([G, H, I])
        b = -0.5 * np.linalg.inv(M3).dot(L)
        c = J + b.dot(M3.dot(b))

    # Scale matrix
    M3_scaled = M3 / -c
    eigvals, eigvecs = np.linalg.eigh(M3_scaled)
    scales = 1.0 / np.sqrt(eigvals)

    return b, eigvecs, scales

def main():
    parser = argparse.ArgumentParser(description="Ellipsoid calibrator from TXT logs")
    parser.add_argument("txt_file", help="Path to log txt file")
    args = parser.parse_args()

    ax, ay, az = parse_txt(args.txt_file)
    if ax.size < 10:
        print("Not enough valid data points parsed.")
        return

    b, Q, scales = fit_ellipsoid(ax, ay, az)

    print("Ellipsoid calibration results:")
    print(f"Center bias b = [{b[0]:.6f}, {b[1]:.6f}, {b[2]:.6f}]")
    print("Eigenvectors Q (columns):")
    for i in range(3):
        print(f"  {Q[:, i]}")
    print("Scale factors:")
    print(f"  scale_x = {scales[0]:.6f}")
    print(f"  scale_y = {scales[1]:.6f}")
    print(f"  scale_z = {scales[2]:.6f}")

if __name__ == "__main__":
    main()
