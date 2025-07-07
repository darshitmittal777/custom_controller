import numpy as np
import matplotlib.pyplot as plt

# --- Given values ---
rmin = 0.7
rmax = 0.9
v = 0.11
c = 0.5
cutoff_y = -0.2
x_min, x_max = -0.1, 1.2  # Match sine plot domain

# Constraint values
y1 = v * rmin
y2 = v * rmax

# --- Solve for a, b ---
rhs1 = y1 - c
rhs2 = y2 - c
A = np.array([
    [rmin**3, rmin],
    [rmax**3, rmax]
])
b_vec = np.array([rhs1, rhs2])
a_val, b_val = np.linalg.solve(A, b_vec)

print(f"Solved coefficients:\na = {a_val}\nb = {b_val}\nc = {c}")

# --- Define g(r) ---
def g(r): return a_val * r**3 + b_val * r + c

# --- Domain and function values ---
r_vals = np.linspace(x_min, x_max, 500)
g_vals_full = g(r_vals)
line_vals = v * r_vals
cutoff_line = np.full_like(r_vals, cutoff_y)

# --- Apply cutoff ---
mask_y = g_vals_full >= cutoff_y
mask_band = line_vals >= cutoff_y

g_vals = np.where(mask_y, g_vals_full, np.nan)
line_vals_shaded = np.where(mask_band, line_vals, np.nan)

# --- Plot ---
plt.figure(figsize=(5, 5))
plt.plot(r_vals, g_vals, label="g(r) = ar³ + br + c", linewidth=2)
plt.plot(r_vals, line_vals, 'r--', linewidth=1.2, label=f"y = +{v}·r")

# Shade only valid region above cutoff
plt.fill_between(r_vals, line_vals_shaded, cutoff_line,
                 where=~np.isnan(line_vals_shaded),
                 color='red', alpha=0.1, label="Region below y = +vr")

# Constraint points (if within view and above cutoff)
if x_min <= rmin <= x_max and y1 >= cutoff_y:
    plt.scatter(rmin, y1, color='black', zorder=5)
if x_min <= rmax <= x_max and y2 >= cutoff_y:
    plt.scatter(rmax, y2, color='black', zorder=5)

# --- Formatting ---
plt.title("Clipped Cubic g(r) = ar³ + br + c", fontsize=11)
plt.xlabel("r")
plt.ylabel("g(r)")
plt.grid(True, linestyle='--', linewidth=0.5)
plt.legend(fontsize=9)

plt.axis('equal')
plt.xlim(x_min, x_max)
plt.ylim(cutoff_y, max(line_vals) + 0.2)

plt.tight_layout()
plt.show()
