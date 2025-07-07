import numpy as np
import matplotlib.pyplot as plt

# --- Given values ---
rmin = 0.7
rmax = 0.9
v = 0.11
c = -0.7
cutoff_y = -0.2
x_min, x_max = -0.1, 1.2

# --- Solve for a and b ---
rhs1 = v * rmin - c
rhs2 = v * rmax - c

A = np.array([
    [1 / rmin, rmin],
    [1 / rmax, rmax]
])
b_vec = np.array([rhs1, rhs2])
a_val, b_val = np.linalg.solve(A, b_vec)

print(f"Solved coefficients:")
print(f"a = {a_val:.6f}")
print(f"b = {b_val:.6f}")
print(f"c = {c}")

# --- Define g(r) ---
def g(r):
    return a_val / r + b_val * r + c

# --- Define (1/r)·dg/dr ---
def dg_dr_over_r(r):
    return -a_val / r**3 + b_val / r

# --- Domain and evaluations ---
r_vals = np.linspace(x_min, x_max, 500)
g_vals_full = g(r_vals)
line_vals = v * r_vals
cutoff_line = np.full_like(r_vals, cutoff_y)

# --- Apply clipping mask ---
mask_y = g_vals_full >= cutoff_y
mask_band = (line_vals >= cutoff_y)

g_vals = np.where(mask_y, g_vals_full, np.nan)
line_vals_shaded = np.where(mask_band, line_vals, np.nan)

# --- Plot ---
plt.figure(figsize=(5, 5))
plt.plot(r_vals, g_vals, label="g(r) = a/r + br + c", linewidth=2)
plt.plot(r_vals, line_vals, 'r--', label=f"y = +{v}·r", linewidth=1.2)

# Shaded region between y = vr and y = cutoff
plt.fill_between(r_vals, line_vals_shaded, cutoff_line,
                 where=~np.isnan(line_vals_shaded),
                 color='red', alpha=0.1, label="Region below y = +vr")

# Constraint points
y1 = v * rmin
y2 = v * rmax
if x_min <= rmin <= x_max and y1 >= cutoff_y:
    plt.scatter(rmin, y1, color='black', zorder=5)
if x_min <= rmax <= x_max and y2 >= cutoff_y:
    plt.scatter(rmax, y2, color='black', zorder=5)

# Formatting
plt.title("Clipped g(r) = a/r + br + c", fontsize=11)
plt.xlabel("r")
plt.ylabel("g(r)")
plt.grid(True, linestyle='--', linewidth=0.5)
plt.legend(fontsize=9)
plt.axis('equal')
plt.xlim(x_min, x_max)
plt.ylim(cutoff_y, max(line_vals) + 0.5)
plt.tight_layout()
plt.show()

# --- Print final expression ---
def print_final_expression(a, b):
    print("\nFinal expression:")
    print(f"w = -({a:.6f}) / r**3 + ({b:.6f}) / r")

print_final_expression(a_val, b_val)

# --- Sample evaluations ---
r_test = np.array([0.7, 0.8, 0.9])
w_vals = dg_dr_over_r(r_test)

print("\nSample evaluations:")
for r, val in zip(r_test, w_vals):
    print(f"r = {r:.2f} | (1/r)·dg/dr = {val:.6f}")
