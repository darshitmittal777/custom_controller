import numpy as np
import matplotlib.pyplot as plt

# --- Given values ---
rmin = 0.7
rmax = 0.9
v = 0.11
c = 1
cutoff_y = -0.2
x_min, x_max = -0.1, 1.2

# Constraint values
y1 = v * rmin
y2 = v * rmax

# --- Right-hand sides after log and subtracting c ---
rhs1 = np.log(y1) - c
rhs2 = np.log(y2) - c

# --- Solve for a, b ---
A = np.array([
    [rmin**2, rmin],
    [rmax**2, rmax]
])
b_vec = np.array([rhs1, rhs2])
a_val, b_val = np.linalg.solve(A, b_vec)

print(f"Solved coefficients:")
print(f"a = {a_val:.6f}")
print(f"b = {b_val:.6f}")
print(f"c = {c}")

# --- Define g(r) ---
def g(r):
    return np.exp(a_val * r**2 + b_val * r + c)

# --- Define 1/r·dg/dr ---
def dg_dr_over_r(r):
    return g(r) * (2 * a_val + b_val / r)

# --- Domain and values ---
r_vals = np.linspace(x_min, x_max, 500)
g_vals_full = g(r_vals)
line_vals = v * r_vals
cutoff_line = np.full_like(r_vals, cutoff_y)

# --- Apply clipping mask ---
mask_y = g_vals_full >= cutoff_y
mask_band = (line_vals >= cutoff_y)

g_vals = np.where(mask_y, g_vals_full, np.nan)
line_vals_shaded = np.where(mask_band, line_vals, np.nan)

# --- Plotting ---
plt.figure(figsize=(5, 5))
plt.plot(r_vals, g_vals, label="g(r) = exp(ar² + br + c)", linewidth=2)
plt.plot(r_vals, line_vals, 'r--', label=f"y = +{v}·r", linewidth=1.2)
plt.fill_between(r_vals, line_vals_shaded, cutoff_line,
                 where=~np.isnan(line_vals_shaded),
                 color='red', alpha=0.1, label="Region below y = +vr")
if x_min <= rmin <= x_max and y1 >= cutoff_y:
    plt.scatter(rmin, y1, color='black', zorder=5)
if x_min <= rmax <= x_max and y2 >= cutoff_y:
    plt.scatter(rmax, y2, color='black', zorder=5)
plt.title("Clipped Exponential g(r) = exp(ar² + br + c)", fontsize=11)
plt.xlabel("r")
plt.ylabel("g(r)")
plt.grid(True, linestyle='--', linewidth=0.5)
plt.legend(fontsize=9)
plt.axis('equal')
plt.xlim(x_min, x_max)
plt.ylim(cutoff_y, max(line_vals) + 0.5)
plt.tight_layout()
plt.show()

# --- Print symbolic expression with computed values ---
def print_final_expression(a, b, c):
    print("\nFinal expression:")
    print(f"w = np.exp({a:.6f} * r**2 + {b:.6f} * r + {c}) * (2 * {a:.6f} + {b:.6f} / r)")

print_final_expression(a_val, b_val, c)

# --- Sample evaluations ---
r_test = np.array([0.7, 0.8, 0.9])
g_eval = g(r_test)
dgdr_eval = dg_dr_over_r(r_test)

print("\nSample evaluations:")
for r, g_val, d_val in zip(r_test, g_eval, dgdr_eval):
    print(f"r = {r:.2f} | g(r) = {g_val:.6f} | (1/r)·dg/dr = {d_val:.6f}")
