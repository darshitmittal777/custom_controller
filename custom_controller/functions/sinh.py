import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import fsolve

# --- Given values ---
rmin = 0.7
rmax = 0.9
v = 0.11
c = 0.55
cutoff_y = -0.2
x_min, x_max = -0.1, 1.2

# Target values
y1 = v * rmin
y2 = v * rmax

# --- Solve for a and b in sinh(ar² + br + c) ---
def equations(vars):
    a, b = vars
    eq1 = np.sinh(a * rmin**2 + b * rmin + c) - y1
    eq2 = np.sinh(a * rmax**2 + b * rmax + c) - y2
    return [eq1, eq2]

initial_guess = [0.1, 0.1]
a_val, b_val = fsolve(equations, initial_guess)

print(f"Solved coefficients:")
print(f"a = {a_val:.6f}")
print(f"b = {b_val:.6f}")
print(f"c = {c}")

# --- Define g(r) ---
def g(r):
    return np.sinh(a_val * r**2 + b_val * r + c)

# --- Define (1/r)·dg/dr ---
def dg_dr_over_r(r):
    u = a_val * r**2 + b_val * r + c
    return (1 / r) * np.cosh(u) * (2 * a_val * r + b_val)

# --- Print symbolic expression ---
def print_final_expression(a, b, c):
    print("\nFinal expression:")
    print(f"w = (1 / r) * math.cosh({a:.6f} * r**2 + {b:.6f} * r + {c}) * (2 * {a:.6f} * r + {b:.6f})")

print_final_expression(a_val, b_val, c)

# --- Sample evaluations ---
r_test = np.array([0.7, 0.8, 0.9])
w_vals = dg_dr_over_r(r_test)

print("\nSample evaluations:")
for r, val in zip(r_test, w_vals):
    print(f"r = {r:.2f} | (1/r)·dg/dr = {val:.6f}")

# --- Plot ---
r_vals = np.linspace(x_min, x_max, 500)
g_vals_full = g(r_vals)
line_vals = v * r_vals
cutoff_line = np.full_like(r_vals, cutoff_y)

mask_y = g_vals_full >= cutoff_y
mask_band = (line_vals >= cutoff_y)
g_vals = np.where(mask_y, g_vals_full, np.nan)
line_vals_shaded = np.where(mask_band, line_vals, np.nan)

plt.figure(figsize=(5, 5))
plt.plot(r_vals, g_vals, label="g(r) = sinh(ar² + br + c)", linewidth=2)
plt.plot(r_vals, line_vals, 'r--', linewidth=1.2, label=f"y = {v}·r")

plt.fill_between(r_vals, line_vals_shaded, cutoff_line,
                 where=~np.isnan(line_vals_shaded),
                 color='red', alpha=0.1, label="Region below y = +vr")

if x_min <= rmin <= x_max and y1 >= cutoff_y:
    plt.scatter(rmin, y1, color='black', zorder=5)
if x_min <= rmax <= x_max and y2 >= cutoff_y:
    plt.scatter(rmax, y2, color='black', zorder=5)

plt.title("g(r) = sinh(ar² + br + c)", fontsize=11)
plt.xlabel("r")
plt.ylabel("g(r)")
plt.grid(True, linestyle='--', linewidth=0.5)
plt.legend(fontsize=9)
plt.axis('equal')
plt.xlim(x_min, x_max)
plt.ylim(cutoff_y, max(line_vals) + 0.2)
plt.tight_layout()
plt.show()
