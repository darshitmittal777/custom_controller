import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import fsolve

# --- Given values ---
rmin = 0.7
rmax = 0.9
v = 0.11
c = 12
cutoff_y = -0.2
x_min, x_max = -0.1, 1.2

# --- Offset to lift sine above lower bound ---
offset = (rmax + rmin + (rmax - rmin)) * v / 2

# Target values
y1 = v * rmin
y2 = v * rmax

# --- Solve for a and b ---
def equations(vars):
    a, b = vars
    eq1 = a * np.sin(c * rmin + b) + offset - y1
    eq2 = a * np.sin(c * rmax + b) + offset - y2
    return [eq1, eq2]

initial_guess = [1.0, 0.0]
a_val, b_val = fsolve(equations, initial_guess)

print(f"Solved coefficients:\na = {a_val:.6f}\nb = {b_val:.6f}\nc = {c}\noffset = {offset:.6f}")

# --- Define g(r) ---
def g(r):
    return a_val * np.sin(c * r + b_val) + offset

# --- Define (1/r)·dg/dr ---
def dg_dr_over_r(r):
    return (a_val * c / r) * np.cos(c * r + b_val)

# --- Domain and values ---
r_vals = np.linspace(x_min, x_max, 500)
g_vals_full = g(r_vals)
line_vals = v * r_vals
cutoff_line = np.full_like(r_vals, cutoff_y)

mask_y = g_vals_full >= cutoff_y
mask_band = (line_vals >= cutoff_y)
g_vals = np.where(mask_y, g_vals_full, np.nan)
line_vals_shaded = np.where(mask_band, line_vals, np.nan)

# --- Plot ---
plt.figure(figsize=(5, 5))
plt.plot(r_vals, g_vals, label="g(r) = a·sin(cr + b) + offset", linewidth=2)
plt.plot(r_vals, line_vals, 'r--', linewidth=1.2, label=f"y = +{v}·r")
plt.fill_between(r_vals, line_vals_shaded, cutoff_line,
                 where=~np.isnan(line_vals_shaded),
                 color='red', alpha=0.1, label="Region below y = +vr")

if x_min <= rmin <= x_max and y1 >= cutoff_y:
    plt.scatter(rmin, y1, color='black', zorder=5)
if x_min <= rmax <= x_max and y2 >= cutoff_y:
    plt.scatter(rmax, y2, color='black', zorder=5)

plt.title("g(r) = a·sin(cr + b) + offset", fontsize=11)
plt.xlabel("r")
plt.ylabel("g(r)")
plt.grid(True, linestyle='--', linewidth=0.5)
plt.legend(fontsize=9)
plt.axis('equal')
plt.xlim(x_min, x_max)
plt.ylim(cutoff_y, max(line_vals) + 0.2)
plt.tight_layout()
plt.show()

# --- Print final expression ---
def print_final_expression(a, b, c):
    print("\nFinal expression:")
    print(f"w = ({a:.6f} * {c}) / r * cos({c} * r + {b:.6f})")

print_final_expression(a_val, b_val, c)

# --- Sample evaluations ---
r_test = np.array([0.7, 0.8, 0.9])
w_vals = dg_dr_over_r(r_test)

print("\nSample evaluations:")
for r, val in zip(r_test, w_vals):
    print(f"r = {r:.2f} | (1/r)·dg/dr = {val:.6f}")
