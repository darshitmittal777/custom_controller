import numpy as np

# --- Given values ---
rmin = 0.7
rmax = 0.9
v = 0.11
c = 0.1

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

# --- Define g(r) and 1/r·dg/dr ---
def g(r):
    return a_val * r**3 + b_val * r + c

def dg_dr_over_r(r):
    return 3 * a_val * r + b_val / r

# --- Print final symbolic form with numbers ---
print("Final expression for g(r):")
print(f"g(r) = {a_val:.4f}·r³ + {b_val:.4f}·r + {c}")

print("\nFinal expression for (1/r)·dg/dr:")
print(f"(1/r)·dg/dr = {3*a_val:.4f}·r + {b_val:.4f}/r")

# Optional: Test evaluation
r_test = np.array([0.7, 0.8, 0.9])
g_vals = g(r_test)
dgdr_vals = dg_dr_over_r(r_test)

print("\nSample evaluations:")
for r, gval, dval in zip(r_test, g_vals, dgdr_vals):
    print(f"r = {r:.1f} | g(r) = {gval:.4f} | (1/r)·dg/dr = {dval:.4f}")
