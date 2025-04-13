import sympy
from sympy import symbols, cos, sin, diff, simplify

# --- 1) Define symbolic variables ---
phi1, phi2, phi3, phi4, phi5 = symbols('phi1 phi2 phi3 phi4 phi5', real=True)
dotphi = sympy.Function('dotphi')(phi1)  # not used this way; we'll just keep velocities symbolic
m, l = symbols('m l', positive=True)

# For convenience, define cos(φ_i - φ_j) short-hands:
C12 = cos(phi1 - phi2)
C13 = cos(phi1 - phi3)
C14 = cos(phi1 - phi4)
C15 = cos(phi1 - phi5)
C23 = cos(phi2 - phi3)
C24 = cos(phi2 - phi4)
C25 = cos(phi2 - phi5)
C34 = cos(phi3 - phi4)
C35 = cos(phi3 - phi5)
C45 = cos(phi4 - phi5)

# --- 2) Build M(phi) explicitly as a 5x5 Sympy Matrix ---
# Diagonal terms (constants wrt angles):
M11 = (13/sympy.Integer(3))*m*l**2
M22 = (10/sympy.Integer(3))*m*l**2
M33 = (7/sympy.Integer(3))*m*l**2
M44 = (4/sympy.Integer(3))*m*l**2
M55 = (1/sympy.Integer(3))*m*l**2

# Off-diagonal with cos(...) factors:
M12 = (7/sympy.Integer(2))*m*l**2*C12
M13 = (5/sympy.Integer(2))*m*l**2*C13
M14 = 2*m*l**2*C14
M15 = 1*m*l**2*C15
M23 = 3*m*l**2*C23
M24 = 2*m*l**2*C24
M25 = 1*m*l**2*C25
M34 = 2*m*l**2*C34
M35 = 1*m*l**2*C35
M45 = 1*m*l**2*C45

M = sympy.Matrix([
 [M11,  M12,  M13,  M14,  M15],
 [M12,  M22,  M23,  M24,  M25],
 [M13,  M23,  M33,  M34,  M35],
 [M14,  M24,  M34,  M44,  M45],
 [M15,  M25,  M35,  M45,  M55],
])

# --- 3) Build the Coriolis matrix C(phi, dotphi). ---
# Formula: C_{k j} = 1/2 sum_{i=1..5}[ dM_{k j}/dphi_i + dM_{k i}/dphi_j - dM_{i j}/dphi_k ] * dotphi_i
# We'll create symbolic placeholders for dotphi_1..dotphi_5 so the final is linear in them.
dotphi_syms = sympy.symbols('dphi1 dphi2 dphi3 dphi4 dphi5', real=True)

# We'll build a 5x5 matrix of expressions that are linear in dphi1..dphi5.
kmax = 5
C_matrix = sympy.zeros(kmax, kmax)

phi_list = [phi1, phi2, phi3, phi4, phi5]

for k in range(kmax):
    for j in range(kmax):
        # Build the expression sum_i of that bracket * dotphi_i
        # inside the bracket is (1/2)*(dM_{k j}/dphi_i + dM_{k i}/dphi_j - dM_{i j}/dphi_k)
        expr_kj = 0
        for i in range(kmax):
            dM_kj_dphii = diff(M[k,j], phi_list[i])
            dM_ki_dphij = diff(M[k,i], phi_list[j])
            dM_ij_dphik = diff(M[i,j], phi_list[k])
            bracket = dM_kj_dphii + dM_ki_dphij - dM_ij_dphik
            expr_kj += sympy.Rational(1,2)*bracket*dotphi_syms[i]
        # store that in C_{k j}:
        C_matrix[k,j] = simplify(expr_kj)

# Print results in a decently readable form:
print("C(phi) = ")
sympy.pprint(C_matrix)
