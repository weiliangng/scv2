from sympy import symbols, Eq, solve, N

# Define symbol for R1 (we'll express R2 as 1000 - R1)
R1 = symbols('R1', real=True, positive=True)
R_total = 1000  # Total resistance in ohms
V_in = 1.65     # Input voltage in volts
V_out = 1.205   # Desired output voltage

# R2 is the remainder of the total resistance
R2 = R_total - R1

# Voltage divider equation
eq = Eq(V_out, V_in * (R2 / (R1 + R2)))

# Solve for R1
R1_solution = solve(eq, R1)[0]
R2_solution = R_total - R1_solution

# Return both R1 and R2
N(R1_solution), N(R2_solution)
