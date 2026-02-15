from sympy import symbols, Eq, solve, N

# Define variables
R1, R2 = symbols('R1 R2', real=True, positive=True)

# Constants
Vin = 26        # Input voltage in volts
Vout = 1.209     # Desired midpoint voltage
Ibias = 20e-6    # Bias current in amps (20 μA)

def compute_R1_given_R2(R2_val):
    # Define the modified voltage divider equation
    eq = Eq(Vout, (Vin * R2 / (R1 + R2)) + Ibias * R2)
    # Solve for R1
    sol = solve(eq.subs(R2, R2_val), R1)
    return N(sol[0]) if sol else None

# Example usage
user_R2 = 7680  # Replace with any value, e.g., float(input("Enter R2 in ohms: "))
R1_value = compute_R1_given_R2(user_R2)

print(f"Given R2 = {user_R2/1000:.2f}kΩ, the required R1 ≈ {R1_value/1000:.2f}kΩ")
