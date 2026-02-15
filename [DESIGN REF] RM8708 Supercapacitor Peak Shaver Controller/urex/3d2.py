import numpy as np
import matplotlib.pyplot as plt

# Define I_LOAD range
i_load = np.linspace(0, 15, 200)

# I_SET values from 0A to 5A → V(I_SET) from 2.5V to 1.75V
i_set_values = np.linspace(0, 5, 5)
v_i_set_values = 2.5 - (i_set_values / 6.6667)

# Plot I_BUFFER and I_BATTERY vs I_LOAD for each V(I_SET)
plt.figure(figsize=(10, 6))
for i_set, v in zip(i_set_values, v_i_set_values):
    i_buffer = 6.6667 * (0.15 * i_load + v) - 16.6667
    i_buffer_clipped = np.minimum(i_buffer, 10)  # Apply 10A ceiling
    i_buffer_flipped = -i_buffer_clipped  # Flip about x-axis
    i_battery = i_buffer_flipped + i_load  # I_BATTERY = I_BUFFER + I_LOAD → flipped version

    plt.plot(i_load, i_buffer_flipped, label=f'I_BUFFER, I_SET = {i_set:.2f} A')
    plt.plot(i_load, i_battery, '--', label=f'I_BATTERY, I_SET = {i_set:.2f} A')

# Labels and legend
plt.xlabel('I_LOAD (A)')
plt.ylabel('Current (A)')
plt.title('I_BUFFER and I_BATTERY vs I_LOAD')
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()
