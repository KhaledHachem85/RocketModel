import pandas as pd
import numpy as np
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt

# 1. Load data
df = pd.read_csv('drag.csv')

# 2. Strict Filtering: Mach 0.1 to 1.75
# This removes the low-speed noise and the post-1.75 drop to -10
df = df[(df['Mach'] >= 0.1) & (df['Mach'] <= 1.75) & (df['Calculated_Cd'] > 0)]

# 3. Prepare for Interpolation: Average duplicates and sort
df = df.groupby('Mach')['Calculated_Cd'].mean().reset_index()
df = df.sort_values('Mach')

# 4. Create the interpolation function
# We use 'linear' to ensure C-code compatibility
cd_function = interp1d(df['Mach'], df['Calculated_Cd'], kind='linear', fill_value="extrapolate")

# 5. Generate Test Range strictly from 0.1 to 1.75
test_mach_range = np.arange(0.2, 1.6, 0.01)
test_cd_values = cd_function(test_mach_range)

# 6. Graphing
plt.figure(figsize=(12, 6))

# Plot the clean data points
plt.scatter(df['Mach'], df['Calculated_Cd'], color='red', s=15, alpha=0.6, label='Cleaned Data (0.1 - 1.75M)')

# Plot the interpolated line
plt.plot(test_mach_range, test_cd_values, color='blue', linewidth=2, label='Linear Interpolation')

plt.title('Drag Coefficient: Mach 0.1 to 1.75 (Cleaned)')
plt.xlabel('Mach Number')
plt.ylabel('$C_d$')
plt.xlim(0.1, 1.75)  # Set graph boundaries strictly
plt.grid(True, which='both', linestyle='--', alpha=0.5)
plt.legend()

plt.show()

# 7. Export for C
# This header will now ONLY contain the valid physical data
with open('drag_data.h', 'w') as f:
    f.write(f"#define DATA_POINTS {len(df)}\n\n")
    f.write("static const double MACH_TABLE[] = {" + ", ".join(map(str, df['Mach'])) + "};\n\n")
    f.write("static const double CD_TABLE[] = {" + ", ".join(map(str, df['Calculated_Cd'])) + "};\n")

print(f"Graph generated. drag_data.h now contains {len(df)} clean points.")