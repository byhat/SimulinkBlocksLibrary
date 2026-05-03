сделай чтобы для тестов можно было извне задавать опорные высоты (TARGET_ALTITUDE) тесты нужно проводить для нескольких высот (от 200 до 2000 с шагом в 200) и в имени файла с тестом должно указываться значение опорной высоты при которой он был проведён#!/usr/bin/env python3
"""
Plot FlightGear test data from Excel files.

This script finds all Excel files matching the pattern "test_*.xlsx" in the
current directory and creates two separate plots for each file:
- Longitudinal Channel Plot: Altitude, Pitch, Theta, Elevator, Throttle, etc.
- Lateral Channel Plot: Roll, Phi, Aileron, Yaw, Psi, Rudder, Heading, etc.
"""

import pandas as pd
import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend
import matplotlib.pyplot as plt
import os
import glob
import numpy as np


# Define Russian labels with units for all parameters
RUSSIAN_LABELS = {
    # Longitudinal Channel
    'altitude': 'Высота (м)',
    'theta': 'Тангаж (град)',
    'thetadot': 'Угловая скорость тангажа (град/с)',
    'vcas': 'Приборная скорость (м/с)',
    'alpha': 'Угол атаки (град)',
    'elevator': 'Руль высоты',
    'throttle_0': 'Дроссель',
    
    # Lateral Channel
    'phi': 'Крен (град)',
    'psi': 'Курс (град)',
    'phidot': 'Угловая скорость крена (град/с)',
    'psidot': 'Угловая скорость рыскания (град/с)',
    'beta': 'Угол скольжения (град)',
    'aileron': 'Элероны',
    'rudder': 'Руль направления',
}

# Define longitudinal channel parameters
# Longitudinal motion involves motion in the pitch plane (X-Z plane)
LONGITUDINAL_PARAMS = [
    'altitude',      # Altitude above sea level
    'theta',         # Pitch angle
    'thetadot',      # Pitch rate
    'vcas',          # Calibrated airspeed
    'alpha',         # Angle of attack
    'elevator',      # Elevator control input
    'throttle_0',    # Throttle control input
]

# Define lateral channel parameters
# Lateral motion involves motion in the roll-yaw plane (Y-axis and rotation about X and Z)
LATERAL_PARAMS = [
    'phi',           # Roll angle
    'psi',           # Yaw/heading angle
    'phidot',        # Roll rate
    'psidot',        # Yaw rate
    'beta',          # Sideslip angle
    'aileron',       # Aileron control input
    'rudder',        # Rudder control input
]


def plot_channel(excel_path, channel_name, channel_params, time_column, df):
    """
    Create a plot for a specific channel (longitudinal or lateral).
    
    Args:
        excel_path: Path to the Excel file
        channel_name: Name of the channel ('Longitudinal' or 'Lateral')
        channel_params: List of parameter names for this channel
        time_column: Name of the time column
        df: DataFrame containing the data
    """
    # Filter parameters that exist in the DataFrame
    available_params = [p for p in channel_params if p in df.columns]
    
    if not available_params:
        print(f"  Warning: No {channel_name.lower()} parameters found in file. Skipping {channel_name.lower()} plot.")
        return
    
    # Extract test name from filename
    filename = os.path.basename(excel_path)
    test_name = filename.replace('test_', '').replace('.xlsx', '')
    
    # Create subplots - one for each parameter, stacked vertically
    num_params = len(available_params)
    fig, axes = plt.subplots(num_params, 1, figsize=(12, 3 * num_params), sharex=True)
    
    # Handle single subplot case
    if num_params == 1:
        axes = [axes]
    
    # Define angle parameters that need conversion from radians to degrees
    ANGLE_PARAMS = {'theta', 'thetadot', 'phi', 'phidot', 'psi', 'psidot', 'alpha', 'beta'}
    
    # Plot each parameter
    for idx, param in enumerate(available_params):
        ax = axes[idx]
        # Convert angle parameters from radians to degrees
        if param in ANGLE_PARAMS:
            y_data = df[param] * (180.0 / np.pi)
        else:
            y_data = df[param]
        ax.plot(df[time_column], y_data, linewidth=2)
        # Use Russian label if available, otherwise use parameter name
        label = RUSSIAN_LABELS.get(param, param)
        ax.set_ylabel(label, fontsize=10)
        ax.grid(True, linestyle='--', alpha=0.7)
        ax.legend([label], loc='upper right')
    
    # Set common x-axis label
    axes[-1].set_xlabel('Time (s)', fontsize=12)
    
    # Set overall title with test name and channel
    fig.suptitle(f'Test: {test_name} - {channel_name} Channel', fontsize=16, y=0.995)
    
    # Adjust layout to prevent overlap
    plt.tight_layout(rect=[0, 0, 1, 0.99])
    
    # Save the plot as PNG with channel suffix
    base_path = excel_path.replace('.xlsx', '')
    output_path = f"{base_path}_{channel_name.lower()}.png"
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"  Saved {channel_name.lower()} plot to: {output_path}")
    
    # Close the figure to free memory
    plt.close(fig)


def plot_excel_file(excel_path):
    """
    Read an Excel file and create two plots: longitudinal and lateral channels.
    
    Args:
        excel_path: Path to the Excel file
    """
    # Extract test name from filename (remove 'test_' prefix and '.xlsx' extension)
    filename = os.path.basename(excel_path)
    test_name = filename.replace('test_', '').replace('.xlsx', '')
    
    print(f"Processing {filename}...")
    
    # Read the Excel file
    df = pd.read_excel(excel_path)
    
    # Get column names
    columns = df.columns.tolist()
    
    if len(columns) < 2:
        print(f"  Warning: {filename} has only {len(columns)} column(s). Skipping.")
        return
    
    # First column is time
    time_column = columns[0]
    
    # Create longitudinal channel plot
    plot_channel(excel_path, 'Longitudinal', LONGITUDINAL_PARAMS, time_column, df)
    
    # Create lateral channel plot
    plot_channel(excel_path, 'Lateral', LATERAL_PARAMS, time_column, df)


def main():
    """Main function to find and process all test Excel files."""
    # Find all Excel files matching the pattern "test_*.xlsx"
    pattern = "test_*.xlsx"
    excel_files = glob.glob(pattern)
    
    if not excel_files:
        print(f"No Excel files matching '{pattern}' found in the current directory.")
        return
    
    print(f"Found {len(excel_files)} test file(s):")
    for f in excel_files:
        print(f"  - {f}")
    print()
    
    # Process each Excel file
    for excel_file in excel_files:
        plot_excel_file(excel_file)
    
    print("\nDone! All plots have been created.")


if __name__ == "__main__":
    main()
