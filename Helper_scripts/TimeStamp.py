import pandas as pd
import numpy as np

# Load the CSV file into a DataFrame



def Time_Stamp(path,timeinterval):


    df = pd.read_csv(path)

    # Define a constant time interval between points (adjust as needed)
    time_interval = timeinterval  # seconds

    # Calculate timestamps based on the constant time interval
    timestamps = np.arange(0, len(df) * time_interval, time_interval)

    # Add timestamps to the DataFrame
    df['Timestamp'] = timestamps


    # Save the DataFrame with timestamps to a new CSV file
    df.to_csv('trajectory_with_timestamps.csv', index=False)
    return print("Trajectory with Timestamps:"), print(df)


path = '/home/pawan/Thesis/F1env/src/f1tenth_simulator/scripts/Additional_Maps/f1tenth_racetracks/MexicoCity/MexicoCity_centerline.csv'
Time_Stamp(path,timeinterval = 0.02)