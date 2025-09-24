def ISA(hinput):
    # Input: altitude in meters
    # Output: temperature (K), pressure (Pa), relative density

    import pandas as pd
    import numpy as np

    # Import ISA data
    ISA_table = pd.read_csv('ISA.csv',header=0)
    h = ISA_table["Elevation (m)"].to_numpy()
    T = ISA_table["Temperature (K)"].to_numpy()
    P = ISA_table["Pressure (bar)"].to_numpy()
    rhorho0 = ISA_table["Relative density"].to_numpy()
    c = ISA_table["Speed of sound (m/s)"].to_numpy()

    # Interpolate to find values at input altitude
    T_h = np.interp(hinput, h, T)
    P_h = np.interp(hinput, h, P)
    rhorho0_h = np.interp(hinput, h, rhorho0)
    c_h = np.interp(hinput, h, c)
    
    return T_h, P_h, rhorho0_h, c_h