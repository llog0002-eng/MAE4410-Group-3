def ISA_rho(rhoinput):
    # Input: density in kg/m3
    # Output: height in metres

    import pandas as pd
    import numpy as np

    rho0 = 1.225                                  # Sea level air density, kg/m3

    # Import ISA data
    ISA_table = pd.read_csv('ISA.csv',header=0)
    h = ISA_table["Elevation (m)"].to_numpy()
    rhorho0 = ISA_table["Relative density"].to_numpy()

    # Reverse arrays if density is in descending order
    if rhorho0[0] > rhorho0[-1]:
        h = h[::-1]
        rhorho0 = rhorho0[::-1]

    # Interpolate to find values at input density
    h_r = np.interp(rhoinput, rhorho0*rho0, h)

    if h_r > max(h):
        print("Warning: Input density is below minimum density in ISA table. Extrapolating.")
    
    return h_r