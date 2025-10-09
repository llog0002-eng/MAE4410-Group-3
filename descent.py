def descent(aircraft):
    # Input: aircraft object with all relevant variables
    # Output: property dataframe

    import numpy as np
    import pandas as pd
    from ISA import ISA
    from ISA_rho import ISA_rho

    g = 9.80665                                     # Gravitational acceleration m2/s
    rho0 = 1.225                                    # Sea level air density, kg/m3
    
    S = aircraft.S                                  # Wing area, m2
    Tmax = aircraft.Tmax                            # Max continuous thrust, N
    V = aircraft.Vmax                               # Cruise speed requirement, m/s
    K = aircraft.KC                                 # Induced drag constant
    CD0 = aircraft.CD0_C                            # Zero lift drag coefficient
    CL0 = aircraft.CL0_C                            # Zero angle of attack lift coefficient
    CLalpha = aircraft.CLalpha_C                    # Lift curve slope, per rad
    
    hdot = aircraft.descent_hdot                    # Rate of descent, m/s
    gamma = aircraft.descent_gamma * np.pi/180      # Flight path angle, deg

    n = 500 # Number of distance steps
    t_descent = aircraft.altitude/hdot # Descent time, s

    # Preallocate arrays
    throttles = np.zeros(n)
    alpha = np.zeros(n)
    m=np.zeros(n)
    Vs=np.zeros(n)
    LDs=np.zeros(n)
    ts = np.linspace(0,t_descent,n)
    dist = np.zeros(n)

    for i in range(n):
        break
