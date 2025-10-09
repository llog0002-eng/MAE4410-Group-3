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
    CD0 = aircraft.CD0_DE                            # Zero lift drag coefficient
    CL0 = aircraft.CL0_DE                            # Zero angle of attack lift coefficient
    CLalpha = aircraft.CLalpha_DE                    # Lift curve slope, per rad
    
    hdot = aircraft.descent_hdot                    # Rate of descent, m/s
    gamma = aircraft.descent_gamma * np.pi/180      # Flight path angle, deg
    V = hdot/np.sin(gamma)                          # Descent velocity, m/s

    n = 500 # Number of distance steps
    t_descent = aircraft.altitude/hdot # Descent time, s

    # Preallocate arrays
    throttles = np.zeros(n)
    alpha = np.zeros(n)
    m=np.zeros(n)
    Vs=np.ones(n)*V
    LDs=np.zeros(n)
    ts = np.linspace(0,t_descent,n)
    dist = np.zeros(n)
    hs = np.zeros(n)                                  # Altitude array, m
    thetas = np.zeros(n)

    hs[0] = aircraft.altitude

    for i in range(n):
        rho = rho
        # CL, assume small contribution from lift due to thrust
        CL = 2 * aircraft.W * g / (rho * aircraft.S * V**2)
        # CD
        CD = aircraft.CD0_DE + aircraft.KC * CL**2
        # LD
        LDs[i] = CL / CD
        # Angle of attack at climb
        alpha[i] = (CL - aircraft.CL0_DE) / (aircraft.CLalpha_DE)
        # Pitch angle
        thetas[i] = alpha[i]+gamma
