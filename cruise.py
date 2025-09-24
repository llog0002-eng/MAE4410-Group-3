def cruise(aircraft):
    # Input: aircraft object with all relevant variables
    # Output: 

    import numpy as np
    g = 9.80665                                   # Gravitational acceleration m2/s
    rho0 = 1.225                                  # Sea level air density, kg/m3

    # Cruise lift coefficient, max L/D
    CL_LDmax = np.sqrt(aircraft.CD0_C / aircraft.KC)

    # Cruise angle of attack, max L/D, rad
    alpha_LDmax = (CL_LDmax - aircraft.CL0_C) / (aircraft.CLalpha_C)

    # Max L/D
    LDmax = CL_LDmax / (aircraft.CD0_C + aircraft.KC * CL_LDmax**2)
    
    # Airspeed performance correction
    a0d_cruise = aircraft.a0_cruise * (1 + aircraft.av * aircraft.Vmax/aircraft.c) 

    n = 1000 # Number of distance steps in cruise simulation
    dt = aircraft.range / (n-1) / aircraft.Vmax  # Time step, s

    cruisedist = np.linspace(0, aircraft.range, n)  # Cruise distance array, m

    for i in range(len(cruisedist)):
        # Altitude performance correction
        rho = 0
        rhorho0 = rho / rho0

        # Ratio of maximum static thrust or power at sea level to the thrust or power at the desired operating condition, cruise
        alphae_cruise = a0d_cruise * (rhorho0)**aircraft.a



    