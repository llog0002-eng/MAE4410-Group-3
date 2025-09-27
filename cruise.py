def cruise(aircraft):
    # Input: aircraft object with all relevant variables
    # Output: 

    import numpy as np
    from ISA import ISA
    g = 9.80665                                   # Gravitational acceleration m2/s
    rho0 = 1.225                                  # Sea level air density, kg/m3

    # Cruise lift coefficient, max L/D
    CL_LDmax = np.sqrt(aircraft.CD0_C / aircraft.KC)

    # Cruise angle of attack, max L/D, rad
    alpha_LDmax = (CL_LDmax - aircraft.CL0_C) / (aircraft.CLalpha_C)

    # Max L/D
    LDmax = CL_LDmax / (aircraft.CD0_C + aircraft.KC * CL_LDmax**2)
    
    n = 1000 # Number of distance steps in cruise simulation
    h = 10668  # Initial altitude, m 
    # !!! -> this should end up being more like 16500 m

    cruiseDist = np.linspace(0, aircraft.range, n)  # Cruise distance array, m
    cruiseW = np.zeros(n)                           # Weight array, kg
    cruiseW[0] = aircraft.W                         # Initial weight, kg
    cruiseLDs = np.zeros(n)                         # L/D array
    cruiseAlpha = np.zeros(n)                       # Angle of attack array, rad
    cruisets = np.zeros(n)                          # Time array, s

    for i in range(len(cruiseDist)):
        # Atmospheric properties at current altitude
        T, P, rhorho0, c = ISA(h)
        rho = rhorho0 * rho0
        V = aircraft.Ma * c

        dt = aircraft.range / (n-1) / V  # Time step, s
        cruisets[i] = i*dt

        # Airspeed performance correction
        a0d_cruise = 1 + aircraft.av * V/aircraft.c
        # Ratio of maximum static thrust or power at sea level to the thrust or power at the desired operating condition, cruise
        alphae_cruise = a0d_cruise * (rhorho0)**aircraft.a

        # CL at steady level cruise, assume small contribution from lift due to thrust
        CL_C = 2 * aircraft.W / (rho * aircraft.S * V**2)
        # CD at steady level cruise
        CD_C = aircraft.CD0_C + aircraft.KC * CL_C**2
        # LD at steady level cruise
        cruiseLDs[i] = CL_C / CD_C
        # Angle of attack at steady level cruise
        cruiseAlpha[i] = (CL_C - aircraft.CL0_C) / (aircraft.CLalpha_C)

        # Solve for required thrust
        T = 1/2 * rho * V**2 * CD_C * aircraft.S

        # Solve for required throttle setting
        a0_cruise = T / (alphae_cruise * aircraft.Tmax)

        # Update weight
        dm = aircraft.TSFC_C * T / 1e6 * dt  # Fuel flow, kg
        aircraft.W -= dm
        cruiseW[i] = aircraft.W

    return cruiseDist, cruiseW, cruiseLDs, cruiseAlpha