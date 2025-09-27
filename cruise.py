def cruise(aircraft):
    # Input: aircraft object with all relevant variables
    # Output: 

    import numpy as np
    from ISA import ISA
    from ISA_rho import ISA_rho
    g = 9.80665                                   # Gravitational acceleration m2/s
    rho0 = 1.225                                  # Sea level air density, kg/m3
    
    S = aircraft.S                                  # Wing area, m2
    Tmax = aircraft.Tmax_continuous                 # Max continuous thrust, N
    V = aircraft.Vmax                               # Cruise speed requirement, m/s
    K = aircraft.KC                                 # Induced drag constant
    CD0 = aircraft.CD0_C                            # Zero lift drag coefficient
    CL0 = aircraft.CL0_C                            # Zero angle of attack lift coefficient
    CLalpha = aircraft.CLalpha_C                    # Lift curve slope, per rad

    # Cruise lift coefficient, max L/D
    CL_LDmax = np.sqrt(CD0 / K)

    # Cruise angle of attack, max L/D, rad
    alpha_LDmax = (CL_LDmax - CL0) / (CLalpha)

    # Max L/D
    LD = 1/2*np.sqrt(1/(K*CD0))
    print('Max L/D: ', LD)
    
    n = 5000 # Number of distance steps in cruise simulation

    cruiseDist = np.linspace(0, aircraft.range, n)  # Cruise distance array, m
    cruiseW = np.zeros(n)                           # Weight array, kg
    cruiseW[0] = aircraft.W                         # Initial weight, kg
    cruiseLDs = np.zeros(n)                         # L/D array
    cruiseAlpha = np.zeros(n)                       # Angle of attack array, rad
    cruisets = np.zeros(n)                          # Time array, s
    cruiseh = np.zeros(n)                           # Altitude array, m
    cruiseV = np.zeros(n)                           # Velocity array, m/s
    cruiseopth = np.zeros(n)                        # Optimal altitude array, m
    cruiseThrottle = np.zeros(n)                    # Throttle setting array

    # Initial cruise density
    rhoi = 2 * aircraft.W * g / (np.sqrt(CD0/K) * S * V**2)
    # Initial cruise altitude
    cruiseh[0] = ISA_rho(rhoi)
    climbing = False

    for i in range(len(cruiseDist)):
        
        # Atmospheric properties at current altitude
        T, P, rhorho0, c = ISA(cruiseh[i])
        rho = rhorho0 * rho0
        V = aircraft.Ma * c
        cruiseV[i] = V

        CL = (2 * aircraft.W * g) / (rho * S * V**2)    # Lift coefficient, assuming level flight
        CD = CD0 + K * CL**2                            # Drag coefficient at max L/D

        # Target cruise air density, kg/m3
        rho_targ = 2 * aircraft.W * g / (np.sqrt(CD0/K) * S * V**2)

        # Target cruise altitude for given L/D, m
        h_targ_opt = ISA_rho(rho_targ)   
        cruiseopth[i] = h_targ_opt

        if climbing == False:
            h_targ = h_targ_opt

        dt = aircraft.range / (n-1) / V  # Time step, s
        cruisets[i] = i*dt

        # Airspeed performance correction
        a0d_cruise = 1 + aircraft.av * V/aircraft.c
        # Ratio of maximum static thrust or power at sea level to the thrust or power at the desired operating condition, cruise
        alphae_cruise = a0d_cruise * (rhorho0)**aircraft.a

        # If altitude error is more than half the climb increment, start climb
        if cruiseh[i] < h_targ - aircraft.cruise_climb_inc/2 or climbing == True:
            climbing = True
            # Climb rate, m/s
            hdot = aircraft.hdotceil
            # CL at climb, assume small contribution from lift due to thrust
            CL_C = 2 * aircraft.W * g / (rho * aircraft.S * V**2)
            # CD at climb
            CD_C = aircraft.CD0_C + aircraft.KC * CL_C**2
            # LD at climb
            cruiseLDs[i] = CL_C / CD_C
            # Angle of attack at climb
            cruiseAlpha[i] = (CL_C - aircraft.CL0_C) / (aircraft.CLalpha_C)

            # Update altitude
            if i < n-1:
                cruiseh[i+1] = cruiseh[i] + hdot * dt

            # If actual altitude is higher than the target altitude plus half the climb increment, stop climbing
            if cruiseh[i] >= h_targ + aircraft.cruise_climb_inc/2:
                climbing = False

        # Otherwise, stay level
        else:
            climbing = False
            # Zero climb rate
            hdot = 0
            # CL at steady level cruise, assume small contribution from lift due to thrust
            CL_C = 2 * aircraft.W * g / (rho * aircraft.S * V**2)
            # CD at steady level cruise
            CD_C = aircraft.CD0_C + aircraft.KC * CL_C**2
            # LD at steady level cruise
            cruiseLDs[i] = CL_C / CD_C
            # Angle of attack at steady level cruise
            cruiseAlpha[i] = (CL_C - aircraft.CL0_C) / (aircraft.CLalpha_C)

            # Update altitude
            if i < n-1:
                cruiseh[i+1] = cruiseh[i]

        # Solve for required thrust
        T = aircraft.W * g / cruiseLDs[i] * (1 + V / (aircraft.Ma * c) * hdot / V)

        # Solve for required throttle setting
        cruiseThrottle[i] = T / (alphae_cruise * aircraft.Tmax)

        # Update weight
        dm = aircraft.TSFC_C * T / 1e6 * dt  # Fuel flow, kg
        aircraft.W -= dm
        cruiseW[i] = aircraft.W

    return cruiseDist, cruiseW, cruiseLDs, cruiseAlpha, cruisets, cruiseh, cruiseV, cruiseopth, cruiseThrottle