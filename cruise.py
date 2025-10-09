def cruise(aircraft):
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
    TSFC = aircraft.TSFC_C/1e6                      # TSFC, kg/(N.s)

    # Cruise lift coefficient, max L/D
    CL_LDmax = np.sqrt(CD0 / K)

    # Cruise angle of attack, max L/D, rad
    alpha_LDmax = (CL_LDmax - CL0) / (CLalpha)
    print('Cruise AoA (deg): ', round(alpha_LDmax*180/np.pi,2))

    # Max L/D
    LD = 1/2*np.sqrt(1/(K*CD0))
    print('Max L/D: ', round(LD,2))
    
    n = 1000 # Number of distance steps in cruise simulation

    cruise_dist = aircraft.range

    dist = np.linspace(0, cruise_dist, n)             # Cruise distance array, m
    m = np.zeros(n)                                   # Mass array, kg
    m[0] = aircraft.W                                 # Initial mass, kg
    LDs = np.zeros(n)                                 # L/D array
    alpha = np.zeros(n)                               # Angle of attack array, rad
    ts = np.zeros(n)                                  # Time array, s
    hs = np.zeros(n)                                  # Altitude array, m
    Vs = np.zeros(n)                                  # Velocity array, m/s
    opths = np.zeros(n)                               # Optimal altitude array, m
    throttles = np.zeros(n)                           # Throttle setting array
    thetas = np.zeros(n)                              # Pitch angle array
    
    # Initial cruise density
    rhoi = 2 * aircraft.W * g / (np.sqrt(CD0/K) * S * V**2)
    # Initial cruise altitude
    hs[0] = ISA_rho(rhoi)
    climbing = False

    for i in range(n):
        
        # Atmospheric properties at current altitude
        T, P, rhorho0, c = ISA(hs[i])
        rho = rhorho0 * rho0
        V = aircraft.Ma * c
        Vs[i] = V

        CL = (2 * aircraft.W * g) / (rho * S * V**2)    # Lift coefficient, assuming level flight
        CD = CD0 + K * CL**2                            # Drag coefficient at max L/D

        # Target cruise air density, kg/m3
        rho_targ = 2 * aircraft.W * g / (np.sqrt(CD0/K) * S * V**2)

        # Target cruise altitude for given L/D, m
        h_targ_opt = ISA_rho(rho_targ)   
        opths[i] = h_targ_opt

        # If not climbing, update altitude target
        if climbing == False:
            h_targ = h_targ_opt

        dt = aircraft.range / (n-1) / V  # Time step, s
        if i != 0:
            ts[i] = ts[i-1] + dt

        # Airspeed performance correction
        a0d_cruise = 1 + aircraft.av * V/c
        # Ratio of maximum static thrust or power at sea level to the thrust or power at the desired operating condition, cruise
        alphae_cruise = a0d_cruise * (rhorho0)**aircraft.a

        # If altitude error is more than half the climb increment, start climb
        if hs[i] < h_targ - aircraft.cruise_climb_inc/2 or climbing == True:
            climbing = True
            # Climb rate, m/s
            hdot = aircraft.hdotceil
            # Flight path angle
            gamma = np.arcsin(aircraft.hdotceil/V)
            # CL at climb, assume small contribution from lift due to thrust
            CL_C = 2 * aircraft.W * g / (rho * aircraft.S * V**2) / np.cos(gamma)
            # CD at climb
            CD_C = aircraft.CD0_C + aircraft.KC * CL_C**2
            # LD at climb
            LDs[i] = CL_C / CD_C
            # Angle of attack at climb
            alpha[i] = (CL_C - aircraft.CL0_C) / (aircraft.CLalpha_C)
            # Pitch angle
            thetas[i] = alpha[i]+gamma

            # Update altitude
            if i < n-1:
                hs[i+1] = hs[i] + hdot * dt

            # If actual altitude is higher than the target altitude plus half the climb increment, stop climbing
            if hs[i] >= h_targ + aircraft.cruise_climb_inc/2:
                climbing = False

        # Otherwise, stay level
        else:
            climbing = False
            # Zero climb rate
            hdot = 0
            # Zero flight path angle
            gamma = 0
            # CL at steady level cruise, assume small contribution from lift due to thrust
            CL_C = 2 * aircraft.W * g / (rho * aircraft.S * V**2)
            # CD at steady level cruise
            CD_C = aircraft.CD0_C + aircraft.KC * CL_C**2
            # LD at steady level cruise
            LDs[i] = CL_C / CD_C
            # Angle of attack at steady level cruise
            alpha[i] = (CL_C - aircraft.CL0_C) / (aircraft.CLalpha_C)
            # Pitch angle
            thetas[i]=alpha[i]

            # Update altitude
            if i < n-1:
                hs[i+1] = hs[i]

        # Solve for required thrust
        T = aircraft.W * g * (1 / LDs[i] + np.sin(gamma))

        # Solve for required throttle setting
        throttles[i] = T / (alphae_cruise * aircraft.Tmax)

        # Update weight
        dm = TSFC * T * dt  # Fuel flow, kg
        aircraft.W -= dm
        m[i] = aircraft.W

    aircraft.altitude = hs[n-1]

    df = pd.DataFrame(
        {
            "distance": dist,
            "mass": m,
            "L/D": LDs,
            "AoA": alpha,
            "time": ts,
            "altitude": hs,
            "speed": Vs,
            "optimal altitude": opths,
            "throttle": throttles,
            "theta": thetas,
        }
    )

    return df