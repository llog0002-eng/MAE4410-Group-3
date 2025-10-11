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
    CD0 = aircraft.CD0_DE                           # Zero lift drag coefficient
    CL0 = aircraft.CL0_DE                           # Zero angle of attack lift coefficient
    CLalpha = aircraft.CLalpha_DE                   # Lift curve slope, per rad
    TSFC = aircraft.TSFC_C/1e6                      # TSFC, kg/(N.s)

    hdot = aircraft.descent_hdot                    # Rate of descent, m/s
    gamma = aircraft.descent_gamma * np.pi/180      # Flight path angle, deg
    V = hdot/np.sin(gamma)                          # Descent velocity, m/s

    n = 500 # Number of distance steps
    t_descent = (aircraft.altitude-aircraft.screenh)/hdot # Descent time, s
    dt=t_descent/n # Timestep, s

    # Preallocate arrays
    throttles = np.zeros(n)
    alpha = np.zeros(n)
    m=np.zeros(n)
    Vs=np.ones(n)*V
    LDs=np.zeros(n)
    ts = np.linspace(0,t_descent,n)
    dist = np.linspace(0,V*np.cos(gamma)*t_descent,n)
    hs = np.zeros(n)                                  # Altitude array, m
    thetas = np.zeros(n)

    hs[0] = aircraft.altitude

    for i in range(n):
        Temp, P, rhorho0, c = ISA(hs[i])
        rho = rhorho0 * rho0

        # CL, assume small contribution from lift due to thrust
        CL = 2 * aircraft.W * g / (rho * aircraft.S * Vs[i]**2) / np.cos(gamma)
        # CD
        CD = aircraft.CD0_DE + aircraft.KC * CL**2
        # LD
        LDs[i] = CL / CD
        # Angle of attack at climb
        alpha[i] = (CL - aircraft.CL0_DE) / (aircraft.CLalpha_DE)
        # Pitch angle
        thetas[i] = alpha[i]+gamma

        # Airspeed performance correction
        a0d = 1 + aircraft.av * Vs[i]/c
        # Ratio of maximum static thrust or power at sea level to the thrust or power at the desired operating condition, cruise
        alphae = a0d * (rhorho0)**aircraft.a

        # Solve for required thrust
        T = aircraft.W * g * (1 / LDs[i] - np.sin(gamma))
        # Clamp thrust to > 0 if negative or V > Vdescent
        if T < 0 or Vs[i] > hdot/np.sin(gamma):
            a = -T/aircraft.W
            Vs[i+1] = Vs[i] + dt * a
            T = 0
        elif i != n-1:
            Vs[i+1] = Vs[i]
        

        # Solve for required throttle setting
        throttles[i] = T / (alphae * aircraft.Tmax)

        # Update weight
        dm = TSFC * T * dt  # Fuel flow, kg
        aircraft.W -= dm
        m[i] = aircraft.W

        if i != n-1:
            # Update altitude
            hs[i+1] = hs[i] - dt * hdot


    aircraft.altitude = hs[n-1]
    aircraft.V = Vs[n-1]

    df = pd.DataFrame(
        {
            "distance": dist,
            "mass": m,
            "L/D": LDs,
            "AoA": alpha,
            "time": ts,
            "altitude": hs,
            "speed": Vs,
            "optimal altitude": np.full(n, np.nan),
            "throttle": throttles,
            "theta": thetas,
        }
    )

    return df
