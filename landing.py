def landing(aircraft):
    # Input: aircraft object with all relevant variables
    # Output: property dataframe

    import numpy as np
    import pandas as pd

    rho0 = 1.225                                    # Air density, kg/m3
    g = 9.80665                                     # Gravitational acceleration m/s2

    Vland = aircraft.Vland                          # Landing speed, m/s
    hdot = aircraft.hdot_land                       # Vertical speed before flare, m/s
    mu = aircraft.mutyre                            # Friction coefficient of tyres

    S = aircraft.S                                  # Wing area, m2
    Tmax = aircraft.Tmax                            # Max continuous thrust, N
    V = aircraft.Vmax                               # Cruise speed requirement, m/s
    K = aircraft.KC                                 # Induced drag constant
    CD0 = aircraft.CD0_C                            # Zero lift drag coefficient
    CL0 = aircraft.CL0_C                            # Zero angle of attack lift coefficient
    CLalpha = aircraft.CLalpha_C                    # Lift curve slope, per rad
    TSFC = aircraft.TSFC_C/1e6                      # TSFC, kg/(N.s)

    n = 1000 # Number of points to initialise
    dt = 0.1
    dist = np.zeros(n)
    m = np.zeros(n)
    LDs = np.zeros(n)
    alpha = np.zeros(n)
    ts = np.zeros(n)
    altitude = np.zeros(n)
    Vs = np.zeros(n)
    throttles = np.ones(n)
    theta = np.zeros(n)

    i = 0 # Inner while loop iteration count
    Vs[0] = Vland
    altitude[0] = aircraft.screenh

    # Landing descent
    while altitude[i] > 0:
        
        if i == n:
            raise Exception("Takeoff iteration count passed datapoint preallocation size, air distance")
         
        gamma = np.arcsin(hdot/Vland)
        Vs[i+1] = Vland
         
        CL = aircraft.W * g/np.cos(gamma) / (1/2 * rho0 * Vs[i+1] ** 2 * S)
        CD = CD0 + K*CL**2

        LDs[i+1] = CL/CD
        D = 0.5*rho0*Vs[i]**2*S*CD

        alpha[i+1] = (CL - CL0) / CLalpha  # Angle of attack, rad
        theta[i+1] = gamma + alpha[i+1]

        # Solve for required thrust
        T = aircraft.W * g * (1 / LDs[i+1] - np.sin(gamma))

        # Solve for required throttle
        # Airspeed performance correction
        a0d = 1 + aircraft.av * V/aircraft.c0
        throttles[i+1] = T / (a0d * Tmax)

        dist[i+1] = dist[i] + Vs[i+1]*np.cos(gamma)*dt
        altitude[i+1] = altitude[i] - hdot*dt
        if altitude[i+1] < 0:
            altitude[i+1] = 0

        dm = aircraft.TSFC_TO*T/1e6*dt # Fuel flow, kg
        aircraft.W -= dm
        m[i+1] = aircraft.W

        if i == 0:
            m[0] = m[1]
            LDs[0] = LDs[1]
            throttles[0] = throttles[1]
            alpha[0] = alpha[1]
            theta[0] = theta[1]

        i += 1 # Inner while loop iteration count
        ts[i+1] = ts[i] + dt
        
    i_groundroll = i # Mark when ground roll starts

    # Ground roll
    while Vs[i] > 0:
        if i == n:
            raise Exception("Takeoff iteration count passed datapoint preallocation size, ground roll")
        
        alpha[i+1] = 0
        gamma = 0
        theta[i+1] = gamma + alpha[i+1]

        CL = CL0
        CD = CD0 + K * CL ** 2

        L = 1/2 * rho0 * Vs[i] ** 2 * CL * S        # Lift
        D = 1/2 * rho0 * Vs[i] ** 2 * CD * S        # Air drag
        muN = (aircraft.W * g - L) * mu             # Ground friction
        a = (D + muN)/aircraft.W                    # Acceleration
        Vs[i+1] = Vs[i] - a * dt                    # Update velocity

        dist[i+1] = dist[i] + Vs[i+1]*np.cos(gamma)*dt
        altitude[i+1] = 0

        m[i+1] = aircraft.W

        throttles[i+1] = 0
        i += 1
        ts[i+1] = ts[i] + dt

    print(f"Landing air distance is {dist[i_groundroll-1]}")
    print(f"Landing ground roll is {dist[i]-dist[i_groundroll]}")

    aircraft.altitude = 0

    df = pd.DataFrame(
        {
            "distance": dist[:i],
            "mass": m[:i],
            "L/D": LDs[:i],
            "AoA": alpha[:i],
            "time": ts[:i],
            "altitude": altitude[:i],
            "speed": Vs[:i],
            "optimal altitude": np.full(i,np.nan),
            "throttle": throttles[:i],
            "theta": theta[:i],
        }
    )

    return df