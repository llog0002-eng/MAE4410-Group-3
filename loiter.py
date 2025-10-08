def loiter(aircraft):
    # Input: aircraft object with all relevant variables
    # Output: property dataframe

    import numpy as np
    import pandas as pd
    from ISA import ISA
    from ISA_rho import ISA_rho
    
    g = 9.80665                                   # Gravitational acceleration m2/s
    rho0 = 1.225                                  # Sea level air density, kg/m3
    
    S = aircraft.S                                  # Wing area, m2
    Tmax = aircraft.Tmax                            # Max continuous thrust, N
    V = aircraft.Vmax                               # Cruise speed requirement, m/s
    K = aircraft.KC                                 # Induced drag constant
    CD0 = aircraft.CD0_C                            # Zero lift drag coefficient
    CL0 = aircraft.CL0_C                            # Zero angle of attack lift coefficient
    CLalpha = aircraft.CLalpha_C                    # Lift curve slope, per rad

    loiterh = aircraft.h
    T, P, rhorho0, c = ISA(loiterh)
    rho = rho0*rhorho0

    n = 100 # Number of time steps in loiter simulation
    dt = aircraft.loiter_time/(n-1)

    throttles = np.zeros(n)
    alpha = np.zeros(n)
    m=np.zeros(n)
    Vs=np.zeros(n)
    LDs=np.zeros(n)
    ts = np.linspace(0,aircraft.loiter_time,n)
    dist = np.zeros(n)

    for i in range(n):
        # Minimum drag velocity
        V = np.sqrt(2*aircraft.W * g/(rho*S) * np.sqrt(K/CD0))
        L = aircraft.W * g
        CL = 2*L/(rho*V**2*S)

        alpha[i]=(CL-CL0)/CLalpha

        CD = CD0 + K * CL ** 2
        T = 0.5 * rho * V ** 2 * CD * S

        # Airspeed performance correction
        a0d_cruise = 1 + aircraft.av * V/c
        # Ratio of maximum static thrust or power at sea level to the thrust or power at the desired operating condition, cruise
        alphae_cruise = a0d_cruise * (rhorho0)**aircraft.a

        # Solve for required throttle setting
        throttles[i] = T / (alphae_cruise * Tmax)

        # Update weight
        dm = aircraft.TSFC_C * T / 1e6 * dt  # Fuel flow, kg
        aircraft.W -= dm
        m[i] = aircraft.W

        Vs[i] = V
        LDs[i] = CL/CD
        if i != 0:
            dist[i] = dist[i-1] + dt * V

    loiterTheta = alpha

    df = pd.DataFrame(
        {
            "distance": dist,
            "mass": m,
            "L/D": LDs,
            "AoA": alpha,
            "time": ts,
            "altitude": np.ones(n)*loiterh,
            "speed": Vs,
            "optimal altitude": np.zeros(n),
            "throttle": throttles,
            "theta": alpha,
        }
    )

    return df