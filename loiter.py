def loiter(aircraft):
    import numpy as np
    import pandas as pd
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

    loiterh = aircraft.h
    T, P, rhorho0, c = ISA(loiterh)
    rho = rho0*rhorho0

    n = 1000 # Number of time steps in loiter simulation
    dt = aircraft.loiter_time/(n-1)

    loiterThrottle = np.zeros(n)
    loiterAlpha = np.zeros(n)
    loiterW=np.zeros(n)
    loiterV=np.zeros(n)
    loiterLDs=np.zeros(n)
    loiterts = np.linspace(0,aircraft.loiter_time,n)
    loiterDist = np.zeros(n)

    for i in range(n):
        # Minimum drag velocity
        V = np.sqrt(2*aircraft.W/(rho*S) * np.sqrt(K/CD0))
        L = aircraft.W
        CL = 2*L/(rho*V**2*S)

        loiterAlpha[i]=(CL-CL0)/CLalpha

        CD = CD0 + K * CL ** 2
        T = 0.5 * rho * V ** 2 * CD * S

        # Airspeed performance correction
        a0d_cruise = 1 + aircraft.av * V/c
        # Ratio of maximum static thrust or power at sea level to the thrust or power at the desired operating condition, cruise
        alphae_cruise = a0d_cruise * (rhorho0)**aircraft.a

        # Solve for required throttle setting
        loiterThrottle[i] = T / (alphae_cruise * Tmax)

        # Update weight
        dm = aircraft.TSFC_C * T / 1e6 * dt  # Fuel flow, kg
        aircraft.W -= dm
        loiterW[i] = aircraft.W

        loiterV[i] = V
        loiterLDs[i] = CL/CD
        if i != 0:
            loiterDist[i] = loiterDist[i-1] + dt * V

    loiterTheta = loiterAlpha

    return loiterAlpha, loiterThrottle, loiterW, loiterts, loiterV, loiterLDs, loiterTheta, loiterDist