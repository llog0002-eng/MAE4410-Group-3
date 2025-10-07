def loiter(aircraft):
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

    n = 1000 # Number of distance steps in loiter simulation

    loiterts = np.linspace(0,aircraft.loiter_time,n)
    for i in range(n):
        Vmindrag = np.sqrt(aircraft.W/S)
        L = aircraft.W
        rho = ISA(aircraft.h)[2]
        CL = 2/(rho*Vmindrag**2*S)
        CD = CD0 + K * CL ** 2 