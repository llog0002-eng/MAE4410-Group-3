import numpy as np
def OEI(aircraft):
    rp = aircraft.rp
    VLOF = aircraft.VLOF
    V2 = aircraft.V2
    Tmax = aircraft.Tmax
    S = aircraft.S
    b = aircraft.b
    rho = aircraft.rho0
    a0_TO = 1                                                               # Throttle setting in Takeoff
    Dp = Tmax*0.1

    V_array = np.array([VLOF, V2])
    n = len(V_array)
    a0d_array = np.zeros(n)
    alphae_array = np.zeros(n)
    T_array = np.zeros(n)
    Cn0_array = np.zeros(n)
    CD0_Eng_array = np.zeros(n)

    for i in range(n):
        a0d_array[i] = a0_TO * (1 + aircraft.av * V_array[i]/aircraft.c0)                   # Airspeed performance correction
        alphae_array[i] = a0d_array[i] * (1)**aircraft.a                                    # Ratio of maximum static thrust or power at sea level to the thrust or power at the desired operating condition, takeoff
        T_array[i] = Tmax*alphae_array[i]
        Cn0_array[i] = (rp*(T_array[i]+Dp))/(0.5*rho*V_array[i]**2*S*b)
        CD0_Eng_array[i] = Dp/(0.5*rho*V_array[i]**2*S*b)