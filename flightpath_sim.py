import matplotlib.pyplot as plt
import numpy as np

class aircraft_class:
    
    """
    General INPUTS
    """
    rho0 = 1.225                # Sea level air density, kg/m3
    rhorho0 = 0.2978            # Ratio of air density and sea level air density
    rho = rhorho0 * rho0        # Operating altitude air density, kg/m3
    c0 = 340.3                  # Speed of sound at SSL
    c = 295.2                   # Speed of sound at 11km, m/s
    Ma = 0.85

    betaw = 1                   # Ratio of actual weight to maximum weight
    betaw_taxi = 0.99
    betaw_takeoff = 0.99*0.99
    betaw_climb = 0.99*0.99*0.995
    betaw_cruise = 0.99*0.99*0.995*0.98
    betaw_loiter = 0.99*0.99*0.995*0.98*0.854
    betaw_descent = 0.99*0.99*0.995*0.98*0.854*0.991
    betaw_landing = 0.99*0.99*0.995*0.98*0.854*0.991*0.99       # Ratio of actual weight to maximum weight, landing

    Vmax = Ma*c                 # Max speed requirement, m/s
    V_stall_max = 60            # Maximum stall speed, m/s

    WTO = 99.9e3                # Maximum takeoff weight, kg
    WOE = 43e3
    Wpay = 26.06e3
    hscreen = 50 * 0.3048       # Screen height, 50 ft (FAR 25), m
    muwet = 0.05                # Friction coefficient for wet sealed
    mudry = 0.03                # Friction coefficient for dry sealed

    climbgrad_oei_landgear = 0  # Minimum climb gradient for one engine inoperative (OEI) with landing gear extended, CASA part 121, section 9.05
    climbgrad_oei_clean = 2/100 # Minimum climb gradient for one engine inoperative (OEI) with landing gear retracted, CASA part 121, section 9.05
    hdotceil = 1.27             # Climb rate requirement for ceiling, typical value, m/s

    h = 3                       # Distance of wing above the ground, m
    b = 60.2                    # Wing span, m
    S = 661.95                  # Wing area, m2
    g = 9.80665                 # Gravitational acceleration m2/s
    Wmax = WTO*g

    """
    Engine Dependent
    """
    Tmax = 2 * 189.2e3          # Max thrust for 2x Rolls-Royce RB211-535, N
    a0 = 1                      # Default throttle setting
    av = -0.3                   # Constant to model decreasing thrust with increasing M
    a = 0.7                     # Altitude performance index, between 0.7 and 1 depending on engine altitude optimisation
    TSCF_TO = 10.8e-6           #
    TSFC_C = 10.8e-6            #

    """
    Aerodynamic Dependent
    """
    K = 0.0646                  # Induced drag constant

    ### Take Off (AoA = 10 deg)
    CL_TO = 0.7821
    CL0_TO = 0.7
    CD_TO = 0.0517
    CD0_TO = 0.0106
    CLalpha_TO = 3.22           # Lift curve slope, per rad

    ### Cruise (AoA = 0 deg)
    CL_C = 0.2660
    CL0_C = 0.02959
    CD_C = 0.0124
    CD0_C = 0.00678

    ### Landing (AoA = 5 deg)
    CL_L = 0.5063
    CL0_L = 1 # 0.2215
    CD_L = 0.0267
    CD0_L = 0.00639+0.004212
    
aircraft = aircraft_class()


