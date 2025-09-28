import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np
from takeoff import takeoff
from cruise import cruise

class aircraft_class:
    
    """
    General INPUTS
    """
    rho0 = 1.225                # Sea level air density, kg/m3
    rhorho0 = 0.2978            # Ratio of air density and sea level air density
    rho = rhorho0 * rho0        # Operating altitude air density, kg/m3
    c0 = 340.3                  # Speed of sound at SSL
    c = 295.2                   # Speed of sound at 11km, m/s
    Ma = 0.85                   # Cruise speed, Ma

    betaw = 1                   # Ratio of actual weight to maximum weight
    betaw_taxi = 0.99
    betaw_takeoff = betaw_taxi*0.99
    betaw_climb = betaw_takeoff*0.97
    betaw_cruise = betaw_climb*0.98
    betaw_loiter = betaw_cruise*0.820
    betaw_descent = betaw_loiter*0.989
    betaw_landing = betaw_descent*0.99       # Ratio of actual weight to maximum weight, landing

    Vmax = Ma*c                 # Max speed requirement, m/s
    V_stall_max = 60            # Maximum stall speed, m/s

    WTO = 98e3                  # Maximum takeoff weight, kg
    W = WTO                     # Initial weight, kg
    hscreen = 50 * 0.3048       # Screen height, 50 ft (FAR 25), m
    muwet = 0.05                # Friction coefficient for wet sealed
    mudry = 0.03                # Friction coefficient for dry sealed

    climbgrad_oei_landgear = 0  # Minimum climb gradient for one engine inoperative (OEI) with landing gear extended, CASA part 121, section 9.05
    climbgrad_oei_clean = 2/100 # Minimum climb gradient for one engine inoperative (OEI) with landing gear retracted, CASA part 121, section 9.05
    hdotceil = 1.27             # Climb rate requirement for ceiling, typical value, m/s

    range = 8000e3              # Range requirement, m
    loiter_time = 30*60         # Loiter time requirement, s
    cruise_climb_inc = 300     # Altitude increment for cruise climb, m

    g = 9.80665                 # Gravitational acceleration m2/s
    Wmax = WTO*g

    """
    Engine Dependent
    """
    Tmax = 2 * 189.2e3              # Max thrust for 2x Rolls-Royce RB211-535, N
    Tmax_continuous = 2 * 156.6e3   # Max continuous thrust for 2x Rolls-Royce RB211-535, N
    a0 = 1                          # Default throttle setting
    a0_cruise = 0.8                 # Throttle setting in cruise
    av = -0.3                       # Constant to model decreasing thrust with increasing M
    a = 0.7                         # Altitude performance index, between 0.7 and 1 depending on engine altitude optimisation
    TSFC_C = 10.8                   # Cruise TSFC, g/(kN.s)
    TSFC_TO = TSFC_C * 1.2          # Takeoff TSFC, g/(kN.s)

    """
    Aerodynamic Dependent
    """

    h = 3                       # Distance of wing above the ground, m
    b = 60.2                    # Wing span, m
    S = 661.95                  # Wing area, m2

    KTO = 0.0646                  # Induced drag constant, takeoff
    KC = 0.0646                   # Induced drag constant, cruise

    ### Take Off
    CL0_TO = 1.5
    CD0_TO = 0.0106 + 0.004212  # Added for landing gear drag
    CLalpha_TO = 2.835          # Lift curve slope, per rad

    ### Climb
    CL0_CL = 0.13460
    CD0_CL = 0.0106
    CLalpha_C = 3.515           # Lift curve slope, per rad

    ### Cruise
    CL0_C = 0.13460
    CD0_C = 0.00678
    CLalpha_C = 3.515              # Lift curve slope, per rad

    ### Loiter
    CL0_LO = 0.02959
    CD0_LO = 0.00678
    CLalpha_LO = 3.515              # Lift curve slope, per rad

    ### Descent
    CL0_DE = 0.7
    CD0_DE = 0.0106
    CLalpha_DE = 2.835           # Lift curve slope, per rad

    ### Landing
    CL0_LA = 1 # 0.2215
    CD0_LA = 0.00639 + 0.004212  # Added for landing gear drag
    CLalpha_LA = 2.835           # Lift curve slope, per rad
    
aircraft = aircraft_class()

#region // Taxi
aircraft.W = aircraft.WTO * aircraft.betaw_taxi
#endregion

#region // Take-off
sair_TO, gamma_TOd, theta_TOd, AoA_TOd = takeoff(aircraft)
#endregion

#region // Climb
#endregion

#region // Cruise
cruiseDist, Wcruises, cruiseLDs, cruiseAlpha, cruisets, cruiseh, cruiseV, cruiseopth, cruiseThrottle = cruise(aircraft)

print(f"Cruise weight ratio: {Wcruises[-1]/Wcruises[0]:.2f}")
print(f"Cruise time: {cruisets[-1]/3600} hours")

fig,axs = plt.subplots(2,2, sharex=True, figsize=(10,6))

axs[0,0].plot(cruiseDist/1e3,cruiseh/1e3,label = "Altitude [km]",color='blue')
axs[0,0].plot(cruiseDist/1e3,cruiseopth/1e3,label = "Optimal altitude [km]",color='cyan')
axs[0,1].plot(cruiseDist/1e3,cruiseAlpha*180/np.pi,label = "Angle of attack [deg]",color='red')
axs[1,0].plot(cruiseDist/1e3,cruiseLDs,label = "L/D ratio",color='green')
axs[1,1].plot(cruiseDist/1e3,Wcruises/1e3,label = "Weight [t]",color='orange')

axs[0,0].set_ylabel("Altitude [km]")
axs[0,1].set_ylabel("Angle of attack [deg]")
axs[1,0].set_ylabel("L/D ratio")
axs[1,1].set_ylabel("Weight [t]")

axs[1,0].set_xlabel("Cruise distance [km]")
axs[0,1].set_xlabel("Cruise distance [km]")
axs[0,0].set_xlabel("Cruise distance [km]")
axs[1,1].set_xlabel("Cruise distance [km]")

axs[0,0].legend()

mpl.rc("savefig", dpi=300)
plt.savefig("../../Cruise Performance.png", bbox_inches='tight')

plt.show()
#endregion

#region // Loiter
#endregion

#region // Descent
#endregion

#region // Landing
#endregion

