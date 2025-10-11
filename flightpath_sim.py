import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np
import pandas as pd
from tabulate import tabulate

from takeoff import takeoff
from climb import Climb
from cruise import cruise
from loiter import loiter
from descent import descent
from landing import landing

from range import Range
from Vn_diagram import Vn_diagram

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
    altitude = 0                # Altitude, m (preallocated)

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
    V = 0                       # Current velocity variable

    WTO = 109e3                 # Maximum takeoff weight, kg
    m_fuel = 26.2e3             # Maximum mass of fuel, kg
    W = WTO                     # Initial weight, kg
    hscreen = 50 * 0.3048       # Screen height, 50 ft (FAR 25), m
    muwet = 0.05                # Friction coefficient for wet sealed
    mudry = 0.03                # Friction coefficient for dry sealed
    mutyre = 0.4                # Worst case tyre friction coefficient

    climbgrad_oei_landgear = 0  # Minimum climb gradient for one engine inoperative (OEI) with landing gear extended, CASA part 121, section 9.05
    climbgrad_oei_clean = 2/100 # Minimum climb gradient for one engine inoperative (OEI) with landing gear retracted, CASA part 121, section 9.05
    hdotceil = 1                # Climb rate requirement for ceiling
    screenh = 50*0.3048         # Landing screen height, m

    range = 8000e3              # Range requirement, m
    loiter_time = 30*60         # Loiter time requirement, s
    cruise_climb_inc = 300      # Altitude increment for cruise climb, m
    descent_gamma = 3           # Typical descent angle, deg (https://books.google.com.au/books?id=ZmTfH0bYz_YC&redir_esc=y)
    descent_hdot = 13           # Typical descent speed, m/s (https://pilotinstitute.com/how-to-calculate-descent)
    Vland = 145*0.5144          # Landing speed, m/s (from 145 kt from RFP)
    hdot_land = 1.5             # Vertical speed before flare, m/s

    g = 9.80665                 # Gravitational acceleration m2/s
    Wmax = WTO*g
    W_fuel = m_fuel*g

    """
    Engine Dependent
    """
    Tmax = 2*143.05e3                   # Max thrust for 2x CFM LEAP-1A, N
    Tmax_continuous = 2*140.96e3        # Max continuous thrust for 2x CFM LEAP-1A, N
    a0 = 1                              # Default throttle setting
    a0_continuous = Tmax_continuous/Tmax# Max continuous throttle setting
    a0_climb = 0.9                      # Throttle setting in climb, 0.9-0.95 (Wk5 lecture note page 26)
    a0_cruise = 0.8                     # Throttle setting in cruise
    av = -0.3                           # Constant to model decreasing thrust with increasing M
    a = 0.7                             # Altitude performance index, between 0.7 and 1 depending on engine altitude optimisation
    TSFC_C = 14.4                       # Cruise TSFC, g/(kN.s)
    TSFC_TO = TSFC_C * 1.2              # Takeoff TSFC, g/(kN.s)
    ip = 0                              # Constant angle of incidence of propulsion system. Zero if engines are parallel to the pitch angle
    rp = 2.658                          # distance between centre of engine to centre of mass, m
    
    """
    Aerodynamic Dependent
    """
    h = 1.1                       # Distance of wing above the ground, m
    b = 40.62                    # Wing span, m
    S = 394                     # Wing area, m2
    c = 5.14668                 # mean geometric chord of the wing, m

    KTO = 0.0868                # Induced drag constant, takeoff
    KC = 0.0868                 # Induced drag constant, cruise

    ### Take Off
    CL0_TO = 1.5
    CD0_TO = 0.0087 + 0.004212    # Added for landing gear drag
    CLalpha_TO = 0.0342*180/np.pi # Lift curve slope, per rad
    CL_max_TO = 0.5456
    CL_min_TO = -0.6082

    ### Climb
    CL0_CL = 0.1364
    CD0_CL = 0.0049
    CLalpha_CL = 0.0415*180/np.pi           # Lift curve slope, per rad

    ### Cruise
    CL0_C = CL0_CL
    CD0_C = CD0_CL
    CLalpha_C = CLalpha_CL              # Lift curve slope, per rad
    CL_max_C = 0.5381
    CL_min_C = -0.5690

    ### Loiter
    CL0_LO = CL0_C
    CD0_LO = CD0_C
    CLalpha_LO = CLalpha_C              # Lift curve slope, per rad

    ### Descent
    CL0_DE = CL0_C
    CD0_DE = CD0_C
    CLalpha_DE = CLalpha_C           # Lift curve slope, per rad

    ### Landing
    CL0_LA = CL0_TO
    CD0_LA = CD0_TO  # Added for landing gear drag
    CLalpha_LA = CLalpha_TO           # Lift curve slope, per rad
    CL_max_LA = 0.5456
    CL_min_LA = -0.6082

    """
    Takeoff Data
    """
    theta_stall = 10 * np.pi/180                                # Stall angle, rad
    CL_stall = CL0_TO + CLalpha_TO * theta_stall                # Lift coefficient at stall, takeoff
    W_TO = Wmax*betaw_taxi                                      # Weight at takeoff, N
    V_stall_TO = np.sqrt(2 * W_TO/S / (rho0 * CL_stall))        # Stall speed at takeoff, m/s
    VR = 1.05 * V_stall_TO                                      # Rotate speed, m/s
    VLOF = 1.1 * V_stall_TO                                     # Lift-off speed, m/s
    V2 = 1.13 * V_stall_TO                                      # Take-off safety speed, m/s


    """
    Climb Date
    """
    W_CL = Wmax*betaw_takeoff


    """
    V-n Diagram Data
    """
    # Both n from course note 13 page 23
    n_pos_limit = 3.5
    n_neg_limit = -1.5

    """
    Landing Data
    """
    W_LA = Wmax*betaw_descent
    V_stall_LA = np.sqrt(2 * (W_LA/S) / (rho0*CL_max_LA))

    
aircraft = aircraft_class()


#region // Taxi
aircraft.W = aircraft.WTO * aircraft.betaw_taxi
#endregion


#region // Take-off
takeoffdf = takeoff(aircraft)
#endregion


# #region // Climb
climb = Climb(aircraft, 0, 11000, 500, 0.0001, 1, takeoffdf)
# climb.best_RC_data()
# climb.best_AC_data()
# climb.get_plot(climb.hdot_list)
# climb.get_Service_Ceiling()
# climb.get_all_plot()
climbdf = climb.get_flight_sim(aircraft)

# #endregion

# #region // Range
#     # used to calculated MAX Range it can raeach, ignoring loiter, descent and landing. It's the max distance after climb
# range = Range(aircraft, aircraft.Vmax, 11000, 0.001, 6)
# range.get_data()
# #endregion


#region // Cruise
cruisedf = cruise(aircraft)

cruisemratio = cruisedf.iloc[-1]["mass"]/cruisedf.iloc[0]["mass"]
cruisetime = cruisedf.iloc[-1]["time"]/3600
print(f"Cruise weight ratio: {cruisemratio:.3f}")
print(f"Cruise time: {cruisetime:.1f} hours")
#endregion


#region // Loiter
loiterdf = loiter(aircraft)

loitermratio = loiterdf.iloc[-1]["mass"]/loiterdf.iloc[0]["mass"]
loitertime = loiterdf.iloc[-1]["time"]/3600
print(f"Loiter weight ratio: {loitermratio:.3f}")
print(f"Loiter time: {loitertime:.1f} hours")
#endregion


#region // Descent
descentdf = descent(aircraft)

descentmratio = descentdf.iloc[-1]["mass"]/descentdf.iloc[0]["mass"]
descenttime = descentdf.iloc[-1]["time"]/3600
print(f"Descent weight ratio: {descentmratio:.3f}")
print(f"Descent time: {descenttime:.1f} hours")
#endregion


#region // Landing
landingdf = landing(aircraft)

landingmratio = landingdf.iloc[-1]["mass"]/landingdf.iloc[0]["mass"]
landingtime = landingdf.iloc[-1]["time"]/3600
print(f"landing weight ratio: {landingmratio:.3f}")
print(f"landing time: {landingtime:.1f} hours")
#endregion


#region // Combining data

dfs = [takeoffdf, climbdf, cruisedf, loiterdf, descentdf, landingdf]

# Update each df's starting time/distance from the previous df's end time/distance
for i in range(len(dfs)-1):
    dfs[i+1]["time"] = dfs[i+1]["time"] + dfs[i].iloc[-1]["time"]
    dfs[i+1]["distance"] = dfs[i+1]["distance"] + dfs[i].iloc[-1]["distance"]

# Concatenate all dfs
totdf = pd.concat(dfs)

#endregion


#region // Plotting

plotdf = totdf

fig,axs = plt.subplots(3,2, sharex=True, figsize=(10,6))

axs[0,0].plot(plotdf["time"],plotdf["altitude"]/1e3,label = "Altitude [km]",color='blue')
axs[0,0].plot(plotdf["time"],plotdf["optimal altitude"]/1e3,label = "Optimal altitude [km]",color='cyan')
axs[0,1].plot(plotdf["time"],plotdf["AoA"]*180/np.pi,label = "Angle of attack [deg]",color='red')
axs[0,1].plot(plotdf["time"],plotdf["theta"]*180/np.pi,label="Pitch angle [deg]",color="blue",linestyle="--")
axs[1,0].plot(plotdf["time"],plotdf["L/D"],label = "L/D ratio",color='green')
axs[1,1].plot(plotdf["time"],plotdf["mass"]/1e3,label = "Weight [t]",color='orange')
axs[2,0].plot(plotdf["time"],plotdf["throttle"],label = "Throttle setting",color='purple')
axs[2,1].plot(plotdf["time"],plotdf["speed"],label = "TAS [m/s]",color='brown')

axs[2,1].ticklabel_format(useOffset=False, style='plain')

axs[0,0].set_ylabel("Altitude [km]")
axs[0,1].set_ylabel("Angle of attack [deg]")
axs[1,0].set_ylabel("L/D ratio")
axs[1,1].set_ylabel("Weight [t]")
axs[2,0].set_ylabel("Throttle setting")
axs[2,1].set_ylabel("Velocity [m/s]")

axs[2,0].set_xlabel("Time [s]")
axs[2,1].set_xlabel("Time [s]")

axs[0,0].legend()
axs[0,1].legend()

mpl.rc("savefig", dpi=300)
plt.savefig("../../Cruise Performance.png", bbox_inches='tight')

fig.suptitle("Flight Simulation")

Vn_diagram(aircraft,0.1)

plt.show()

#endregion

