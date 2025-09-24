import numpy as np

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
TSCF_TO = 10.8e-6
TSFC_C = 10.8e-6


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
CL0_L = 0.2215
CD_L = 0.0267
CD0_L = 0.00639

"""
Take-Off Airspeed
"""

theta_LOF = 10*np.pi/180                            # liftoff pitch angle, rad

CL_LOF = CL0_TO + CLalpha_TO * theta_LOF            # Lift coefficient at liftoff
W_TO = Wmax*betaw_taxi                              # Weight at takeoff, N
kTO = 1.13
V_stall_TO = np.sqrt(2 * W_TO/S / (rho0 * CL_LOF))  # Stall speed at takeoff, m/s
VR = 1.05 * V_stall_TO                              # Rotate speed, m/s
VLOF = 1.1 * V_stall_TO                             # Lift-off speed, m/s
V2 = 1.13 * V_stall_TO                              # Take-off safety speed, m/s

a0_TO = 1                                           # Throttle setting in Takeoff
a0d_TO = a0_TO * (1 + av * VLOF/c0)                 # Airspeed performance correction
alphae_TO = a0d_TO * (1)**a                         # Ratio of maximum static thrust or power at sea level to the thrust or power at the desired operating condition, takeoff
T_TO = Tmax*alphae_TO

theta_TO = theta_LOF                                # Initial takeoff pitch angle, rad

dt_TO = 0.000001


# Initialise variables
modifier = 1
h_TO = 0
iteration_i = 0
iteration_o = 0
Vair_TO = VLOF
sair_TO = 0
speed_too_low = True

while abs(1-Vair_TO/V2) > 0.001 or Vair_TO <= V2 or h_TO < hscreen:
    if abs(1-Vair_TO/V2) > 0.001:
        print(f'Current airspeed: {Vair_TO}, V2: {V2}, Pitch angle: {theta_TO*180/np.pi}')
    elif Vair_TO <= V2:
        print(f'Current airspeed: {Vair_TO}, V2: {V2}, airspeed too low')
    elif h_TO < hscreen:
        print(f'Current height: {h_TO}, screen height: {hscreen}, height too low')
    
    h_TO = 0
    iteration_i = 0
    Vair_TO = VLOF
    sair_TO = 3*VLOF

    while h_TO < hscreen:
        CL = W_TO*np.cos(theta_TO) / (0.5*rho0*Vair_TO**2*S)
        CD = CD0_TO + K*CL**2
        D = 0.5*rho0*Vair_TO**2*S*CD
        AoA_TO = (CL - CL0_TO) / CLalpha_TO  # Angle of attack, rad

        gamma_TO = theta_TO - AoA_TO # Flight path angle, rad
        Vair_TO += (T_TO - D - W_TO*np.sin(theta_TO))/(W_TO/g) * dt_TO
        sair_TO += Vair_TO*np.cos(gamma_TO)*dt_TO
        hdot_TO = Vair_TO*np.sin(gamma_TO)
        h_TO += hdot_TO*dt_TO
        iteration_i += 1

        mdot_TO = TSCF_TO*T_TO
        W_TO -= mdot_TO*dt_TO*g

        # if iteration_i > 1000000:
        #     print("The inner function has iterated 100,000 times and hscreen is still not reached. Please check the code.")
        #     print(theta_TO*180/np.pi)
        #     break

        if gamma_TO < -5*np.pi/180:
            print("The flight path angle has become negative. Please check the code.")
            print(gamma_TO*180/np.pi)
            raise Exception("Flight path angle negative")

    # if iteration_o > 100:
    #         print("The outer function has iterated 100 times and hscreen is still not reached. Please check the code.")
    #         print(theta_TO*180/np.pi)
    #         break

    speed_too_low_last = speed_too_low

    if Vair_TO < V2:
        speed_too_low = True
    else:
        speed_too_low = False

    if speed_too_low_last != speed_too_low:
        modifier *= 0.5

    if speed_too_low:
        theta_TO -= modifier * 0.1 * np.pi/180
    else:
        theta_TO += modifier * 0.1 * np.pi/180

    iteration_o += 1
    


print(f'The function has iterated {iteration_o} times.\n\nThe final height of the airplane is {h_TO} m.\n\n\
The final airspeed of takeoff is {Vair_TO} m/s.\n\nThe final air distance is {sair_TO} m.')
if Vair_TO < V2:
    print(f'The final airspeed of takeoff is {Vair_TO} and it\'s less than the safety speed V2={V2}. VLOF={VLOF}.')
print(f"The final flight path angle is {gamma_TO*180/np.pi} degrees.")
print(f"The final pitch angle is {theta_TO*180/np.pi} degrees.")



"""
Climb
"""
# theta_C = 0
# AoA_C = 10*np.pi/180
# W_C = Wmax*betaw_takeoff
# alpha_p = AoA_C

# theta_old = theta_C
# theta_new = theta_C
# while theta_new-theta_old >= 0.1:
#     theta_old = theta_new
#     L = W_C*np.cos(theta_old) - Tmax*np.sin(alpha_p)
#     CL = L/(0.5*rho0*)


"""
Range
"""
# TSFC_C = 10.7e-6
# ma = WTO*betaw_climb
# mf = ma - WOE - Wpay

# mn = ma
# mn1 = ma
# while mn1 > ma-mf:
#     mn = mn1
#     L = ma*g - 
#     mfdot = TSFC_C*(T+)



