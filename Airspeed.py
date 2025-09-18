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
betaw_taxi = 0.99*0.99
betaw_takeoff = 0.99*0.99*0.995
betaw_climb = 0.99*0.99*0.995*0.98
betaw_cruise = 0.99*0.99*0.995*0.98*0.810
betaw_loiter = 0.99*0.99*0.995*0.98*0.810*0.99
betaw_descent = 0.99*0.99*0.995*0.98*0.810*0.99*0.99
betaw_landing = 0.752       # Ratio of actual weight to maximum weight, landing

Vmax = Ma*c                 # Max speed requirement, m/s
V_stall_max = 60            # Maximum stall speed, m/s

WTO = 118e3               # Maximum takeoff weight, kg
WOE = 57.4e3
Wpay = 26.06e3
hscreen = 50 * 0.3048       # Screen height, 50 ft (FAR 25), m
muwet = 0.05                # Friction coefficient for wet sealed
mudry = 0.03                # Friction coefficient for dry sealed


climbgrad_oei_landgear = 0      # Minimum climb gradient for one engine inoperative (OEI) with landing gear extended, CASA part 121, section 9.05
climbgrad_oei_clean = 2/100     # Minimum climb gradient for one engine inoperative (OEI) with landing gear retracted, CASA part 121, section 9.05
hdotceil = 1.27                 # Climb rate requirement for ceiling, typical value, m/s

h = 3                       # Distance of wing above the ground, m
b = 60.2              # Wing span, m
S = 661.95          # Wing area, m2
g = 9.80665                 # Gravitational acceleration m2/s
Wmax = WTO*g

"""
Engine Dependent
"""
Tmax = 2 * 264.4e3            # Max thrust for 2x Rolls-Royce RB211-535, N
a0 = 1                      # Default throttle setting
av = -0.3                   # Constant to model decreasing thrust with increasing M
a = 0.7                     # Altitude performance index, between 0.7 and 1 depending on engine altitude optimisation


"""
Aerodynamic Dependent
"""
K = 0.0646              # Induced drag constant

### Take Off (AoA = 10 deg)
CL_TO = 2
CD_TO = 0.0517
CD0_TO = 0.0096

### Cruise (AoA = 0 deg)
CL_C = 0.2657
CD_C = 0.0112
CD0_C = 0.0061

### Landing (AoA = 5 deg)
CL_L = 0.4521
CD_L = 0.0237
CD0_L = 0.0072


"""
Take-Off Airspeed
"""
theta_TO = 12*np.pi/180                             # Takeoff climb angle 
AoA_TO = 10*np.pi/180                               # AoA during takeoff

W_TO = Wmax*betaw_taxi
kTO = 1.13
V_stall_TO = np.sqrt(2 * W_TO/S / (rho0 * CL_TO))     # Stall speed at takeoff, m/s
VR = 1.05 * V_stall_TO                              # Rotate speed, m/s
VLOF = 1.1 * V_stall_TO                             # Lift-off speed, m/s
V2 = 1.13 * V_stall_TO                              # Take-off safety speed, m/s
a0_TO = 1                                           # Throttle setting in Takeoff
a0d_TO = a0_TO * (1 + av * Vmax/c0)                 # Airspeed performance correction
alphae_TO = a0d_TO * (1)**a                   # Ratio of maximum static thrust or power at sea level to the thrust or power at the desired operating condition, takeoff
T_TO = Tmax*alphae_TO

iteration = 0
gamma_TO = theta_TO-AoA_TO
dt_TO = 0.00001
h_TO = 0
Vair_TO = VLOF
sair_TO = 3*VLOF
while h_TO < hscreen:
    Vair_TO += (Tmax-0.5*rho0*Vair_TO**2*S*(CD0_TO+K*CL_TO**2) - W_TO*np.sin(theta_TO))/(WTO*betaw_taxi) * dt_TO
    sair_TO += Vair_TO*np.cos(gamma_TO)*dt_TO
    hdot_TO = Vair_TO*np.sin(gamma_TO)
    h_TO += hdot_TO*dt_TO
    iteration += 1

    # print(Tmax)
    # print(0.5*rho0*Vair_TO**2*S*(CD0_TO+K*CL_TO**2))
    # print(W_TO*np.sin(theta_TO))


if Vair_TO > V2:
    print(f'The function has iterated {iteration} times.\n\nThe final height of the airplane is {h_TO} m.\n\n\
The final airspeed of takeoff is {Vair_TO} m/s.\n\nThe final air distance is {sair_TO} m.')
else:
    print(f'The final airspeed of takeoff is {Vair_TO} and it\'s less than the safety speed V2={V2}. VLOF={VLOF}.')




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



