import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np

### TODO

# Add different betaw values for the relevant stage of flight
# Add one engine inoperative condition

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
betaw_takeoff = 0.99*0.99
betaw_climb = 0.99*0.99*0.995
betaw_cruise = 0.99*0.99*0.995*0.98
betaw_loiter = 0.99*0.99*0.995*0.98*0.854
betaw_descent = 0.99*0.99*0.995*0.98*0.854*0.991
betaw_landing = 0.99*0.99*0.995*0.98*0.854*0.991*0.99       # Ratio of actual weight to maximum weight, landing

Vmax = Ma*c # Max speed requirement, m/s
V_stall_max = 74.5 # Maximum stall speed, m/s
sTO = 2200 # Maximum overall takeoff distance, m

WTO = 98e3 # Maximum takeoff weight, kg
hscreen = 50 * 0.3048 # Screen height, 50 ft (FAR 25), m
muwet = 0.05 # Friction coefficient for wet sealed
mudry = 0.03 # Friction coefficient for dry sealed
sTO_max = 2200 # Maximum takeoff distance

climbgrad = 3.3/100 # Minimum climb gradient for takeoff https://www.avfacts.com.au/aaos/inst/inst3.PDF
climbgrad_oei_landgear = 0 # Minimum climb gradient for one engine inoperative (OEI) with landing gear extended, CASA part 121, section 9.05
climbgrad_oei_clean = 2/100 # Minimum climb gradient for one engine inoperative (OEI) with landing gear retracted, CASA part 121, section 9.05
hdotceil = 1.27 # Climb rate requirement for ceiling, typical value, m/s

h = 3               # Distance of wing above the ground, m
b = 60.2            # Wing span, m
S = 661.95          # Wing area, m2
g = 9.80665         # Gravitational acceleration m2/s


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
LD_C = 17                    # Target to drag ratio in cruise

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


"""
Engine Dependent
"""
Tmax = 2 * 189.2e3          # Max thrust for 2x Rolls-Royce RB211-535, N
Tmax_continuous = 2 * 156.6e3   # Max continuous thrust for 2x Rolls-Royce RB211-535, N
a0 = 1                      # Default throttle setting
av = -0.3                   # Constant to model decreasing thrust with increasing M
a = 0.7                     # Altitude performance index, between 0.7 and 1 depending on engine altitude optimisation

WSact = WTO*g/S # Actual design wing loading
TWRact = Tmax / (WTO * g) # Actual design thrust to weight ratio
plt.plot(WSact,TWRact,marker = "o", color = "r", ms = 4)
plt.gca().annotate("Design point",(WSact+50, TWRact+0.01))

### Generate wing loading (x) array
n = 1000 # Number of points

# Wing loading
WSstart, WSend = 500, 7000 # N/m2
WS = np.linspace(WSstart, WSend, n)

closest = np.min(np.abs(WS-WSact))
index = np.where(np.abs(WS-WSact) == closest)


"""
Maximum speed requirement
"""
a0_cruise = 0.8 # Throttle setting in cruise
a0d_cruise = a0_cruise * (1 + av * Vmax/c) # Airspeed performance correction
alphae_cruise = a0d_cruise * (rhorho0)**a # Ratio of maximum static thrust or power at sea level to the thrust or power at the desired operating condition, cruise
CL_C = (2 * WS * betaw_cruise) / (rho * Vmax**2) # Lift coefficient, assuming level flight
CD_C = CD0_C + KC*CL_C**2
TWmaxspeed = Tmax/Tmax_continuous * (1/2 * rho * Vmax**2 * CD_C) / alphae_cruise * (1/WS) + ((2 * KC * betaw_taxi) / (alphae_cruise * rho * Vmax**2)) * WS
plt.plot(WS,TWmaxspeed,label="Max speed at altitude")

CL_C_act = (2 * WSact * betaw_cruise) / (rho * Vmax**2) # Lift coefficient, assuming level flight
CD_C_act = CD0_C + KC*CL_C_act**2
Vmax = np.sqrt((TWRact*Tmax_continuous/Tmax*WSact*alphae_cruise)/(CD_C_act*rho) + (np.sqrt(WSact**2*(TWRact**2*alphae_cruise**2-4*CD_C_act*KC*betaw_taxi**2)*rho**2)/(CD_C_act*rho**2)))
Mach_max = Vmax/c
print(f'Vmax = {Vmax:.2f}, Mach = {Mach_max:.2f}')


"""
T/O Distance Requirement
"""
kTO = 1.13
alpha_TO = 2.19 * np.pi / 180                      # TO angle of attack, rad
CL_TO = CL0_TO + CLalpha_TO * alpha_TO          # Lift coefficient at takeoff
V_stall_TO = np.sqrt(2 * WS / (rho0 * CL_TO))   # Stall speed at takeoff, m/s
VR = 1.05 * V_stall_TO                          # Rotate speed, m/s
VLOF = 1.1 * V_stall_TO                         # Lift-off speed, m/s
V2 = 1.13 * V_stall_TO                          # Take-off safety speed, m/s

KIGE = KTO * (16 * (h/b) ** 2) / (1 + 16 * (h/b) ** 2) # McCormick induced drag constant adjusted for ground effect
if KIGE/KTO < 0.5:
    KIGE = 0.5 * KTO

DS = 1/2 * rho0 * (0.7 * VLOF)**2 * (CD0_TO + KIGE * CL_TO**2) # Takeoff drag loading (N/m2)
LS = 1/2 * rho0 * (0.7 * VLOF)**2 * (CL_TO)                    # Takeoff lift loading (N/m2)

D = DS * S      # Drag force, N
T = Tmax        # Thrust, N
L = LS * S      # Lift, N
W = WS * S      # Weight, N

# Air distance requirement
sa = 237 # Air distance (m)
TWTOdistair = 1/sa * ((V2**2 - VLOF**2)/(2*g) + hscreen) + DS * (1/WS)

# Ground distance requirement
sg = sTO_max - sa
TWTOdistgnd = VLOF**2 / (2 * g * sg) + mudry + (DS - mudry * LS) * (1/WS)

plt.plot(WS,TWTOdistgnd,label="T/O ground distance")
plt.plot(WS,TWTOdistair,label="T/O air distance")


"""
Climb Rate Requirement
"""

### Climb rate requirement, one engine inoperative, landing gear not retracted
Vc = V2 # Takeoff climb velocity
CL = np.sqrt(CD0_LA/KTO) # Lift coefficient, assuming max climb rate occurs at max L/D
CD = CD0_LA + KTO * CL ** 2 # Drag coefficient at max L/D
D = 1/2 * rho0 * Vc ** 2 * S * CD # Drag, N
climba = np.arctan(climbgrad_oei_landgear) # Climb angle, rad
hdot = Vc * np.sin(climba)

a0d_SSL = a0 * (1 + av * Vc/c0) # Airspeed performance correction
alphae_SSL = a0d_SSL # Ratio of maximum static thrust or power at sea level to the thrust or power at the desired operating condition, SSL

TWClimbSSL_OEI_LG = betaw/alphae_SSL * (hdot/Vc + rho0*Vc**2*CD0_LA/(2*betaw*WS) + 2*KTO*betaw/(rho0*Vc**2)*WS)

plt.plot(WS,TWClimbSSL_OEI_LG,label="Climb rate, OEI, landing gear extended")

### Climb rate requirement, one engine inoperative, landing gear retracted
Vc = V2 # Takeoff climb velocity
CL = np.sqrt(CD0_C/KTO) # Lift coefficient, assuming max climb rate occurs at max L/D
CD = CD0_C + KTO * CL ** 2 # Drag coefficient at max L/D
D = 1/2 * rho0 * Vc ** 2 * S * CD # Drag, N
climba = np.arctan(climbgrad_oei_clean) # Climb angle, rad
hdot = Vc * np.sin(climba)

a0d_SSL = a0 * (1 + av * Vc/c0) # Airspeed performance correction
alphae_SSL = a0d_SSL # Ratio of maximum static thrust or power at sea level to the thrust or power at the desired operating condition, SSL

TWClimbSSL_OEI_clean = betaw/alphae_SSL * (hdot/Vc + rho0*Vc**2*CD0_C/(2*betaw*WS) + 2*KTO*betaw/(rho0*Vc**2)*WS)

plt.plot(WS,TWClimbSSL_OEI_clean,label="Climb rate, OEI, landing gear retracted")


"""
Service Ceiling Requirement
    Determing whether a certain airspeed can be maintained at the desired altitude
"""
Vc = np.sqrt((2*betaw_climb)/(rho*np.sqrt(CD0_C/KC))*WS) # Optimal climb velocity
# CL = np.sqrt(CD0_C/K) # Lift coefficient, assuming max climb rate occurs at max L/D
# CD = CD0_C + K * CL ** 2 # Drag coefficient at max L/D
# D = 1/2 * rho * Vc ** 2 * S * CD # Drag, N

a0d_ceil = a0 * (1 + av * Vc/c) # Airspeed performance correction
alphae_ceil = a0d_ceil * (rhorho0)**a # Ratio of maximum static thrust or power at sea level to the thrust or power at the desired operating condition, ceiling

TWClimbceil = 0.5*rho*Vmax**2*CD0_C/alphae_ceil/WS + 2*KC*betaw_climb**2*WS/(alphae_ceil*rho*Vmax**2)

# plt.plot(WS,TWClimbSSL,label="Climb (sea level)")
plt.plot(WS,TWClimbceil,label="Service ceiling climb rate")

""""
Stall speed requirement
"""

CL_L = 1.5     # Lift coefficient at landing
WS_stall_speed = 1/2 * rho0 * V_stall_max ** 2 * CL_L / betaw_descent # Wing loading for stall requirement
print(WS_stall_speed)
plt.axvline(WS_stall_speed, label = "Stall speed", color = "pink")

"""
Plotting
"""

plt.title("Constraint Analysis")
plt.xlabel("Wing loading (N/m2)")
plt.ylabel("Thrust-to-Weight Ratio")
plt.legend()

fig = plt.gcf()
ax = fig.get_axes()[0]
ax.set_ylim((0,1))
ylim = ax.get_ylim()
y1 = np.maximum.reduce([TWmaxspeed,TWTOdistgnd,TWTOdistair,TWClimbceil, TWClimbSSL_OEI_LG, TWClimbSSL_OEI_clean])
plt.fill_between(WS, y1, y2=ylim[1], where = (WS<=WS_stall_speed), alpha = 0.2, color = "g")
plt.margins(x=0, y=0)

### Plot
plt.xlabel("Wing loading (N/m2)")
plt.ylabel("Thrust to weight ratio (TWR)")
plt.legend(loc = "upper right")
plt.ylim((0,1))

mpl.rc("savefig", dpi=300)
plt.savefig("../../Constraint Analysis.png", bbox_inches='tight')

plt.show()

