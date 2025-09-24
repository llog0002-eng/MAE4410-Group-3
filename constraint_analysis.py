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
Ma = 0.85 # Cruise speed, Ma

betaw = 1                   # Ratio of actual weight to maximum weight
betaw_taxi = 0.99
betaw_takeoff = 0.99*0.99
betaw_climb = 0.99*0.99*0.995
betaw_cruise = 0.99*0.99*0.995*0.98
betaw_loiter = 0.99*0.99*0.995*0.98*0.854
betaw_descent = 0.99*0.99*0.995*0.98*0.854*0.991
betaw_landing = 0.99*0.99*0.995*0.98*0.854*0.991*0.99       # Ratio of actual weight to maximum weight, landing

Vmax = Ma*c # Max speed requirement, m/s
print(Vmax)
V_stall_max = 74.5 # Maximum stall speed, m/s
sTO = 2200 # Maximum overall takeoff distance, m

WTO = 99.9e3 # Maximum takeoff weight, kg
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
Aerodynamics Dependent
"""
K = 0.0646              # Induced drag constant

### Take Off (AoA = 10 deg)
CL_TO = 0.7821
CL0_TO = 0.2201
CD_TO = 0.0517
CD0_TO = 0.0106

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
Engine Dependent
"""
Tmax = 2 * 264.4e3        # Max thrust for 2x Rolls-Royce RB211-535, N
a0 = 1                  # Default throttle setting
av = -0.3               # Constant to model decreasing thrust with increasing M
a = 0.7                 # Altitude performance index, between 0.7 and 1 depending on engine altitude optimisation



WSact = WTO*g/S # Actual design wing loading
TWRact = Tmax / (WTO * g) # Actual design thrust to weight ratio
plt.plot(WSact,TWRact,marker = "o", color = "r", ms = 4)
plt.gca().annotate("Design point",(WSact+50, TWRact+0.01))

### Generate wing loading (x) array
n = 10000 # Number of points

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
CD = CD0_C + K*CL_C**2
TWmaxspeed = (1/2 * rho * Vmax**2 * CD) / alphae_cruise * (1/WS) + ((2 * K * betaw_taxi) / (alphae_cruise * rho * Vmax**2)) * WS
plt.plot(WS,TWmaxspeed,label="Max speed at altitude")


Vmax = np.sqrt((TWRact*WSact*alphae_cruise)/(CD*rho) + (np.sqrt(WSact**2*(TWRact**2*alphae_cruise**2-4*CD*K*betaw_taxi**2)*rho**2)/(CD*rho**2)))
Mach_max = Vmax/c
print(f'Vmax = {Vmax}, Mach = {Mach_max}')


"""
T/O Distance Requirement
"""
kTO = 1.13
V_stall_TO = np.sqrt(2 * WS / (rho0 * CL_TO))   # Stall speed at takeoff, m/s
VR = 1.05 * V_stall_TO                          # Rotate speed, m/s
VLOF = 1.1 * V_stall_TO                         # Lift-off speed, m/s
V2 = 1.13 * V_stall_TO                          # Take-off safety speed, m/s

KIGE = K * (16 * (h/b) ** 2) / (1 + 16 * (h/b) ** 2) # McCormick induced drag constant adjusted for ground effect
if KIGE/K < 0.5:
    KIGE = 0.5 * K

DS = 1/2 * rho0 * (0.7 * VLOF)**2 * (CD0_TO + KIGE * CL_TO**2) # Takeoff drag loading (N/m2)
LS = 1/2 * rho0 * (0.7 * VLOF)**2 * (CL_TO)                    # Takeoff lift loading (N/m2)

D = DS * S      # Drag force, N
T = Tmax        # Thrust, N
L = LS * S      # Lift, N
W = WS * S      # Weight, N

# Air distance requirement
sa = 415 # Air distance (m)
TWTOdistair = 1/sa * ((V2**2 - VLOF**2)/(2*g) + hscreen) + DS * (1/WS)

# Ground distance requirement
sg = sTO_max - sa
TWTOdistgnd = VLOF**2 / (2 * g * sg) + mudry + (DS - mudry * LS) * (1/WS)

# Calculate distances for design point
sg_act = 0.5 * WTO * VLOF[index]**2 / (Tmax - D[index] - mudry * (WTO * g - L[index]))
sa_act = WSact * S / (Tmax - D[index]) * ((V2[index]**2 - VLOF[index]**2)/(2*g) + hscreen)

print(f"Takeoff ground distance at design point: {sg_act[0]} m")
print(f"Takeoff air distance at design point: {sa_act[0]} m")
print(f'Total takeoff distance at design point: {sg_act[0]+sa_act[0]}')

plt.plot(WS,TWTOdistgnd,label="T/O ground distance")
plt.plot(WS,TWTOdistair,label="T/O air distance")

"""
Climb Rate Requirement
"""
Vc = np.sqrt((2*betaw_takeoff)/(rho0*np.sqrt(CD0_TO/K))*WS) # Optimal climb velocity
# CL = np.sqrt(CD0/K) # Lift coefficient, assuming max climb rate occurs at max L/D
# CD = CD0 + K * CL ** 2 # Drag coefficient at max L/D
# D = 1/2 * rho0 * Vc ** 2 * S * CD # Drag, N
climba = np.arctan(climbgrad) # Climb angle, rad
hdot = Vc * np.sin(climba)

# a0d_SSL = a0 * (1 + av * V2/c0) # Airspeed performance correction
# alphae_SSL = a0d_SSL # Ratio of maximum static thrust or power at sea level to the thrust or power at the desired operating condition, SSL

# TWClimbSSL = betaw_takeoff/alphae_SSL * (hdot/Vc + rho0*Vc**2*CD0_TO/(2*betaw_takeoff*WS) + 2*K*betaw_takeoff/(rho0*Vc**2)*WS)

# plt.plot(WS,TWClimbSSL,label="Climb rate (sea level)")



### Climb rate requirement, one engine inoperative, landing gear not retracted
Vc = V2 # Takeoff climb velocity
CL = np.sqrt(CD0_L/K) # Lift coefficient, assuming max climb rate occurs at max L/D
CD = CD0_L + K * CL ** 2 # Drag coefficient at max L/D
D = 1/2 * rho0 * Vc ** 2 * S * CD # Drag, N
climba = np.arctan(climbgrad_oei_landgear) # Climb angle, rad
hdot = Vc * np.sin(climba)

a0d_SSL = a0 * (1 + av * Vc/c0) # Airspeed performance correction
alphae_SSL = a0d_SSL # Ratio of maximum static thrust or power at sea level to the thrust or power at the desired operating condition, SSL

TWClimbSSL_OEI_LG = betaw/alphae_SSL * (hdot/Vc + rho0*Vc**2*CD0_L/(2*betaw*WS) + 2*K*betaw/(rho0*Vc**2)*WS)

plt.plot(WS,TWClimbSSL_OEI_LG,label="Climb rate, OEI, landing gear extended")

### Climb rate requirement, one engine inoperative, landing gear retracted
Vc = V2 # Takeoff climb velocity
CL = np.sqrt(CD0_C/K) # Lift coefficient, assuming max climb rate occurs at max L/D
CD = CD0_C + K * CL ** 2 # Drag coefficient at max L/D
D = 1/2 * rho0 * Vc ** 2 * S * CD # Drag, N
climba = np.arctan(climbgrad_oei_clean) # Climb angle, rad
hdot = Vc * np.sin(climba)

a0d_SSL = a0 * (1 + av * Vc/c0) # Airspeed performance correction
alphae_SSL = a0d_SSL # Ratio of maximum static thrust or power at sea level to the thrust or power at the desired operating condition, SSL

TWClimbSSL_OEI_clean = betaw/alphae_SSL * (hdot/Vc + rho0*Vc**2*CD0_C/(2*betaw*WS) + 2*K*betaw/(rho0*Vc**2)*WS)

plt.plot(WS,TWClimbSSL_OEI_clean,label="Climb rate, OEI, landing gear retracted")



"""
Service Ceiling Requirement
    Determing whether a certain airspeed can be maintained at the desired altitude
"""
Vc = np.sqrt((2*betaw_climb)/(rho*np.sqrt(CD0_C/K))*WS) # Optimal climb velocity
# CL = np.sqrt(CD0_C/K) # Lift coefficient, assuming max climb rate occurs at max L/D
# CD = CD0_C + K * CL ** 2 # Drag coefficient at max L/D
# D = 1/2 * rho * Vc ** 2 * S * CD # Drag, N

a0d_ceil = a0 * (1 + av * Vc/c) # Airspeed performance correction
alphae_ceil = a0d_ceil * (rhorho0)**a # Ratio of maximum static thrust or power at sea level to the thrust or power at the desired operating condition, ceiling

TWClimbceil = 0.5*rho*Vmax**2*CD0_C/alphae_ceil/WS + 2*K*betaw_climb**2*WS/(alphae_ceil*rho*Vmax**2)

# plt.plot(WS,TWClimbSSL,label="Climb (sea level)")
plt.plot(WS,TWClimbceil,label="Service ceiling climb rate")


""""
Stall speed requirement
"""

WS_stall_speed = rho0 * V_stall_max ** 2 * CL_L / betaw_descent # Wing loading for stall requirement

plt.axvline(WS_stall_speed, label = "Stall speed", color = "pink")


"""
Plotting
"""

mpl.rc("savefig", dpi=300)

plt.title("Constraint Analysis")
plt.xlabel("Wing loading (N/m2)")
plt.ylabel("Thrust-to-Weight Ratio")
plt.legend()

fig = plt.gcf()
ax = fig.get_axes()[0]
ax.set_ylim(bottom=0)
ylim = ax.get_ylim()
y1 = np.maximum.reduce([TWmaxspeed,TWTOdistgnd,TWTOdistair,TWClimbceil, TWClimbSSL_OEI_LG, TWClimbSSL_OEI_clean])
plt.fill_between(WS, y1, y2=ylim[1], where = (WS<=WS_stall_speed), alpha = 0.2, color = "g")
plt.margins(x=0, y=0)

### Plot
plt.xlabel("Wing loading (N/m2)")
plt.ylabel("Thrust to weight ratio (TWR)")
plt.legend(loc = "upper right")
plt.ylim((0,1))
plt.show()

