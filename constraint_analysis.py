import matplotlib.pyplot as plt
import numpy as np

### TODO

# Add different betaw values for the relevant stage of flight
# Add one engine inoperative condition

### INPUTS
rho0 = 1.225 # Sea level air density, kg/m3
rhorho0 = 0.2971 # Ratio of air density and sea level air density
rho = rhorho0 * rho0 # Operating altitude air density, kg/m3
c = 295.2 # Speed of sound at 11km, m/s
Ma = 0.85

betaw = 1 # Ratio of actual weight to maximum weight
betaw_landing = 0.7 # Ratio of actual weight to maximum weight, landing

Vmax = Ma*c # Max speed requirement, m/s
V_stall_max = 60 # Maximum stall speed, m/s

WTO = 127.6e3 # Maximum takeoff weight, kg
hscreen = 50 * 0.3048 # Screen height, 50 ft (FAR 25), m
muwet = 0.05 # Friction coefficient for wet sealed
mudry = 0.03 # Friction coefficient for dry sealed

climbgrad = 3.3/100 # Minimum climb gradient for takeoff https://www.avfacts.com.au/aaos/inst/inst3.PDF
climbgrad_oei_landgear = 0 # Minimum climb gradient for one engine inoperative (OEI) with landing gear extended, CASA part 121, section 9.05
climbgrad_oei_clean = 2/100 # Minimum climb gradient for one engine inoperative (OEI) with landing gear retracted, CASA part 121, section 9.05
hdotceil = 1.27 # Climb rate requirement for ceiling, typical value, m/s

# AERO DEPENDENT
CD0 = 0.0064 # Zero lift drag
CD0_landing_gear = 0.04 # Zero lift drag, landing gear extended
K = 0.0734 # Induced drag constant

h = 3 # Distance of wing above the ground, m
b = 65 # Wing span, m
S = 768.39 # Wing area, m2

CLcruise = 0.148 
CLmaxclean = 1.8 # Maximum lift coefficient, clean
CLmaxTO = 0.949 # Maximum lift coefficient, takeoff
CL0TO = 0.3 # Zero AoA lift coefficient, takeoff
CLmaxL = 0.63 # Maximum lift coefficient, landing

CLalpha = 2*(np.pi)**2/180 # Lift slope, 1/deg
h = 3 # Distance of wing above the ground, m
b = 65 # Wing span, m
S = 768.395 # Wing area, m2

# ENGINE DEPENDENT
Tmax = 2 * 189.2e3 # Max thrust for 2x Rolls-Royce RB211-535, N
a0 = 1 # Default throttle setting
av = -0.3 # Constant to model decreasing thrust with increasing M
a = 1 # Altitude performance index, between 0.7 and 1 depending on engine altitude optimisation

# CONSTANTS
g = 9.80665 # Gravitational acceleration m2/s

WSact = 4500 # Actual design wing loading
TWRact = Tmax / (WTO * g) # Actual design thrust to weight ratio
plt.plot(WSact,TWRact,marker = "o", color = "r", ms = 4)
plt.gca().annotate("Design point",(WSact+50, TWRact+0.01))

### Generate wing loading (x) array
n = 100 # Number of points

# Wing loading
WSstart, WSend = 2000, 7000 # N/m2
WS = np.linspace(WSstart, WSend, n)

### Maximum speed requirement
a0_cruise = 0.8 # Throttle setting in cruise
a0d_cruise = a0_cruise * (1 + av * Vmax/c) # Airspeed performance correction
alphae_cruise = a0d_cruise * (rhorho0)**a # Ratio of maximum static thrust or power at sea level to the thrust or power at the desired operating condition, cruise
CD = CD0 + K*CLcruise**2
TWmaxspeed = (1/2 * rho * Vmax**2 * CD) / alphae_cruise * (1/WS) + ((2 * K * betaw) / (alphae_cruise * rho * Vmax**2)) * WS
plt.plot(WS,TWmaxspeed,label="Max speed at altitude")

print(CLcruise/2*rho*Vmax**2*S/(WTO*g))

### T/O distance requirement
V_stall_TO = np.sqrt(2 * WS / (rho0 * CLmaxTO)) # Stall speed at takeoff, m/s
VR = 1.05 * V_stall_TO # Rotate speed, m/s
VLOF = 1.1 * V_stall_TO # Lift-off speed, m/s
V2 = 1.13 * V_stall_TO # Take-off safety speed, m/s

####################### REPLACE CLmaxTO WITH CL0 + CL_alpha * alpha IF NOT TAKING OFF AT MAX CL
KIGE = K * (16 * (h/b) ** 2) / (1 + 16 * (h/b) ** 2) # McCormick induced drag constant adjusted for ground effect
if KIGE/K < 0.5:
    KIGE = 0.5 * K

DS = 1/2 * rho0 * (0.7 * VLOF)**2 * (CD0_landing_gear + KIGE * CL0TO**2) # Takeoff drag loading
LS = 1/2 * rho0 * (0.7 * VLOF)**2 * (CL0TO)

D = DS * S # Drag force, N
T = Tmax # Thrust, N
L = LS * S # Lift, N
W = WS * S # Weight, N

# Ground distance estimation
sg = W * VLOF**2 / (2 * g *(T - D - muwet * (W - LS * S))) # Ground distance length
TWTOdistgnd = VLOF**2 / (2 * g * sg) + muwet + (DS - muwet * LS) * (1/WS)

plt.plot(WS,TWTOdistgnd,label="T/O ground distance")

# Air distance estimation
DS_air = 1/2 * rho0 * ((V2 + VLOF)/2)**2 * (CD0 + KIGE * CL0TO**2) # Takeoff air drag loading
sa = W /(T - D) * ((V2**2 - VLOF**2) / (2*g) + hscreen) # Air distance length
TWTOdistair = 1/sa * ((V2**2 - VLOF**2)/(2*g) + hscreen) + DS_air * (1/WS)

plt.plot(WS,TWTOdistair,label="T/O air distance")

### Climb rate requirement
Vc = np.sqrt((2*betaw)/(rho0*np.sqrt(CD0/K))*WS) # Optimal climb velocity
CL = np.sqrt(CD0/K) # Lift coefficient, assuming max climb rate occurs at max L/D
CD = CD0 + K * CL ** 2 # Drag coefficient at max L/D
D = 1/2 * rho0 * Vc ** 2 * S * CD # Drag, N
climba = np.arctan(climbgrad) # Climb angle, rad
hdot = Vc * np.sin(climba)

a0d_SSL = a0 * (1 + av * Vc/c) # Airspeed performance correction
alphae_SSL = a0d_SSL # Ratio of maximum static thrust or power at sea level to the thrust or power at the desired operating condition, SSL

TWClimbSSL = betaw/alphae_SSL * (hdot/Vc + rho0*Vc**2*CD0/(2*betaw*WS) + 2*K*betaw/(rho0*Vc**2)*WS)

plt.plot(WS,TWClimbSSL,label="Climb rate")

### Climb rate requirement, one engine inoperative, landing gear not retracted
Vc = np.sqrt((2*betaw)/(rho0*np.sqrt(CD0_landing_gear/K))*WS) # Optimal climb velocity
CL = np.sqrt(CD0/K) # Lift coefficient, assuming max climb rate occurs at max L/D
CD = CD0 + K * CL ** 2 # Drag coefficient at max L/D
D = 1/2 * rho0 * Vc ** 2 * S * CD # Drag, N
climba = np.arctan(climbgrad_oei_landgear) # Climb angle, rad
hdot = Vc * np.sin(climba)

a0d_SSL = a0 * (1 + av * Vc/c) # Airspeed performance correction
alphae_SSL = a0d_SSL # Ratio of maximum static thrust or power at sea level to the thrust or power at the desired operating condition, SSL

TWClimbSSL_OEI_LG = betaw/alphae_SSL * (hdot/Vc + rho0*Vc**2*CD0/(2*betaw*WS) + 2*K*betaw/(rho0*Vc**2)*WS)

plt.plot(WS,TWClimbSSL_OEI_LG,label="Climb rate, OEI landing gear extended")

### Climb rate requirement, one engine inoperative, landing gear retracted
Vc = np.sqrt((2*betaw)/(rho0*np.sqrt(CD0/K))*WS) # Optimal climb velocity
CL = np.sqrt(CD0/K) # Lift coefficient, assuming max climb rate occurs at max L/D
CD = CD0 + K * CL ** 2 # Drag coefficient at max L/D
D = 1/2 * rho0 * Vc ** 2 * S * CD # Drag, N
climba = np.arctan(climbgrad_oei_clean) # Climb angle, rad
hdot = Vc * np.sin(climba)

a0d_SSL = a0 * (1 + av * Vc/c) # Airspeed performance correction
alphae_SSL = a0d_SSL # Ratio of maximum static thrust or power at sea level to the thrust or power at the desired operating condition, SSL

TWClimbSSL_OEI_clean = betaw/alphae_SSL * (hdot/Vc + rho0*Vc**2*CD0/(2*betaw*WS) + 2*K*betaw/(rho0*Vc**2)*WS)

plt.plot(WS,TWClimbSSL_OEI_clean,label="Climb rate, OEI landing gear retracted")

### Service ceiling requirement
Vc = np.sqrt((2*betaw)/(rho*np.sqrt(CD0/K))*WS) # Optimal climb velocity
CL = np.sqrt(CD0/K) # Lift coefficient, assuming max climb rate occurs at max L/D
CD = CD0 + K * CL ** 2 # Drag coefficient at max L/D
D = 1/2 * rho * Vc ** 2 * S * CD # Drag, N

a0d_ceil = a0 * (1 + av * Vc/c) # Airspeed performance correction
alphae_ceil = a0d_ceil * (rhorho0)**a # Ratio of maximum static thrust or power at sea level to the thrust or power at the desired operating condition, ceiling

TWClimbceil = betaw/alphae_ceil * (hdotceil/Vc + rho*Vc**2*CD0/(2*betaw*WS) + 2*K*betaw/(rho*Vc**2)*WS)

plt.plot(WS,TWClimbSSL,label="Climb (sea level)")
plt.plot(WS,TWClimbceil,label="Climb (ceiling)")

### Stall speed requirement

WS_stall_speed = rho0 * V_stall_max ** 2 * CLmaxL / betaw_landing # Wing loading for stall requirement

# Stall speed requirement largely irrelevant...
# plt.axvline(WS_stall_speed, label = "Stall speed")

### Plot
plt.title("Constraint Analysis")
plt.xlabel("Wing loading (N/m2)")
plt.ylabel("Thrust-to-Weight Ratio")
plt.legend()

fig = plt.gcf()
ax = fig.get_axes()[0]
ax.set_ylim(bottom=0)
ylim = ax.get_ylim()
y1 = np.maximum.reduce([TWmaxspeed,TWTOdistgnd,TWTOdistair,TWClimbSSL,TWClimbceil,TWClimbSSL_OEI_clean,TWClimbSSL_OEI_LG])
plt.fill_between(WS, y1, y2=ylim[1], alpha = 0.2, color = "g")
plt.margins(x=0, y=0)

### Plot
plt.xlabel("Wing loading (N/m2)")
plt.ylabel("Power loading (TWR)")
plt.legend(loc = "upper right")
plt.show()

