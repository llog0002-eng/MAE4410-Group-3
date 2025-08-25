import matplotlib.pyplot as plt
import numpy as np

### TODO

# Add different betaw values for the relevant stage of flight

### INPUTS
rho0 = 1.225 # Sea level air density, kg/m3
rhorho0 = 0.2971 # Ratio of air density and sea level air density
rho = rhorho0 * rho0 # Operating altitude air density, kg/m3
betaw = 1 # Ratio of actual weight to maximum weight
Vmax = 250 # Max speed requirement, m/s
WTO = 127.6e3 # Maximum takeoff weight, kg
hscreen = 50 * 0.3048 # Screen height, 50 ft (FAR 25), m
muwet = 0.05 # Friction coefficient for wet sealed
mudry = 0.03 # Friction coefficient for dry sealed
c = 295.2 # Speed of sound at 11km, m/s

# AERO DEPENDENT
CD0 = 0.02 # Zero lift drag
K = 0.06 # Induced drag constant
CLmaxclean = 1.8 # Maximum lift coefficient, clean
CLmaxTO = 2.2 # Maximum lift coefficient, takeoff
CL0TO = 0.3 # Zero AoA lift coefficient, takeoff
CLmaxL = 2.7 # Maximum lift coefficient, landing
CLalpha = 2*(np.pi)**2/180 # Lift slope, 1/deg
h = 3 # Distance of wing above the ground, m
b = 65 # Wing span, m
S = 300 # Wing area, m2

# ENGINE DEPENDENT
Tmax = 2 * 189.2e3 # Max thrust for 2x Rolls-Royce RB211-535, N
a0 = 1 # Throttle setting
av = -0.3 # Constant to model decreasing thrust with increasing M
a0d = a0 * (1 + av * Vmax/c) # Airspeed performance correction
a = 1 # Altitude performance index, between 0.7 and 1 depending on engine altitude optimisation
alphae = a0d * (rhorho0)**a # Ratio of maximum static thrust or power at sea level to the thrust or power at the desired operating condition

# CONSTANTS
g = 9.80665 # Gravitational acceleration m2/s

### Generate wing loading (x) array
n = 100 # Number of points

# Wing loading
WSstart, WSend = 500, 8000 # N/m2
WS = np.linspace(WSstart, WSend, n)

### Maximum speed requirement
PWmaxspeed = (1/2 * rho * Vmax**2 * CD0) / alphae * (1/WS) + ((2 * K * betaw**2) / (alphae * rho * Vmax**2)) * WS
plt.plot(WS,PWmaxspeed,label="Max speed at altitude")

### T/O distance requirement
VstallTO = np.sqrt(2 * WS / (rho * CLmaxTO)) # Stall speed at takeoff, m/s
VR = 1.05 * VstallTO # Rotate speed, m/s
VLOF = 1.1 * VstallTO # Lift-off speed, m/s
V2 = 1.13 * VstallTO # Take-off safety speed, m/s

####################### REPLACE CLmaxTO WITH CL0 + CL_alpha * alpha IF NOT TAKING OFF AT MAX CL
KIGE = K * (16 * (h/b) ** 2) / (1 + 16 * (h/b) ** 2) # McCormick induced drag constant adjusted for ground effect
if KIGE/K < 0.5:
    KIGE = 0.5 * K

DS = 1/2 * rho0 * (0.7 * VLOF)**2 * (CD0 + KIGE * CL0TO**2) # Takeoff drag loading
LS = 1/2 * rho0 * (0.7 * VLOF)**2 * (CL0TO)

D = DS * S # Drag force, N
T = Tmax # Thrust, N

# Ground distance estimation
sg = 1/2 * WS * S * VLOF**2 / (T - D - muwet * (WS * S * g - LS * S)) # Ground distance length
PWTOdistgnd = VLOF**2 / (2*g*sg) + muwet + (DS - muwet * LS) * (1/WS)

plt.plot(WS,PWTOdistgnd,label="T/O ground distance")

# Air distance estimation
sa = WS * S * g /(T - D) * ((V2**2 - VLOF**2) / (2*g) + hscreen) # Air distance length
PWTOdistair = 1/sa * ((V2**2 - VLOF**2)/(2*g) + hscreen) + DS * (1/WS)

plt.plot(WS,PWTOdistair,label="T/O air distance")

### Climb rate requirement

# SSL
Vc = np.sqrt((2*betaw)/(rho0*np.sqrt(CD0/K))*WS[50]) # Optimal climb velocity
CL = np.sqrt(CD0/K) # Lift coefficient, assuming max climb rate occurs at max L/D
CD = CD0 + K * CL ** 2 # Drag coefficient at max L/D
D = 1/2 * rho0 * Vc ** 2 * S * CD # Drag, N
hdot = Vc * (Tmax - D) / (betaw * WS * S)
print(Vc)
print(np.sqrt(2 * WS / (rho * CL)))
PWClimbSSL = betaw * (hdot/Vc + rho0*Vc**2*CD0/(2*betaw*WS) + 2*K*betaw/(rho0*Vc**2)*WS)

# Ceiling
Vc = np.sqrt((2*betaw)/(rho*np.sqrt(CD0/K))*WS[50]) # Optimal climb velocity
CL = np.sqrt(CD0/K) # Lift coefficient, assuming max climb rate occurs at max L/D
CD = CD0 + K * CL ** 2 # Drag coefficient at max L/D
D = 1/2 * rho * Vc ** 2 * S * CD # Drag, N
hdot = Vc * (Tmax - D) / (betaw * WS * S)
print(Vc)
PWClimbceil = betaw/alphae * (hdot/Vc + rho*Vc**2*CD0/(2*betaw*WS) + 2*K*betaw/(rho*Vc**2)*WS)

plt.plot(WS,PWClimbSSL,label="Climb (sea level)")
plt.plot(WS,PWClimbceil,label="Climb (ceiling)")

### Service ceiling requirement


### Stall speed requirement


### Plot
plt.xlabel("Wing loading (N/m2)")
plt.ylabel("Power loading (TWR)")
plt.legend()
plt.show()

