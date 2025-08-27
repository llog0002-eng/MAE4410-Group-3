import matplotlib.pyplot as plt
import numpy as np

### INPUTS
rho = 1.225 # Air density, kg/m3
betaw = 0.8 # Ratio of actual weight to maximum weight
Vmax = 250 # Max speed requirement, m/s
WTO = 127.6 # Takeoff weight, t
hscreen = 50 * 0.3048 # Screen height, 50 ft (FAR 25), m
muwet = 0.05 # Friction coefficient for wet sealed
mudry = 0.03 # Friction coefficient for dry sealed

# AERO DEPENDENT
CD0 = 0.02 # Zero lift drag
K = 0.06 # Induced drag constant
CLmaxclean = 1.8 # Maximum lift coefficient, clean
CLmaxTO = 2.2 # Maximum lift coefficient, takeoff
CLmaxL = 2.7 # Maximum lift coefficient, landing
CLalpha = 2*(np.pi)**2/180 # Lift slope, 1/deg
h = 3 # Distance of wing above the ground, m
b = 65 # Wing span, m
S = 300 # Wing area, m2

# ENGINE DEPENDENT
Tmax = 2 * 189.2e3 # Max thrust for 2x Rolls-Royce RB211-535, N
alphae = 69.2/375.3 # Ratio of maximum static thrust or power at sea level to the thrust or power at the desired operating condition
# source: https://www.kimerius.com/app/download/5781574315/The+GE90+-+An+introduction.pdf

# CONSTANTS
g = 9.80665 # Gravitational acceleration m2/s

### Generate wing loading (x) array
n = 100 # Number of points

# Wing loading
WSstart, WSend = 2000, 8000 # N/m2
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

DS = 1/2 * rho * (0.7 * VLOF)**2 * (CD0 + KIGE * CLmaxTO**2) # Takeoff drag loading
LS = 1/2 * rho * (0.7 * VLOF)**2 * (CLmaxTO)

D = DS * S # Drag force, N
T = Tmax # Thrust, N

# Ground distance estimation

sg = 1/2 * WTO / 1000 * VLOF**2 / (T - D - muwet * (WTO * 1000 * g - LS * S))
PWTOdistgnd = VLOF**2 / (2*g*sg) + muwet + (DS - muwet * LS) * (1/WS)

plt.plot(WS,PWTOdistgnd,label="T/O ground distance")

print(muwet * (WTO * 1000 * g - LS * S))

# Air distance estimation

sa = WTO * 1000 * g /(T - D) * ((V2**2 - VLOF**2) / (2*g) + hscreen)
PWTOdistair = 1/sa * ((V2**2 - VLOF**2)/(2*g) + hscreen) + DS * (1/WS)

plt.plot(WS,PWTOdistair,label="T/O air distance")

### Climb rate requirement


### Service ceiling requirement


### Stall speed requirement


### Plot
plt.xlabel("Wing loading (N/m2)")
plt.ylabel("Power loading (TWR)")
plt.legend()
plt.show()

