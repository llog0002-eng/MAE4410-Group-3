import numpy as np
import matplotlib.pyplot as plt

# from flightpath_sim import aircraft_class as aircraft

class Range:
    def __init__(self, aircraft, airspeed, altitude, tolerence, dt):
        with open('ISA.csv', 'r') as file:
            next(file)
            for lines in file:
                elevation = float(lines.strip().split(',')[0])
                if altitude == elevation:
                    self.c = float(lines.strip().split(',')[-1])
                    self.sigma = float(lines.strip().split(',')[3])
                    self.rho = self.sigma*aircraft.rho0
                    break
        file.close()

        self.W_C = aircraft.Wmax*aircraft.betaw_climb
        self.ma = self.W_C / aircraft.g
        self.mf = aircraft.betaw_climb*aircraft.m_fuel
        
        a0_C = aircraft.a0_cruise
        a0d_C = a0_C * (1 + aircraft.av * airspeed/self.c)                   # Airspeed performance correction
        alphae_C = a0d_C * (self.sigma)**aircraft.a                          # Ratio of maximum static thrust or power at sea level to the thrust or power at the desired operating condition, cruise
        T_C = aircraft.Tmax*alphae_C

        
        error = tolerence
        AoA_c = 0
        AoA_p = AoA_c + aircraft.ip
        T = T_C
        while error >= tolerence:
            AoA_p_old = AoA_p
            L = self.W_C - T*np.sin(AoA_p)
            CL = L/(0.5*self.rho*airspeed**2*aircraft.S)
            D = 0.5*self.rho*airspeed**2*aircraft.S*(aircraft.CD0_C+aircraft.KC*CL**2)

            T = D/np.cos(AoA_p)
            AoA_c = (CL-aircraft.CL0_C)/aircraft.CLalpha_C
            AoA_p = AoA_c + aircraft.ip
            
            error = np.abs(AoA_p - AoA_p_old)
        self.T_C = T
        self.AoA_p = AoA_p*180/np.pi

        mn = self.ma
        n = 0
        while mn > self.ma - self.mf:
            mf_dot = aircraft.TSFC_C*10**(-6)*self.T_C
            mn -= mf_dot*dt
            n += 1
        self.m_final = mn
        self.total_time = dt*n
        self.total_range = self.total_time*airspeed
    
    def get_data(self):
        print(f'\nTotal mass at the start of cruise: {self.ma} kg\
\nTotal fuel mass at the start of cruise: {self.mf} kg\
\nTotal mass at the end of cruise: {self.m_final} kg\
\nTotal time: {self.total_time} s = {self.total_time/3600} h\
\nTotal range: {self.total_range/1000} km\
\nTotal thrust during cruise: {self.T_C/1000} kN\
\nAngle of attack during cruise: {self.AoA_p} degrees\n')

