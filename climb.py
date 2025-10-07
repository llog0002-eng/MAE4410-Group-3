# from flightpath_sim import aircraft_class as aircraft

import numpy as np
import matplotlib.pyplot as plt
from tabulate import tabulate


class Climb:
    def __init__(self, aircraft, start, end, increment, tolerence, step):
        
        self.start = start
        self.end = end
        self.increment = increment
        self.tol = tolerence
        self.step = step
    
        altitude_list = list(range(start, end+increment, increment))
        with open('ISA.csv', 'r') as file:
            c_list = []
            sigma_list = []
            next(file)
            for line in file:
                altitude = float(line.strip().split(',')[0])
                for num in altitude_list:
                    if altitude == num:
                        c_list.append(float(line.strip().split(',')[-1]))
                        sigma_list.append(float(line.strip().split(',')[3]))
            self.altitude_list = np.array(altitude_list)
            self.c_list = np.array(c_list)
            self.sigma_list = np.array(sigma_list)
        file.close()

        self.W_CL = aircraft.W_CL
        K = aircraft.KTO
        self.Vair_CL = np.arange(aircraft.V2, aircraft.Vmax+step, step)

        a0_CL = 0.9                                                 # Throttle setting in Climb, 0.9-0.95 (Wk5 lecture note page 26)

        gamma_CL_list = []
        theta_CL_list = []
        theta_CL_pct_list = []
        n_list = []
        AoA_list = []
        hdot_list = []
        Thrust_list = []
        Drag_list = []

        for i in range(len(c_list)):
            c = c_list[i]

            gamma_CL_list.append([])
            theta_CL_list.append([])
            theta_CL_pct_list.append([])
            n_list.append([])
            AoA_list.append([])
            hdot_list.append([])
            Thrust_list.append([])
            Drag_list.append([])

            for V in self.Vair_CL:
                error = self.tol
                theta_C = 0
                AoA_C = 10*np.pi/180
                AoA_p = AoA_C + aircraft.ip
                n = 0

                a0d_CL = a0_CL * (1 + aircraft.av * V/c)            # Airspeed performance correction
                alphae_CL = a0d_CL * (sigma_list[i])**aircraft.a    # Ratio of maximum static thrust or power at sea level to the thrust or power at the desired operating condition, takeoff
                T_CL = aircraft.Tmax*alphae_CL
                Thrust_list[-1].append(T_CL)
                
                while error >= self.tol:
                    theta_C_old = theta_C
                    L = self.W_CL*np.cos(theta_C) - T_CL*np.sin(AoA_p)
                    CL = L/(0.5*aircraft.rho0*V**2*aircraft.S)       
                    D = 0.5*aircraft.rho0*V**2*aircraft.S*(aircraft.CD0_CL+K*CL**2)
                    gamma_C = np.arctan( (T_CL*np.cos(AoA_p)-D) / (L+T_CL*np.sin(AoA_p)))

                    AoA_C = (CL-aircraft.CL0_CL)/aircraft.CLalpha_CL
                    AoA_p = AoA_C + aircraft.ip
                    theta_C = gamma_C + AoA_C
                    hdot = V*np.sin(gamma_C)

                    n += 1

                    error = np.abs(theta_C - theta_C_old)
                
                gamma_CL_list[-1].append(gamma_C*180/np.pi)
                theta_CL_list[-1].append(theta_C*180/np.pi)
                theta_CL_pct_list[-1].append(np.tan(theta_C) * 100)
                n_list[-1].append(n)
                AoA_list[-1].append(AoA_p*180/np.pi)
                hdot_list[-1].append(hdot)
                Drag_list[-1].append(D)

        
        self.gamma_CL_list = ["Flight Path Angle $\\gamma$", "Air Speed [m/s]", "$\\gamma$ [degrees]", np.array(gamma_CL_list)]
        self.theta_CL_list = ["$\\theta$", "Air Speed [m/s]", "$\\theta$ [degrees]", np.array(theta_CL_list)]
        self.theta_CL_pct_list = ["Climb Gradient tan($\\theta$)", "Air Speed [m/s]", "tan($\\theta$) [%]", np.array(theta_CL_pct_list)]
        self.n_list = ["FNumber of Iteration", "Air Speed [m/s]", "N [times]", np.array(n_list)]
        self.AoA_list = ["Angle of Attack Angle of Attack $\\alpha$", "Air Speed [m/s]", "$\\alpha$ [degrees]", np.array(AoA_list)]
        self.hdot_list = ["Max R/C", "Air Speed [m/s]", "h_dot [m/s]", np.array(hdot_list)]
        self.Thrust_list = ["Maximum Thrsut at Diff Altitude", "Air Speed [m/s]", "T [N]", np.array(Thrust_list)]
        self.Drag_list = ["Drag at Diff Altitude", "Air Speed [m/s]", "D [N]", np.array(Drag_list)]


        self.max_hdot_list = []
        self.max_theta_list = []
        for i in range(len(c_list)):
            max_hdot = np.max(hdot_list[i])
            max_hdot_index = np.where(hdot_list[i] == max_hdot)
            max_hdot_V = self.Vair_CL[max_hdot_index]
            max_hdot_theta = self.theta_CL_list[-1][i][max_hdot_index]
            max_hdot_gamma = self.gamma_CL_list[-1][i][max_hdot_index]
            max_hdot_AoA = self.AoA_list[-1][i][max_hdot_index]
            self.max_hdot_list.append([self.altitude_list[i], max_hdot, max_hdot_index, max_hdot_V, max_hdot_theta, max_hdot_gamma, max_hdot_AoA])

            max_thetae = np.max(theta_CL_list[i])
            max_theta_index = np.where(theta_CL_list[i] == max_thetae)
            max_theta_v = self.Vair_CL[max_theta_index]
            max_theta_hdot = self.hdot_list[-1][i][max_theta_index]
            max_theta_gamma = self.gamma_CL_list[-1][i][max_theta_index]
            max_theta_AoA = self.AoA_list[-1][i][max_theta_index]
            self.max_theta_list.append([self.altitude_list[i], max_thetae, max_theta_index, max_theta_hdot, max_theta_v, max_theta_gamma, max_theta_AoA])


        Time_CL = []
        Horiz_dis_CL = []
        Time_accel = []
        Horiz_dis_accel = []
        W_fuel_list = []
        V_new = self.max_hdot_list[0][3]
        for i in range(1, len(c_list)):
            # Time to climb and Horizontal distance covered
            max_hdot_index = self.max_hdot_list[i][2]
            max_hdot_V = self.max_hdot_list[i][3]
            max_hdot = self.max_hdot_list[i][1]
            max_hdot_gamma = self.gamma_CL_list[-1][i][max_hdot_index]

            dt_CL = increment/max_hdot
            dSC_CL = max_hdot_V*np.cos(max_hdot_gamma)*dt_CL
            Time_CL.append(dt_CL)
            Horiz_dis_CL.append(dSC_CL)

            # Time to accelerate and Horizontal distance covered
            V_old = V_new
            V_new = max_hdot_V
            dV = np.abs(V_old - V_new)
            T = self.Thrust_list[-1][i][max_hdot_index]
            D = self.Drag_list[-1][i][max_hdot_index]
            dt_accel = self.W_CL/aircraft.g*dV / (T-D)
            dSC_accel = dV*dt_accel/2
            Time_accel.append(dt_accel)
            Horiz_dis_accel.append(dSC_accel)

            mf_dot = aircraft.TSFC_TO*10**(-6)*T
            W_fuel = mf_dot*(dt_CL + dt_accel)*aircraft.g
            self.W_CL -= W_fuel
            W_fuel_list.append(W_fuel)

        self.Time_CL = np.array(Time_CL)
        self.Horiz_dis_CL = np.array(Horiz_dis_CL)
        self.Time_accel = np.array(Time_accel)
        self.Horiz_dis_accel = np.array(Horiz_dis_accel)
        self.W_fuel_list = np.array(W_fuel_list)

        print(f'Fuel consumption during the climb is {np.sum(self.W_fuel_list)/aircraft.g} kg.\
        \nThe final weight is {self.W_CL[0]/aircraft.g} kg\
        \nTotal time is {(np.sum(self.Time_CL) + np.sum(self.Time_accel))/60} mins')

    def get_plot(self, data_list):
        title = data_list[0]
        title_xaxis = data_list[1]
        title_yaxis = data_list[2]
        data = data_list[3]
        plt.figure(title)
        for i in range(len(self.altitude_list)):
            plt.plot(self.Vair_CL, data[i], label = f'{self.altitude_list[i]} m')
        plt.title(title)
        plt.xlabel(title_xaxis)
        plt.ylabel(title_yaxis)
        plt.legend(title = "Altitude")
        # plt.show()
    
    def get_Service_Ceiling(self):
        plt.figure("Service Ceiling")
        for i in range(len(self.max_hdot_list)):
            plt.plot(self.max_hdot_list[i][1], self.altitude_list[i], '.', color = 'blue', ms = '3')
        plt.title("Service Ceiling")
        plt.xlabel("Max Climb Rate [m/s]")
        plt.ylabel("Altitude [m]")
        # plt.show()
    
    def best_RC_data(self):
        data = self.max_hdot_list
        headers = ["Altitude [m]", "Max Climb Rate [m/s]", "Index", "Air Speed [m/s]", "theta [degrees]", "gamma [degrees]", "alpha [degrees]"]
        print(tabulate(data, headers=headers))
    
    def best_AC_data(self):
        data = self.max_theta_list
        headers = ["Altitude [m]", "Max Climb Angle [degress]", "Index", "Climb Rate [m/s]", "Air Speed [m/s]", "gamma [degrees]", "alpha [degrees]"]
        print(tabulate(data, headers=headers))
    
    def get_all_plot(self):
        fig, axs = plt.subplots(3, 3, figsize = (16, 9), constrained_layout = True)

        for i in range(len(self.c_list)):
            axs[0, 0].plot(self.Vair_CL, self.hdot_list[-1][i], label = f'{self.altitude_list[i]} m')
            axs[0, 1].plot(self.Vair_CL, self.theta_CL_pct_list[-1][i])
            axs[0, 2].plot(self.Vair_CL, self.theta_CL_list[-1][i])
            axs[1, 0].plot(self.Vair_CL, self.AoA_list[-1][i])
            axs[1, 1].plot(self.Vair_CL, self.gamma_CL_list[-1][i])
            axs[1, 2].plot(self.Vair_CL, self.Thrust_list[-1][i])
            axs[2, 0].plot(self.Vair_CL, self.n_list[-1][i])

        for i in range(len(self.max_hdot_list)):
            axs[2, 1].plot(self.max_hdot_list[i][1], self.altitude_list[i], '.', color = 'blue', ms = '3')


        fig.canvas.manager.set_window_title("All Plots")

        handles, labels = axs[0, 0].get_legend_handles_labels()
        fig.legend(
            handles, labels,
            loc='upper right',       # position
            ncol=1,                   # number of columns
            title='Altitude',        # legend title
        )


        axs[0, 0].set_title("Max R/C")
        axs[0, 0].set_xlabel("Air speed [m/s]")
        axs[0, 0].set_ylabel("Climb rate h_dot [m/s]")

        axs[0, 1].set_title("Climb Gradient tan($\\theta$)")
        axs[0, 1].set_xlabel("Air speed [m/s]")
        axs[0, 1].set_ylabel("Climb Gradient tan($\\theta$) [%]")

        axs[0, 2].set_title("$\\theta$")
        axs[0, 2].set_xlabel("Air speed [m/s]")
        axs[0, 2].set_ylabel("$\\theta$ [degrees]")

        axs[1, 0].set_title("Angle of Attack $\\alpha$")
        axs[1, 0].set_xlabel("Air speed [m/s]")
        axs[1, 0].set_ylabel("Angle of Attack $\\alpha$ [degrees]")

        axs[1, 1].set_title("Flight Path Angle $\\gamma$")
        axs[1, 1].set_xlabel("Air speed [m/s]")
        axs[1, 1].set_ylabel("Flight Path Angle $\\gamma$ [degrees]")

        axs[1, 2].set_title("Maximum Thrsut at Diff Altitude")
        axs[1, 2].set_xlabel("Air speed [m/s]")
        axs[1, 2].set_ylabel("Thrust [N]")

        axs[2, 0].set_title("Number of Iteration")
        axs[2, 0].set_xlabel("Air speed [m/s]")
        axs[2, 0].set_ylabel("Number of iteration [times]")

        axs[2, 1].set_title("Service Ceiling")
        axs[2, 1].set_xlabel("Max Climb Rate [m/s]")
        axs[2, 1].set_ylabel("Altitude [m]")


        






# plt.show()