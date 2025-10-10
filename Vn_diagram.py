import numpy as np
import matplotlib.pyplot as plt


def Vn_diagram(aircraft, step):
    VS1 = aircraft.V_stall_TO        # Stall velocity at takeoff, m/s
    # VS0 = 
    Uref_array = np.array([56 / 3.281, 28 / 3.281])      # m/s
    W_array = np.array([aircraft.Wmax * aircraft.betaw_taxi, aircraft.Wmax * aircraft.betaw_descent])
    S = aircraft.S
    VC = aircraft.Vmax
    VD = 1.25*VC
    rho = aircraft.rho0
    c = aircraft.c
    g = aircraft.g
    n_pos_limit = aircraft.n_pos_limit
    n_neg_limit = aircraft.n_neg_limit
    CLalpha_array = np.array([aircraft.CLalpha_TO, aircraft.CLalpha_LA])

    n = len(W_array)
    VB_array = np.zeros(n)
    VA_array = np.zeros(n)
    Kg_array = np.zeros(n)

    for i in range(n):
        W = W_array[i]
        mu = 2 * (W/S) / (rho*c*g*CLalpha_array[i])
        CLalpha = CLalpha_array[i]
        Uref = Uref_array[i]

        Kg = 0.88*mu / (5.3+mu)
        VB = VS1 * (1 + (Kg*Uref*VC*CLalpha)/(0.2496* (W/S)) )**0.5
        VA = VS1*np.sqrt(n_pos_limit)
        VB_array[i] = VB
        VA_array[i] = VA
        Kg_array[i] = Kg












    CL_max = aircraft.CL_max_TO
    CL_min = aircraft.CL_min_TO
    V_corner_pos = np.sqrt(2 * W_array[0] * n_pos_limit / (rho * S * CL_max))
    V_corner_neg = np.sqrt(2 * W_array[0] * n_neg_limit / (rho * S * CL_min))



    Vair_array = lambda end: np.arange(0, end+step, step)
    f_nVCD_pos = lambda i, speed: 1 + (Kg_array[i]*rho*speed*Uref_array[i]*CLalpha_array[i]) / (2* W_array[i] / S)
    f_nVCD_neg = lambda i, speed: 1 - (Kg_array[i]*rho*speed*Uref_array[i]*CLalpha_array[i]) / (2* W_array[i] / S)

    # Stall Velocity
    vair_stall_pos = Vair_array(VA_array[0])
    nstall_pos_list = 0.5 * rho * vair_stall_pos**2 * CL_max / (W_array[0] / S)

    vair_stall_neg = Vair_array(VA_array[0])
    nstall_neg_list = 0.5 * rho * vair_stall_neg**2 * CL_min / (W_array[0] / S)


    # VC
    vair_VC_pos = Vair_array(V_corner_pos)
    nVC_pos = f_nVCD_pos(0, vair_VC_pos)

    vair_VC_neg = Vair_array(V_corner_neg)
    nVC_neg = f_nVCD_neg(0, vair_VC_neg)


    # VD
    vair_VD_pos = Vair_array(VD)
    nVD_pos = f_nVCD_pos(1, vair_VD_pos)

    vair_VD_neg = Vair_array(VD)
    nVD_neg = f_nVCD_neg(1, vair_VD_neg)
    





    plt.figure("V-n Diagram", figsize=(10, 6))

    plt.plot(vair_stall_pos, nstall_pos_list, label = "Positive Stall Limit")
    plt.plot(vair_stall_neg, nstall_neg_list, label = "Negative Stall Limit")
    plt.plot(vair_VC_pos, nVC_pos, '--', label = "+VC gust line")
    plt.plot(vair_VD_pos, nVD_pos, '--', label = "+VD gust line")
    plt.plot(vair_VC_neg, nVC_neg, '--', label = "-VC gust line")
    plt.plot(vair_VD_neg, nVD_neg, '--', label = "-VD gust line")


    # Full limit lines
    plt.axvline(x = VD, color = 'k', linestyle = '-', label = "V_D")
    plt.axvline(x = VS1, ymin = 2/5, ymax = 0.495, color = 'k', linestyle = '-')
    plt.text(VS1+1, 0.2, 'VS1')
    plt.text(VA_array[0], 1, 'VA')
    plt.text(V_corner_pos, 1, 'V_corner')
    plt.axhline(y=nstall_pos_list[-1], xmin=vair_stall_pos[-1]/(50+VD), xmax=VD/(50+VD), color='r', linestyle='-', label="+Structural limit")
    plt.axhline(y=nstall_neg_list[-1], xmin=vair_stall_neg[-1]/(50+VD), xmax=VD/(50+VD), color='b', linestyle='-', label="-Structural limit")
    plt.plot([vair_VC_pos[-1], VD], [nVC_pos[-1], nVD_pos[-1]], '--')
    plt.plot([vair_VC_neg[-1], VD], [nVC_neg[-1], nVD_neg[-1]], '--')
    plt.xlim(0, VD+50)
    plt.ylim(-2, 3)
    plt.grid(True)
    


    # x_env = np.concatenate([
    #     Vair_pos,                    # up along stall
    #     [V_corner_pos, VD, VD, V_corner_neg],# along top, down, and back
    #     Vair_neg[::-1]               # down along stall
    # ])
    # y_env = np.concatenate([
    #     nstall_pos_list,             # positive stall
    #     [n_pos_limit, n_pos_limit, n_neg_limit, n_neg_limit], # straight edges
    #     nstall_neg_list[::-1]        # negative stall
    # ])

    # plt.fill(x_env, y_env, color='lightgreen', alpha=0.3, label="Flight Envelope")

    plt.xlabel("Airspeed [m/s]")
    plt.ylabel("Load Factor n")
    plt.legend(loc = 'center', bbox_to_anchor=(1.2, 0.5), borderaxespad=0)
    plt.tight_layout()
    plt.title("V-n Diagram: Flight Envelope")


