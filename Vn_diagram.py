import numpy as np
import matplotlib.pyplot as plt

from flightpath_sim import aircraft_class as aircraft

def Vn_diagram(step):
    W = aircraft.Wmax
    rho = aircraft.rho0
    S = aircraft.S
    n_pos_limit = aircraft.n_pos_limit
    n_neg_limit = aircraft.n_neg_limit



    CL_max = 
    CL_min = 
    V_C_pos = np.sqrt(2 * W * n_pos_limit / (rho * S * CL_max))
    V_C_neg = np.sqrt(2 * W * n_neg_limit / (rho * S * CL_min))
    V_D = 1.25 * V_C_pos

    Vair_pos = np.arange(0, V_C_pos + step, step)
    Vair_neg = np.arange(0, V_C_neg + step, step)

    nstall_pos_list = 0.5 * rho * Vair_pos**2 * CL_max / (W / S)
    nstall_neg_list = 0.5 * rho * Vair_neg**2 * CL_min / (W / S)


    plt.figure("V-n Diagram")
    # Stall curves
    plt.plot(Vair_pos, nstall_pos_list, label="Positive Stall Limit")
    plt.plot(Vair_neg, nstall_neg_list, label="Negative Stall Limit")

    # Full limit lines
    plt.axvline(x=V_D, color='k', linestyle='-', label="V_D")
    plt.axhline(y=n_pos_limit, color='r', linestyle='-', label="n_pos limit")
    plt.axhline(y=n_neg_limit, color='b', linestyle='-', label="n_neg limit")


    x_env = np.concatenate([
        Vair_pos,                    # up along stall
        [V_C_pos, V_D, V_D, V_C_neg],# along top, down, and back
        Vair_neg[::-1]               # down along stall
    ])
    y_env = np.concatenate([
        nstall_pos_list,             # positive stall
        [n_pos_limit, n_pos_limit, n_neg_limit, n_neg_limit], # straight edges
        nstall_neg_list[::-1]        # negative stall
    ])

    plt.fill(x_env, y_env, color='lightgreen', alpha=0.3, label="Flight Envelope")

    plt.xlabel("Airspeed [m/s]")
    plt.ylabel("Load Factor n")
    plt.legend()
    plt.title("V–n Diagram: Flight Envelope")


Vn_diagram(1)
plt.show()
