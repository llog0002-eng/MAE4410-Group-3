def takeoff(aircraft):
    # Input: aircraft object with all relevant variables
    # Output: property dataframe

    import numpy as np
    import pandas as pd

    Vmax = aircraft.Ma*aircraft.c                 # Max speed requirement, m/s
    g = 9.80665                                   # Gravitational acceleration m2/s


    theta_TO = 10 * np.pi/180                                               # Initial pitch angle, rad
    CL_stall = aircraft.CL0_TO + aircraft.CLalpha_TO * theta_TO             # Lift coefficient at stall, takeoff
    W_TO = aircraft.W * g                                                   # Weight at takeoff, N
    V_stall_TO = np.sqrt(2 * W_TO/aircraft.S / (aircraft.rho0 * CL_stall))  # Stall speed at takeoff, m/s
    VR = 1.05 * V_stall_TO                                                  # Rotate speed, m/s
    VLOF = 1.1 * V_stall_TO                                                 # Lift-off speed, m/s
    V2 = 1.13 * V_stall_TO
    
    # Angle required to lift off                                            # liftoff pitch angle, rad
    a0_TO = 1                                                               # Throttle setting in Takeoff
    a0d_TO = a0_TO * (1 + aircraft.av * VLOF/aircraft.c0)                   # Airspeed performance correction
    alphae_TO = a0d_TO * (1)**aircraft.a                                    # Ratio of maximum static thrust or power at sea level to the thrust or power at the desired operating condition, takeoff
    T_TO = aircraft.Tmax*alphae_TO

    dt = 0.1 # time step, s

    # Initialise variables
    err = 1
    modifier = 1 # Pitch angle convergence modifier
    j = 0 # Outer while loop iteration count
    i = 0 # Inner while loop iteration count
    speed_too_low = True

    while err > 0.001 or altitude[i] < aircraft.hscreen:
        j += 1 # Outer while loop iteration count

        n = 5000 # Number of points to initialise
        dist = np.zeros(n)
        m = np.zeros(n)
        LDs = np.zeros(n)
        alpha = np.zeros(n)
        ts = np.zeros(n)
        altitude = np.zeros(n)
        Vs = np.zeros(n)
        throttles = np.ones(n)
        theta = np.ones(n)*theta_TO

        i = 0 # Inner while loop iteration count
        Vs[0] = VLOF
        dist[0] = 3*VLOF
        m[0] = aircraft.W
        CL = W_TO/np.cos(theta_TO) / (0.5*aircraft.rho0*Vs[0]**2*aircraft.S)
        alpha[0] = (CL - aircraft.CL0_TO) / aircraft.CLalpha_TO  # Angle of attack, rad

        # While below the screen height
        while altitude[i] < aircraft.hscreen:

            if i > n:
                raise Exception("Takeoff iteration count passed datapoint preallocation size")
            
            CL = W_TO/np.cos(theta_TO) / (0.5*aircraft.rho0*Vs[i]**2*aircraft.S)
            CD = aircraft.CD0_TO + aircraft.KTO*CL**2
            LDs[i] = CL/CD
            D = 0.5*aircraft.rho0*Vs[i]**2*aircraft.S*CD

            alpha[i+1] = (CL - aircraft.CL0_TO) / aircraft.CLalpha_TO  # Angle of attack, rad
            gamma_TO = theta_TO - alpha[i+1] # Flight path angle, rad

            Vs[i+1] = Vs[i] + (T_TO - D - W_TO*np.sin(gamma_TO))/(W_TO/g) * dt
            dist[i+1] = dist[i] + Vs[i]*np.cos(gamma_TO)*dt
            hdot_TO = Vs[i]*np.sin(gamma_TO)
            altitude[i+1] = altitude[i] + hdot_TO*dt

            dm = aircraft.TSFC_TO*T_TO/1e6*dt # Fuel flow, kg
            aircraft.W -= dm
            m[i+1] = aircraft.W

            i += 1 # Inner while loop iteration count
            ts[i] = i * dt

            if i > 100000:
                print("The inner function has iterated 100,000 times and hscreen is still not reached. Please check the code.")
                break

            if gamma_TO < -5*np.pi/180:
                raise Exception("Flight path angle negative")
            
            if j > 100:
                raise Exception("The outer function has iterated 100 times and hscreen is still not reached. Please check the code.")

        speed_too_low_last = speed_too_low

        # Find if speed is too low
        if Vs[i] < V2:
            speed_too_low = True
        else:
            speed_too_low = False

        # If went past target speed, halve convergence modifier
        if speed_too_low_last != speed_too_low:
            modifier *= 0.5

        # If speed is too low, decrease pitch angle
        if speed_too_low:
            theta_TO -= modifier * 0.1 * np.pi/180
        # If speed is too high, increase pitch angle
        else:
            theta_TO += modifier * 0.1 * np.pi/180

        err = abs(1-Vs[i]/V2)

    print(f'Airspeed at takeoff is {Vs[i]:.2f} m/s.\n\nThe final air distance is {dist[i]:.2f} m.')
    print(f"Angle of attack at takeoff is {alpha[i]*180/np.pi:.2f} degrees.")

    df = pd.DataFrame(
        {
            "distance": dist[:i],
            "mass": m[:i],
            "L/D": LDs[:i],
            "AoA": alpha[:i],
            "time": ts[:i],
            "altitude": altitude[:i],
            "speed": Vs[:i],
            "optimal altitude": np.full(i,np.nan),
            "throttle": throttles[:i],
            "theta": theta[:i],
        }
    )

    return(df)
