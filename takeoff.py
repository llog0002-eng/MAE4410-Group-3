def takeoff(aircraft):
    # Input aircraft object with all relevant variables
    # Output: sa (m), flight path angle (deg), pitch angle (deg), angle of attack (deg)

    import numpy as np

    Vmax = aircraft.Ma*aircraft.c                 # Max speed requirement, m/s
    g = 9.80665                 # Gravitational acceleration m2/s

    Wmax = aircraft.WTO*g

    theta_LOF = 14.5*np.pi/180                            # liftoff pitch angle, rad

    CL_LOF = aircraft.CL0_TO + aircraft.CLalpha_TO * theta_LOF            # Lift coefficient at liftoff
    W_TO = Wmax*aircraft.betaw_taxi                              # Weight at takeoff, N
    V_stall_TO = np.sqrt(2 * W_TO/aircraft.S / (aircraft.rho0 * CL_LOF))  # Stall speed at takeoff, m/s
    VR = 1.05 * V_stall_TO                              # Rotate speed, m/s
    VLOF = 1.1 * V_stall_TO                             # Lift-off speed, m/s
    V2 = 1.13 * V_stall_TO                              # Take-off safety speed, m/s

    a0_TO = 1                                           # Throttle setting in Takeoff
    a0d_TO = a0_TO * (1 + aircraft.av * VLOF/aircraft.c0)                 # Airspeed performance correction
    alphae_TO = a0d_TO * (1)**aircraft.a                         # Ratio of maximum static thrust or power at sea level to the thrust or power at the desired operating condition, takeoff
    T_TO = aircraft.Tmax*alphae_TO

    theta_TO = theta_LOF                                # Initial takeoff pitch angle, rad

    dt_TO = 0.001


    # Initialise variables
    modifier = 1
    h_TO = 0
    iteration_i = 0
    iteration_o = 0
    Vair_TO = VLOF
    sair_TO = 0
    speed_too_low = True

    while abs(1-Vair_TO/V2) > 0.001 or Vair_TO <= V2 or h_TO < aircraft.hscreen:
        if abs(1-Vair_TO/V2) > 0.001:
            print(f'Current airspeed: {Vair_TO}, V2: {V2}, Pitch angle: {theta_TO*180/np.pi}')
        elif Vair_TO <= V2:
            print(f'Current airspeed: {Vair_TO}, V2: {V2}, airspeed too low')
        elif h_TO < aircraft.hscreen:
            print(f'Current height: {h_TO}, screen height: {aircraft.hscreen}, height too low')
        
        h_TO = 0
        iteration_i = 0
        Vair_TO = VLOF
        sair_TO = 3*VLOF

        while h_TO < aircraft.hscreen:
            CL = W_TO*np.cos(theta_TO) / (0.5*aircraft.rho0*Vair_TO**2*aircraft.S)
            CD = aircraft.CD0_TO + aircraft.K*CL**2
            D = 0.5*aircraft.rho0*Vair_TO**2*aircraft.S*CD
            AoA_TO = (CL - aircraft.CL0_TO) / aircraft.CLalpha_TO  # Angle of attack, rad

            gamma_TO = theta_TO - AoA_TO # Flight path angle, rad
            Vair_TO += (T_TO - D - W_TO*np.sin(theta_TO))/(W_TO/g) * dt_TO
            sair_TO += Vair_TO*np.cos(gamma_TO)*dt_TO
            hdot_TO = Vair_TO*np.sin(gamma_TO)
            h_TO += hdot_TO*dt_TO
            iteration_i += 1

            mdot_TO = aircraft.TSFC_TO*T_TO
            W_TO -= mdot_TO*dt_TO*g

            if iteration_i > 100000:
                print("The inner function has iterated 100,000 times and hscreen is still not reached. Please check the code.")
                print(theta_TO*180/np.pi)
                break

            if gamma_TO < -5*np.pi/180:
                print("The flight path angle has become negative. Please check the code.")
                print(gamma_TO*180/np.pi)
                raise Exception("Flight path angle negative")

        if iteration_o > 100:
                print("The outer function has iterated 100 times and hscreen is still not reached. Please check the code.")
                print(theta_TO*180/np.pi)
                break

        speed_too_low_last = speed_too_low

        if Vair_TO < V2:
            speed_too_low = True
        else:
            speed_too_low = False

        if speed_too_low_last != speed_too_low:
            modifier *= 0.5

        if speed_too_low:
            theta_TO -= modifier * 0.1 * np.pi/180
        else:
            theta_TO += modifier * 0.1 * np.pi/180

        iteration_o += 1
        


    print(f'The function has iterated {iteration_o} times.\n\nThe final height of the airplane is {h_TO} m.\n\n\
    The final airspeed of takeoff is {Vair_TO} m/s.\n\nThe final air distance is {sair_TO} m.')
    if Vair_TO < V2:
        print(f'The final airspeed of takeoff is {Vair_TO} and it\'s less than the safety speed V2={V2}. VLOF={VLOF}.')
    print(f"The final flight path angle is {gamma_TO*180/np.pi} degrees.")
    print(f"The final pitch angle is {theta_TO*180/np.pi} degrees.")
    print(f"The final angle of attack is {AoA_TO*180/np.pi} degrees.")

    return(sair_TO, gamma_TO*180/np.pi, theta_TO*180/np.pi, AoA_TO*180/np.pi)
