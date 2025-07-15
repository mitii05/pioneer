import math
import sim
import numpy as np
import math as mat
import time
import csv


class Pioneer():

    def __init__(self):

        self.y_out = []
        self.x_out = []
        self.phi = 0
        self.v_max_wheels = 15
        self.v_min_wheels = -15
        self.v_linear = 100
        self.posError = []
        self.Min_error_distance = 0.05

    def connect_Pioneer(self, port):

        sim.simxFinish(-1)
        clientID = sim.simxStart('127.0.0.1', port, True, True, 2000, 5)
        if clientID == 0:
            print("Connect to", port)
        else:
            print("Can not connect to", port)

        returnCode, robot = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx',
                                                    sim.simx_opmode_blocking) 
        returnCode, MotorE = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor',
                                                     sim.simx_opmode_blocking) 
        returnCode, MotorD = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor',
                                                     sim.simx_opmode_blocking)  
        returnCode, ball = sim.simxGetObjectHandle(clientID, 'ball', sim.simx_opmode_blocking)

        return clientID, robot, MotorE, MotorD, ball

    def Speed_Pioneer(self, U, omega, lock_parar_simulacao, signed, error_phi):

        L = 381  
        R = 95  

        vd = ((2 * (U) + omega * L) / (2 * R))
        vl = ((2 * (U) - omega * L) / (2 * R))
        a = 1

        Max_Speed = self.v_max_wheels
        Min_Speed = self.v_min_wheels

        if vd >= Max_Speed:
            vd = Max_Speed
        if vd <= Min_Speed:
            vd = Min_Speed

        if vl >= Max_Speed:
            vl = Max_Speed
        if vl <= Min_Speed:
            vl = Min_Speed

        if lock_parar_simulacao == 1 and error_phi <= 0.08:
            a = 0
            vl = 0
            vd = 0

        return vl, vd, a

    def PID_Controller_phi(self, kp, ki, kd, deltaT, error, interror, fant, Integral_part):

        saturacao_da_integral = 10

        raizes = np.roots([kd, kp, ki])
        absoluto = abs(raizes)
        mayor = max(absoluto)
        Filter_e = 1 / (mayor * 10)

        ummenosalfaana = mat.exp(-(deltaT / Filter_e))
        alfaana = 1 - ummenosalfaana
        interror = interror + error
        f = ummenosalfaana * fant + alfaana * error
        if fant == 0:
            deerror = (f / deltaT)
        else:
            deerror = (float((f - fant) / (deltaT)))

        if Integral_part > saturacao_da_integral:
            Integral_part = saturacao_da_integral
        elif Integral_part < -saturacao_da_integral:
            Integral_part = -saturacao_da_integral
        else:
            Integral_part = ki * interror * deltaT


        PID = kp * error + Integral_part + deerror * kd

        return PID, f, interror, Integral_part

    def Robot_Pioneer(self, x, deltaT, filename):

        kpi = x[0]
        kii = x[1]
        kdi = x[2]
        cont_ar=0

        (clientID, robot, motorE, motorD, ball) = self.connect_Pioneer(19999)

        raizes = np.roots([kdi, kpi, kii])
        acumulate_error = 0
        absoluto = abs(raizes)
        mayor = max(absoluto)
        Filter_e = 1 / (mayor * 10)
        ummenosalfaana = mat.exp(-(deltaT / Filter_e))
        alfaana = 1 - ummenosalfaana
        omega_ant = 0
        a = 1
        angulo_anterior_phid = 0
        angulo_anterior_phi_robot = 0

        Numbero_de_Iteracoes = 0
        Time_Sample = []
        interror_phi = 0
        fant_phi = 0
        Integral_part_phi = 0

        if (sim.simxGetConnectionId(clientID) != -1):
            
            waypoints = [
                [-1.75, -0.75],
                [-0.9625, -1.4125],
                [-0.175, -0.75],
                [-0.175, -2.05],
            ]
            
                                   
            current_target_index = 0
            ballPos = waypoints[current_target_index]

            while (a == 1):

                if Numbero_de_Iteracoes <= 1:

                    s, position = sim.simxGetObjectPosition(clientID, robot, -1, sim.simx_opmode_streaming)
                    s, angle_robot = sim.simxGetObjectOrientation(clientID, robot, -1, sim.simx_opmode_blocking)
                    self.phi = 0
                    sim.simxSetJointTargetVelocity(clientID, motorE, 0, sim.simx_opmode_blocking)
                    sim.simxSetJointTargetVelocity(clientID, motorD, 0, sim.simx_opmode_blocking)
                else:

                    s, position = sim.simxGetObjectPosition(clientID, robot, -1, sim.simx_opmode_streaming)
                    returnCode, orientation = sim.simxGetObjectOrientation(clientID, robot, -1,
                                                                           sim.simx_opmode_blocking)
                    signed = 1
                    phi_robot = orientation[2]
                    self.phi = phi_robot

                    error_distance = math.sqrt((ballPos[1] - position[1]) ** 2 + (ballPos[0] - position[0]) ** 2)

                    if error_distance >= self.Min_error_distance:

                        phid = math.atan2(ballPos[1] - position[1], ballPos[0] - position[0])

                        controller_Linear = self.v_linear * error_distance 
                        lock_parar_simulacao = 0

                    else:
                        current_target_index += 1
                        if current_target_index < len(waypoints):
                            ballPos = waypoints[current_target_index]
                            prevPos = waypoints[current_target_index - 1]
                            dx = ballPos[0] - prevPos[0]
                            dy = ballPos[1] - prevPos[1]
                            phid = math.atan2(dy, dx)

                        elif current_target_index == len(waypoints):
                            phid = 1.57
                            controller_Linear = 0
                        else:
                            lock_parar_simulacao = 1         
                                
                    diferenca_phid = phid - angulo_anterior_phid
                    diferenca_phi = self.phi - angulo_anterior_phi_robot
                    if diferenca_phid > math.pi:
                        phid -= 2 * math.pi
                    elif diferenca_phid < -math.pi:
                        phid += 2 * math.pi
                    if diferenca_phi > math.pi:
                        self.phi -= 2 * math.pi
                    elif diferenca_phi < -math.pi:
                        self.phi += 2 * math.pi

                    angulo_anterior_phid = phid
                    angulo_anterior_phi_robot = self.phi

                    print(f'phid == > {phid}, self.phi ==> {self.phi}, error ==> {phid - self.phi}')
                    error_phi = phid - self.phi

                    acumulate_error = acumulate_error + abs(phid - self.phi)

                    omega, fant_phi, interror_phi, Integral_part_phi = self.PID_Controller_phi(kpi, kii, kdi, deltaT,
                                                                                               error_phi, interror_phi,
                                                                                               fant_phi,
                                                                                               Integral_part_phi)

                    if omega >= 100 or omega <= -100:
                        omega = omega_ant
                    else:
                        omega_ant = omega

                    self.posError.append(error_distance)

                    vl, vd, a = self.Speed_Pioneer(controller_Linear, omega, lock_parar_simulacao, signed,
                                               abs(phid - self.phi))

                    print(f'Speed lef == {vl}, Speed Right == {vd}')

                    sim.simxSetJointTargetVelocity(clientID, motorE, vl, sim.simx_opmode_blocking)
                    sim.simxSetJointTargetVelocity(clientID, motorD, vd, sim.simx_opmode_blocking)

                Numbero_de_Iteracoes = Numbero_de_Iteracoes + 1
                Time_Sample.append(Numbero_de_Iteracoes * deltaT)
                if Numbero_de_Iteracoes >= 60:
                    a == 0
                self.y_out.append(position[1])
                self.x_out.append(position[0])
            if len(self.y_out) != len(self.x_out):
                raise ValueError("self.y_out and self.x_out must be of the same length")

            with open(filename, mode='w', newline='') as file:
                writer = csv.writer(file)

                writer.writerow(['x_out', 'y_out'])

                for x, y in zip(self.x_out, self.y_out):
                    writer.writerow([x, y])

            print(f"Data saved to {filename}")

        for x, y in zip(self.x_out, self.y_out):
            print(f"{x:.4f}, {y:.4f}")

if __name__ == "__main__":
    Rvss = 1.6
    RPioneer = 95
    FS = RPioneer / Rvss
    crb01 = Pioneer()
    filename = "Pioneer_experiment.csv"
    kpi_DE = [0.3629, 0.3609, 0.8000, 0.3746, 0.3432]
    kii_DE = [0.1891, 0.3841, 0.0479, 0.0001, 0.0001]
    kdi_DE = [0.0001, 0.0039, 0.0001, 0.0001, 0.0001]
    kpi_MFO = [0.3902, 0.3504, 0.3201, 0.3278, 0.3413]
    kii_MFO = [0.3468, 0.2910, 0.0001, 0.0774, 0.1230]
    kdi_MFO = [0.0001, 0.0050, 0.0001, 0.0002, 0.0049]
    x = [kpi_MFO[4], kii_MFO[4], kdi_MFO[4]]
    deltaT = 0.05
    crb01.Robot_Pioneer(x, deltaT, filename)