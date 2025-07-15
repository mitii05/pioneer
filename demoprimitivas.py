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
        self.v_linear = 0.1
        self.posError = []
        self.Min_error_distance = 0.1

    def connect_Pioneer(self, port):

        sim.simxFinish(-1)
        clientID = sim.simxStart('127.0.0.1', port, True, True, 2000, 5)
        if clientID == 0:
            print("Conectado no", port)
        else:
            print("Nao foi possivel conectar no", port)

        returnCode, robot = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx',
                                                    sim.simx_opmode_blocking)
        returnCode, MotorE = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor',
                                                     sim.simx_opmode_blocking) 
        returnCode, MotorD = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor',
                                                     sim.simx_opmode_blocking)
        returnCode, ball = sim.simxGetObjectHandle(clientID, 'ball', sim.simx_opmode_blocking)

        return clientID, robot, MotorE, MotorD, ball

    def Speed_Pioneer(self, U, omega, lock_parar_simulacao, signed, error_phi):

        L = 0.381
        R = 0.095

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


    def Robot_Pioneer_Primitivas(self, primitiva, duracao=1.0):
        clientID, robot, motorE, motorD, _ = self.connect_Pioneer(19999)

        if sim.simxGetConnectionId(clientID) != -1:
            sim.simxSetJointTargetVelocity(clientID, motorE, 0, sim.simx_opmode_blocking)
            sim.simxSetJointTargetVelocity(clientID, motorD, 0, sim.simx_opmode_blocking)

            controller_Linear = self.v_linear
            lock_parar_simulacao = 0

            if primitiva == 1:
                omega = 0
            elif primitiva == 2:
                radio_ideal = 0.45
                omega = -self.v_linear / (-radio_ideal)
            elif primitiva == 3:
                radio_ideal = 0.45
                omega = -self.v_linear / radio_ideal  
            else:
                omega = 0

            vl, vd, _ = self.Speed_Pioneer(controller_Linear, omega, lock_parar_simulacao, 1, 0)

            sim.simxSetJointTargetVelocity(clientID, motorE, vl, sim.simx_opmode_blocking)
            sim.simxSetJointTargetVelocity(clientID, motorD, vd, sim.simx_opmode_blocking)

            time.sleep(duracao)

            sim.simxSetJointTargetVelocity(clientID, motorE, 0, sim.simx_opmode_blocking)
            sim.simxSetJointTargetVelocity(clientID, motorD, 0, sim.simx_opmode_blocking)

    def executar_movimento_com_primitivas(self, x, y, v_linear):
        L = 0.381
        radio_ideal = 0.45
        self.v_linear = v_linear  

        clientID, robot, _, _, _ = self.connect_Pioneer(19999)
        _, pos_inicial = sim.simxGetObjectPosition(clientID, robot, -1, sim.simx_opmode_blocking)
              
        if y != 0 and x != 0:
            
            tempo_y = abs((y - radio_ideal) / v_linear)
            print(f"Andando {y}m no eixo Y")
            self.Robot_Pioneer_Primitivas(1, tempo_y)
                
            tempo_giro = (math.pi / 2) * (L / v_linear)
            print("Girando 90° para a direita")
            self.Robot_Pioneer_Primitivas(3, tempo_giro)

            tempo_x = abs((x - radio_ideal) / v_linear) 
            print(f"Andando {x}m no eixo X")
            self.Robot_Pioneer_Primitivas(1, tempo_x)
        
        elif x == 0 and y != 0:
            
            tempo_y = abs((y) / v_linear)
            print(f"Andando {y}m no eixo Y")
            self.Robot_Pioneer_Primitivas(1, tempo_y)
        
        elif y == 0 and x != 0:
            
            tempo_giro = (math.pi / 2) * (L / v_linear)
            print("Girando 90° para a direita")
            self.Robot_Pioneer_Primitivas(3, tempo_giro)
            tempo_x = abs((x - 2*radio_ideal) / v_linear)
            print(f"Andando {x}m no eixo X")
            self.Robot_Pioneer_Primitivas(1, tempo_x)
            
            tempo_giro = (math.pi / 2) * (L / v_linear)
            print("Girando 90° para a direita")
            self.Robot_Pioneer_Primitivas(3, tempo_giro)
            
        _, pos_final = sim.simxGetObjectPosition(clientID, robot, -1, sim.simx_opmode_blocking)

        print("\nTrajetória geral:")
        print(f"Inicial: {pos_inicial[0]:.4f}, {pos_inicial[1]:.4f}")
        print(f"Final:   {pos_final[0]:.4f}, {pos_final[1]:.4f}")
        print(f"Distância percorrida:   Δx = {abs(pos_inicial[0] - pos_final[0]):.4f}, Δy = {abs(pos_inicial[1] - pos_final[1]):.4f}")


if __name__ == "__main__":
    crb01 = Pioneer()
    x = 1.5 
    y = 2  
    v = 0.1 

    crb01.executar_movimento_com_primitivas(x, y, v)