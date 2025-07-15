# Trabalho 1
Trabalho 1 de planejamento de rotas realizado pela aluna Leticia Miti Takahashi - 231035437 para a matéria de Tópicos Especiais em Eletrônica - Robótica administrada pelo professor Roberto Baptista

Este projeto tem como objetivo implementar e comparar dois métodos distintos de controle para um robô móvel Pioneer no simulador CoppeliaSim.

- **`demopaths.py`**: usando *waypoints* com orientação por PID.
- **`demoprimitivas.py`**: usando primitivas de movimento.

##  Requisitos

- Python 3.x
- [CoppeliaSim](https://www.coppeliarobotics.com/)
- Biblioteca `sim.py` (incluída no CoppeliaSim para comunicação remota)
- Pacotes Python:
  - `numpy`
  - `math`
  - `csv`
 

###  demopaths

É possível editar a lista de *waypoints* no código:

```python
waypoints = [
    [-1.75, -0.75],
    [-0.9625, -1.4125],
    [-0.175, -0.75],
    [-0.175, -2.05],
]
```
Nesta abordagem, o robô percorre uma sequência de pontos no espaço, utilizando um controlador PID para corrigir seu ângulo de orientação. A lógica do movimento é baseada no erro angular entre a direção atual do robô e a direção do próximo waypoint. O movimento do robô segue o ballPos.

O cálculo da distância ao alvo é feito por:

```python
error_distance = math.sqrt((ballPos[1] - position[1])**2 + (ballPos[0] - position[0])**2)
```

E o ângulo desejado é:

```python
phid = math.atan2(ballPos[1] - position[1], ballPos[0] - position[0])
```

O erro angular entre a orientação do robô e a direção desejada é usado como entrada do PID:

```python
error_phi = phid - self.phi
```

Se o erro de distância for maior que um valor mínimo predefinido, o PID ajusta a velocidade das rodas para alinhar o robô corretamente ao waypoint:

```python
if error_distance >= self.Min_error_distance:
    phid = math.atan2(ballPos[1] - position[1], ballPos[0] - position[0])
    controller_Linear = self.v_linear * error_distance
```

---

###  demoprimitivas

É possível editar os parâmetros no `main`:

```python
if __name__ == "__main__":
    crb01 = Pioneer()
    x = 1.5      # deslocamento no eixo X (após o giro)
    y = 2        # deslocamento inicial no eixo Y
    v = 0.1      # velocidade linear (m/s)

    crb01.executar_movimento_L_com_primitivas(x, y, v)
```

Essa abordagem utiliza uma sequência de movimentos básicos (primitivas) para realizar trajetórias planejadas. Os movimentos possíveis são:

1. Seguir reto
2. Curva à esquerda (90°)
3. Curva à direita (90°)
   
E o robo vai usá-los para chegar em seu destino.

A implementação dos 3 movimentos foi feita da maneira a seguir:

```python
if primitiva == 1:
    omega = 0
elif primitiva == 2:
    radio_ideal = 0.5
    omega = -self.v_linear / (-radio_ideal)  # curva à esquerda
elif primitiva == 3:
    radio_ideal = 0.5
    omega = -self.v_linear / radio_ideal     # curva à direita
else:
    omega = 0
```
Para que o robô execute curvas à direita ou à esquerda, foi necessário definir um raio de giro que representa a trajetória circular descrita durante a rotação. A movimentação é composta por trechos retos e curvos, em que o robô primeiro avança no eixo Y em linha reta e, em seguida, realiza uma curva de quase 90°, para se alinhar com o eixo X ao final do arco.

A movimentação do robô foi baseada no tempo de execução. Para correlacionar o tempo com a distância informada pelo usuário, adotou-se a relação: tempo = distancia / velocidade_linear, onde t é o tempo de execução, d é a distância desejada e 𝑣 é a velocidade linear do robô. Essa relação é implementada diretamente no código, conforme evidenciado na seguinte linha:

```python
tempo_x = abs((x - radio_ideal) / v_linear))
tempo_y = abs((y - radio_ideal) / v_linear)
tempo_giro = (math.pi / 2) * (L / v_linear)
```

Note que, nos trechos retilíneos, a distância informada pelo usuário (x ou y) é ajustada subtraindo o raio da curva (radio_ideal), pois ao realizar a curva o robô já se desloca parcialmente nos dois eixos. Esse ajuste garante maior precisão na execução do trajeto, mantendo a coerência entre a movimentação circular e os deslocamentos lineares subsequentes.
    
## Teoria

Em ambas as abordagens, o robô simulado é do tipo diferencial com duas rodas. Os principais conceitos utilizados incluem:

Definição das velocidades individuais das rodas

<img width="150" height="145" alt="Image" src="https://github.com/user-attachments/assets/d6d90722-7c79-49d0-bfdc-d0fa3159d4ca" />

Cálculo da velocidade linear e angular

<img width="150" height="137" alt="Image" src="https://github.com/user-attachments/assets/f7e59fda-58f1-4c12-89cc-266b3e561acd" />

 Cálculo de orientação via atan2:

 <img width="200" height="45" alt="image" src="https://github.com/user-attachments/assets/fb1f11c3-31ac-43e9-b409-90170a4c7944" />

Correção do movimento baseada em erro angular (PID) ou em sequências temporizadas (primitivas)

<img width="139" height="35" alt="Image" src="https://github.com/user-attachments/assets/7c93a464-ba6a-42ed-83b3-bc86317c8efe" />

---

## Conclusão

Este projeto demonstra diferentes estratégias de controle para um robô móvel, comparando controle por feedback (PID) e controle baseado em sequência de ações (primitivas). Ambas as abordagens permitem desenvolver habilidades importantes em robótica móvel, controle e simulação.
