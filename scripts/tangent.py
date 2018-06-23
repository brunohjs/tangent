#!/usr/bin/env python
#coding: utf-8

"""
Executando:
    - Em um terminal novo, execute o roscore;
    - Abra outro terminal vá até a pasta 'catkin_ms' e execute: 
        rosrun stage_ros stageros src/tangent/world/create_hokuyo.world
    - Abra um terceiro terminal e execute o script:
        rosrun tangent tangent.py
        *Obs: se o ROS não reconhecer o script, vá até a pasta onde está o script e execute:
            chmod +x tangent.py
            cd ~/catkin_ms
            catkin_make

Informações sobre o log do terminal:

siglas:
    AG (Goal Angle)             Ângulo entre o vetor do robô e o vetor
                                da distância entre o robô e o ponto destino;
    
    DG (Goal Distance)          Distância entre o robô e o ponto destino;
    
    CD (Colision Distances)     Leitura do LaserScan frontal do robô, 
                                respectivamente [esquerdo(-22.5 graus), centro(0 graus), direito(+25,5 graus)].

Opcodes:
Os opcodes informam o quê o robô está identificando. Para cada identificação há uma ação,
    [ none ] - Nada a fazer;
    [ frnt ] - Colisão frontal. Consequência: o robô vai virar para qualquer lado e, em seguida, vai dar ré;
    [cofntl] - Colisão esquerda-frontal. Consequência: o robô vai virar para a direita;
    [cofntr] - Colisão direita-frontal. Consequência: o robô vai virar para a esquerda;
    [colsde] - Colisão lateral. Consequência: o robô vai seguir reto com uma leve inclinação para os lados;
    [indirc] - Está na direção do destino. Consequência: o robô seguirá em frente com uma leve inclinação para os lados;
    [ntdir1] - Não está na diração do destino. Consequência: o robô irá rotacionar até achar a direção certa;
    [ntdir2] - Semelhante ao anterior, porém o robô irá rotacionar para o lado contrário;

"""

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from math import degrees,atan2,asin,acos,radians,cos,sin
from time import sleep

current_pose = Odometry()
current_laser = LaserScan()

def lasercallback(data):
    global current_laser
    current_laser = [   
                        data.ranges[180],   #Esquerdo   (-90 graus)
                        data.ranges[360],   #Esquerdo   (-45 graus)
                        data.ranges[450],   #Esquerdo   (-22.5 graus)
                        data.ranges[540],   #Centro     (0 graus)
                        data.ranges[630],   #Direito    (+22.5 graus)
                        data.ranges[720],   #Direito    (+45 graus)
                        data.ranges[900]    #Direito    (+90 graus)
    ]

def posecallback(data):
    global current_pose
    current_pose = data

'Função para aplicar a nova velocidade do robô'
def setSpeed(linear_angular, speed, vel_pub):
    speed.linear.x = linear_angular[0]
    speed.angular.z = linear_angular[1]
    vel_pub.publish(speed)

'Função para mudar a orientação e velocidade do robô'
def setOrientation(goal_angle, goal_dist, prvs_angle, prvs_side, speed, vel_pub):
    #Iniciar a variável {log}
    log = ' none '

    #Se a distância do objetivo estiver a menos de 0.5 unidades, então o caminho foi encontrado
    if goal_dist <= 0.5:
        print('Caminho encontrado!')
        rospy.on_shutdown(main())
    
    #Se o robô for colidir de frente com um obstáculo
    if collide() == 'front':
        log = ' frnt '
        if prvs_angle < goal_angle:
            if prvs_side == 'R':
                prvs_side = 'L'
                setSpeed((0, 4), speed, vel_pub)
            else:
                prvs_side = 'R'
                setSpeed((0, -4), speed, vel_pub)
        elif prvs_angle > goal_angle:
            if prvs_side == 'R':
                prvs_side = 'R'
                setSpeed((0, -4), speed, vel_pub)
            else:
                prvs_side = 'L'
                setSpeed((0, 4), speed, vel_pub)
        setSpeed((-2, 1), speed, vel_pub)

    #Se o robô for colidir de frente-esquerdo (-22.5 graus) com um obstáculo
    elif collide() == 'front-lf':   
        log = 'cofntl'
        setSpeed((1, -2), speed, vel_pub)
    
    #Se o robô for colidir de frente-direito (+22.5 graus) com um obstáculo'
    elif collide() == 'front-rg':
        log = 'cofntr'
        setSpeed((1, 2), speed, vel_pub)

    #Se o robô for colidir de lado com um obstáculo'
    elif collide() == 'side':
        log = 'colsde'
        if prvs_side == 'R':
            setSpeed((1, 0.3), speed, vel_pub)
            prvs_side = 'L'
        else:
            setSpeed((1, -0.3), speed, vel_pub)
            prvs_side = 'R'   

    #Se o robô estiver orientado na direção do ponto de destino
    elif goal_angle < 10:
        log = 'indirc'
        if prvs_side == 'R':
            setSpeed((1, 0.3), speed, vel_pub)
            prvs_side = 'L'
        else:
            setSpeed((1, -0.3), speed, vel_pub)
            prvs_side = '   R'

    #Se o robô não estiver orientado na direção do ponto de destino   
    elif prvs_angle < goal_angle:
        log = 'ntdir1'
        if prvs_side == 'R':
            prvs_side = 'L'
            setSpeed((0, 2), speed, vel_pub)
        else:
            prvs_side = 'R'
            setSpeed((0, -2), speed, vel_pub)
    elif prvs_angle > goal_angle:
        log = 'ntdir2'
        if prvs_side == 'R':
            prvs_side = 'R'
            setSpeed((0, -2), speed, vel_pub)
        else:
            prvs_side = 'L'
            setSpeed((0, 2), speed, vel_pub)
    
    return goal_angle, prvs_side, log

'Função que verifica se o robô vai colidir com algum obstáculo'
def collide():
    if current_laser[3] < 0.8:
        return 'front'
    elif current_laser[4] < 1:
        return 'front-lf'
    elif current_laser[2] < 1:
        return 'front-rg'
    elif (current_laser[0] < 0.5) or (current_laser[6] < 0.5):
        return 'side'
    elif (current_laser[1] < 1) or (current_laser[5] < 1):
        return 'side'
    else:
        return False

'Função que retorna a posição atual do robô'
def getPose():
    return (current_pose.pose.pose.position.x, current_pose.pose.pose.position.y)

'Função que retorna a distância do robô até o destino'
def goalDistance(goal):
    rx, ry = getPose()
    return ((goal[0] - rx)**2 + (goal[1] - ry)**2)**0.5

'Função que retorna o ângulo entre os vetores da frente do robô e a distância até o destino'
def goalAngle(goal):
    rx, ry = getPose()
    r_angle = radians(getAngle())
    angle = ((goal[0] - rx)*cos(r_angle) + (goal[1] - ry)*sin(r_angle)) / goalDistance(goal)
    angle = acos(angle)
    return degrees(angle)

'Função que retorna o ângulo Z atual do robô em graus'
def getAngle():
    w = current_pose.pose.pose.orientation.w
    x = current_pose.pose.pose.orientation.x
    y = current_pose.pose.pose.orientation.y
    z = current_pose.pose.pose.orientation.z
    ax, ay, az = quaternion2EulerAngle(w, x, y, z)
    return az

'Função que converte os ângulos do robô, de quaternion para graus'
def quaternion2EulerAngle(w, x, y, z):
	t0 = +2 * (w * x + y * z)
	t1 = +1 - 2 * (x * x + y**2)
	X = degrees(atan2(t0, t1))
	t2 = +2 * (w * y - z * x)
	t2 = +1 if t2 > +1 else t2
	t2 = -1 if t2 < -1 else t2
	Y = degrees(asin(t2))
	t3 = +2 * (w * z + x * y)
	t4 = +1 - 2 * (y**2 + z * z)
	Z = degrees(atan2(t3, t4))
	return X, Y, Z

'Função que printa o log no terminal'
def logInfo(goal, log):
        print('[%s] GA: %6.2f   GD: %5.2f   CD: (%5.2f, %5.2f, %5.2f)'
            %(log, goalAngle(goal), goalDistance(goal), 
                current_laser[4], current_laser[3], current_laser[2]))

'Função principal'
def main():
    'Iniciar o nodo'
    rospy.init_node('Path', anonymous=True)
    
    'Definir o ponto de destino'
    goalx = 7
    goaly = 13

    'Iniciar os tópicos'
    sub = rospy.Subscriber("base_scan", LaserScan, lasercallback, queue_size=10)
    sub1 = rospy.Subscriber("base_pose_ground_truth", Odometry, posecallback, queue_size=10)
    vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
    
    'Número de quadros por ciclo'
    loop_rate = rospy.Rate(4)
    
    'Variáveis auxiliares'
    prvs_angle = 180
    prvs_side = 'R'
    speed = Twist()
    sleep(1)

    'Looping principal'
    while not rospy.is_shutdown():
        'Pegar posição atual e ângulo atual do robô e a distância atual entre o robô e o objetivo'
        robot_pose = getPose()
        goal_angle = goalAngle((goalx, goaly))
        goal_dist = goalDistance((goalx, goaly))

        'Definir para onde o robô vai andar'
        prvs_angle, prvs_side, log = setOrientation(goal_angle, goal_dist, prvs_angle, prvs_side, speed, vel_pub)
        
        'Printar o log no terminal'
        logInfo((goalx, goaly), log)

        loop_rate.sleep()

if __name__ == '__main__':
    try:
        main()
        exit()
    except rospy.ROSInterruptException:
        pass