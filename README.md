# tangent
Trabalho 1 da disciplina de Sistemas Robóticos

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
Os opcodes informam o quê o robô está identificando. Para cada identificação há uma ação:

    [ none ] - Nada a fazer;
    [ frnt ] - Colisão frontal. Consequência: o robô vai virar para qualquer lado e, em seguida, vai dar ré;
    [cofntl] - Colisão esquerda-frontal. Consequência: o robô vai virar para a direita;
    [cofntr] - Colisão direita-frontal. Consequência: o robô vai virar para a esquerda;
    [colsde] - Colisão lateral. Consequência: o robô vai seguir reto com uma leve inclinação para os lados;
    [indirc] - Está na direção do destino. Consequência: o robô seguirá em frente com uma leve inclinação para os lados;
    [ntdir1] - Não está na diração do destino. Consequência: o robô irá rotacionar até achar a direção certa;
    [ntdir2] - Semelhante ao anterior, porém o robô irá rotacionar para o lado contrário;
