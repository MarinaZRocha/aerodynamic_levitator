# Levitador Aerodinâmico

Trabalho desenvolvido como parte da avaliação da disciplina de Sistema de Controle, do curso de Engenharia de Automação da FURG.
Construção de um protótipo de um sistema de controle PID de um levitador aerodinâmico.

![imagem1](https://github.com/MarinaZRocha/aerodynamic_levitator/blob/master/foto4.jpeg)

## :notebook_with_decorative_cover: 	Materiais utilizados:
* 1 Tubo de plástico de 52cm de altura;
* 1 Esfera de isopor;
* 1 Motor de aeromodelismo com hélices adaptadas;
* 1 ESC;
* 1 Bateria;
* 1 Arduino UNO;
* 1 Sensor ultrassônico.

## :mag: Metodologia:
O controlador a ser desenvolvido tem o objetivo de manter a estabilidade de uma esfera de isopor flutuando em relação a uma referência pré estabelecida, além de corrigir qualquer tipo de distúrbio aplicado nesse sistema. Para isso, foi utilizado um controlador proporcional, integral e derivativo (PID), por sua maior precisão, resultando em melhor estabilidade.
Seu funcionamento consiste no sensor ultrassônico que identifica a altura no instante atual da bolinha, baseado na altura identificada compara-se com o valor pré-definido como referência, e, para que a bolinha atinja o valor estipulado como referência o controle aumenta e/ou diminui a rotação do motor.

### Diagrama de blocos
<p align="center">
<img src="https://github.com/MarinaZRocha/aerodynamic_levitator/blob/master/diagrama_blocos.jpeg" width="500">
</p>

### Diagrama do circuito
<p align="center">
<img src="https://github.com/MarinaZRocha/aerodynamic_levitator/blob/master/diagrama_circuito.jpeg" width="500">
</p>

## :link: Vídeo:
<p align="center">
<a href="http://www.youtube.com/watch?feature=player_embedded&v=n-pNwGUZQv8" target="_blank">
 <img src="https://github.com/MarinaZRocha/aerodynamic_levitator/blob/master/capa_video.png" width="800" alt="Watch the video" />
</a>
</p>
