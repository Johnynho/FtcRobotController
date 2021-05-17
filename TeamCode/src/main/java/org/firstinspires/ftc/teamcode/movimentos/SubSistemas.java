package org.firstinspires.ftc.teamcode.movimentos;

import org.firstinspires.ftc.teamcode.AutonomoGeral;
import org.firstinspires.ftc.teamcode.HardwareClass;

/**
 * Programa que serve para se alinhar em Y (frente e trás visto da aliança vermelha)
 * também serve para se alinhar em X (frente e trás visto dos observadores)
 * Algumas considerações importantes:
 *
 * 1- Nossos motores são mechanum's (96mm ou 3.7") da goBILDA, chassi strafer kit
 *
 * 2- 312 RPM = 5.2 rotações por segundo
 *
 * 3- 5.2*3.7 = 19.24 (polegadas andadas em um segundo)
 *
 * 4- O certo do alinharX é fazer o robô andar por certo tempo (com base na distância)
 * e não retornar o tanto de voltas necessárias para isso
 */

public class SubSistemas{
    //Polegadas por segundo
    final static double inS = 19.24;

    //p também é polegadas em segundo
    //Usado para calcular polegadas por segundo sem ser velocidade máxima (19.24)
    double p;

    //Objeto para pegar os motores do HardwareClass
    HardwareClass hard = new HardwareClass();

    AutonomoGeral aut = new AutonomoGeral();

    //Método para alinhar em Y (aliança vermelha para azul, primeira sendo -Y)
    public void alinharY(double y, float a) {
        aut.alinharGyro(y, a, 1);
    }

    //X é a distância em relação ao setPoint (Vuforia)
    public void alinharX(double x, double v){
        aut.encoderDrive(v, x, x, 5 );
    }

    /*
     * Método que tem como objetivo retornar valores de 0 a 1 (input servo), mas receber graus
     * 180° = 1
     * 0° = 0
     */
    public static double servoPosicao(double angulo) {
         return angulo/180;
    }
}

