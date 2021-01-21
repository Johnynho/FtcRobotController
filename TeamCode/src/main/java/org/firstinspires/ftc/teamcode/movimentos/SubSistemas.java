package org.firstinspires.ftc.teamcode.movimentos;
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

    //Método para alinhar em Y (aliança vermelha para azuprimeira sendo -Y)
    public void alinharY(double y, float a) {

        //Valor para motores
        double p = y < 0 ? -a : a;

        //Sinais que irão fazer o robô se movimentar para esquerda/direita
        //Se for -a e entrar no setPower do motorEsquerdaTras, ele deve ficar positivo -(-a)
        hard.motorEsquerda.setPower(p);
        hard.motorEsquerdaTras.setPower(-p);
        hard.motorDireita.setPower(-p);
        hard.motorDireitaTras.setPower(p);
    }

    //X é a distância em relação ao setPoint (Vuforia)
    public void alinharX(double x, double v) throws InterruptedException {
        //p é polegadas em um segundo, só que sem ser somente na velocidade máxima
        //Exemplo: 19.24 = velocidade máxima, metade 19.24*0.5 (0.5 é um suposto input do motor)
        p = v * inS;

        //Retorna o tempo que o robô precisa se movimentar para alcançar a distância X
        double time = x/p;

        hard.motorEsquerda.setPower(v);
        hard.motorEsquerdaTras.setPower(v);
        hard.motorDireita.setPower(v);
        hard.motorDireitaTras.setPower(v);

        Thread.sleep((long) (time*1000));

        hard.motorEsquerda.setPower(0);
        hard.motorEsquerdaTras.setPower(0);
        hard.motorDireita.setPower(0);
        hard.motorDireitaTras.setPower(0);

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

