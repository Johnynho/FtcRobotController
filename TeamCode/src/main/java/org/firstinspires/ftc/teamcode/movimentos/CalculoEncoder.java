package org.firstinspires.ftc.teamcode.movimentos;

import org.firstinspires.ftc.teamcode.HardwareClass;

public class CalculoEncoder {
    HardwareClass robot = new HardwareClass();
    //Constantes de valores de encoder
    private static final double     COUNTS_PER_MOTOR_GOBILDA  = 537.6;   //CPR * 4 * Redução
    private static final double     DRIVE_GEAR_REDUCTION    = 1.0;      //Redução fora do motor
    private static final double     WHEEL_DIAMETER_INCHES   = 3.7;     //Diâmetro da roda em in
    private static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_GOBILDA * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);                        //Contagens por polegadas

    public double newLeftTarget;
    public double newRightTarget;

    //Calculo para saber quanto cada roda precisa andar
    private void encoderCalc(double rightIn, double leftIn) {
        newLeftTarget =  robot.motorEsquerda.getCurrentPosition() + (int)(leftIn * COUNTS_PER_INCH);
        newRightTarget = robot.motorDireita.getCurrentPosition() + (int)(rightIn * COUNTS_PER_INCH);
    }

    //Método para setar valor com vuforia
    public void setDistanceVuf(double valor) {
        encoderCalc(valor, valor);
    }

    //Para as rodas (utilizar no autônomo)
    public void setDistancWheels(double valor) {
        encoderCalc(valor, valor);
    }

    //Manipulação da variável que retorna a posição da roda Direita
    public double getDireita() {
        return newRightTarget;
    }
    //
    //Manipulação da variável que retorna a posição da roda Esquerda
    public double getEsquerda() {
        return newLeftTarget;
    }

}
