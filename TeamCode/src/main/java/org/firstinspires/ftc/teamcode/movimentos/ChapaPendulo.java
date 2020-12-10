package org.firstinspires.ftc.teamcode.movimentos;

import org.firstinspires.ftc.teamcode.HardwareClass;

public class ChapaPendulo {
    HardwareClass robot = new HardwareClass();
    //Constantes de valores de encoder
    private static final double     COUNTS_PER_MOTOR_REV = 537.6;   //CPR * 4 * Redução
    private static final double     DRIVE_GEAR_REDUCTION    = 1.0;      //Redução fora do motor
    private static final double     WHEEL_DIAMETER_INCHES   = 3.7;     //Diâmetro da roda em in
    private static final double     MOVIMENTO_CHAPA        = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);                        //Contagens por polegadas


    

}
