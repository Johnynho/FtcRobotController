package org.firstinspires.ftc.teamcode.NewTestes;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Este código tem como finalidade tentar usar a variação do ângulo Z para tornar o movimento em
 * um código autônomo sendo ele RETO, de forma a não fazer curvas espontâneas
 */

public class GyroThread extends HardwareClass implements Runnable{
    Orientation anglesPara;
    double axeZ;

    HardwareClass robot2 = new HardwareClass();
    @Override
    public void run() {
        anglesPara = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        axeZ = anglesPara.firstAngle;

        if(axeZ > 0) {
            robot2.motorEsquerda.setPower(AutonomoEncoderGyro.DRIVE_SPEED - 0.2);
        }
        if(axeZ < 0) {
            robot2.motorDireita.setPower(AutonomoEncoderGyro.DRIVE_SPEED - 0.2);
        }
    }
}
