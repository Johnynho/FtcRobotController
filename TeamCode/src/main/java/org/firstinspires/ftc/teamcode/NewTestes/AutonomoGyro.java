package org.firstinspires.ftc.teamcode.NewTestes;

import com.qualcomm.robotcore.hardware.PIDCoefficients;


/*public class AutonomoGyro extends HardwareClass implements Runnable{
    AutonomoEncoderGyro robot = new AutonomoEncoderGyro();
    PIDCoefficients pidDrive = new PIDCoefficients(1, 0, 0);
    int setPoint, integral = 0;
    double error;
    double outPut;

    public void run() {
        PID();
        //if(robot.gyroCalculate() < 0) {
            robot.speedDireita = -outPut;
            robot.speedEsquerda = outPut;
        } else if (robot.gyroCalculate() > 0) {
            robot.speedDireita = outPut;
            robot.speedEsquerda = -outPut;
        }
    }
    public void PID() {
        error = setPoint - robot.gyroCalculate();
        integral += error;
        outPut = pidDrive.p*error + pidDrive.i*integral;
    }

}*/
