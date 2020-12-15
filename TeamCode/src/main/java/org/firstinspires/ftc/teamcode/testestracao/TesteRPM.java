package org.firstinspires.ftc.teamcode.testestracao;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

@TeleOp(name="Teste de RPM Under Ctrl 14391", group="Linear TesteOp")
public class TesteRPM extends LinearOpMode {

    HardwareClassRPM robo = new HardwareClassRPM();

    DcMotorControllerEx porta0;
    DcMotorControllerEx porta1;

    double v1Ticks, v1Volta;
    double v2Ticks, v2Volta;
    int v1RPM;
    int v2RPM;
    double power;

    public void runOpMode() {
        //HardwareGeral
        robo.hardwareGeral(hardwareMap);

        //Retorna o tipo de motor
        MotorConfigurationType motor1 = porta0.getMotorType(0);
        MotorConfigurationType motor2 = porta1.getMotorType(1);

        //RPM para quantas voltas em 1 segundo
        double rpmPerSecond1 = motor1.getMaxRPM() / 60;
        double rpmPerSecond2 = motor2.getMaxRPM() / 60;

        //Pega o valor que precisa para chegar no max rpm
        v1RPM = (int)(motor1.getMaxRPM()/motor1.getTicksPerRev());
        v2RPM = (int)(motor2.getMaxRPM()/motor2.getTicksPerRev());

        while(opModeIsActive()) {
            if(gamepad1.dpad_up && power < 1) {
                power += 0.1;
                porta0.setMotorPower(0, power);
                porta1.setMotorPower(1, power);
                sleep(1000);
            } else if (gamepad1.dpad_down && power > -1) {
                power -= 0.1;
                porta0.setMotorPower(0, power);
                porta1.setMotorPower(1, power);
                sleep(1000);
            }
            //Retorna a velocidade do motor (ticks per second)
            v1Ticks = porta0.getMotorVelocity(0);
            v2Ticks = porta1.getMotorVelocity(1);

            //Ticks em 1 volta (1120)
            v1Volta = v1Ticks/rpmPerSecond1;
            v2Volta = v2Ticks/rpmPerSecond2;

            //Telemetria de ticks per second, ticks per rev e rpm
            telemetry.addData("Motor porta 1 in ticks per second  %.1f", v1Ticks);
            telemetry.addData("Motor porta 2 in ticks per second  %.1f", v2Ticks);
            telemetry.addData("Motor porta 1 in ticks per rev  %.1f", v1Volta);
            telemetry.addData("Motor porta 2 in ticks per rev  %.1f", v2Volta);
            telemetry.addData("Motor porta 1 in rpm  %.1f", v1RPM * v1Volta);
            telemetry.addData("Motor porta 2 in rpm  %.1f", v2RPM * v2Volta);
            telemetry.addData("Motor porta 2 in rpm  %.1f", power);

            telemetry.update();
        }
    }
}


