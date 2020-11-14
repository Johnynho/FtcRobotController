package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="Teleoperado TeamCodeGoal", group="Linear TesteOp")
public class TeleOperado extends LinearOpMode {

     ElapsedTime runtime = new ElapsedTime();

     HardwareClass hard = new HardwareClass();

    Orientation angles;
    double angle;

    //Respectivamente eixos do gamepad y, x, x outro analógico e temp = drive;
    double drive, turn, giro, temp;
    final double pi = 3.1415926;

    //Vetor para poderes;
    double []poder = new double[4];

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        hard.hardwareGeral(hardwareMap);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            //Variáveis gamepad
            drive = -gamepad1.left_stick_y;
            turn = gamepad1.left_stick_x * 1.5;
            giro = gamepad1.right_stick_x;

            processamentoGame(drive, turn);

            //Motor Esquerda Frente;
            poder[0] = drive + turn + giro;
            //Motor Esquerda trás;
            poder[1] = drive - turn + giro;
            //Motor Direita Frente;
            poder[2] = drive - turn - giro;
            //Motor Direita trás;
            poder[3] = drive + turn - giro;

            if (Math.abs(poder[0]) > 1 || Math.abs(poder[1]) > 1
                    || Math.abs(poder[2]) > 1 || Math.abs(poder[3]) > 1) {

                //Achar o maior valor
                double max;
                max = Math.max(Math.abs(poder[0]), Math.abs(poder[1]));
                max = Math.max(Math.abs(poder[2]), max);
                max = Math.max(Math.abs(poder[3]), max);

                //Não ultrapassar +/-1 (proporção);
                poder[0] /= max;
                poder[1] /= max;
                poder[2] /= max;
                poder[3] /= max;
                }

            //Metodo setPower que manda força para os motores.
            hard.motorEsquerda.setPower(poder[0]);
            hard.motorEsquerdaTras.setPower(poder[1]);
            hard.motorDireita.setPower(poder[2]);
            hard.motorDireitaTras.setPower(poder[3]);

            //Telemetria com os valores de cada roda
            telemetry.addData("Motor Esquerdo", poder[0]);
            telemetry.addData("Motor EsquerdoTras", poder[1]);
            telemetry.addData("Motor Direita", poder[2]);
            telemetry.addData("Motor DireitaTras", poder[3]);

            telemetry.update();
            }
        }
        private void processamentoGame(double driveP, double turnP) {
            angle = gyroCalculate() * pi/180;
            temp = driveP * Math.cos(angle) - turnP * Math.sin(angle);
            turn = driveP * Math.sin(angle) + turnP * Math.cos(angle);
            drive = temp;
        }
        private double gyroCalculate(){
        angles = HardwareClass.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
        }
    }