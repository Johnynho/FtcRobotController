package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@TeleOp(name="Teleoperado Under Ctrl 14391", group="Linear TesteOp")
public class TeleOperado extends LinearOpMode {

     ElapsedTime runtime = new ElapsedTime();

     private final HardwareClass hard = new HardwareClass();
     private final Vuforia h = new Vuforia();
     Orientation angles;

     //Respectivamente eixos do gamepad y, x, x outro analógico
     double drive, turn, giro;
     double pi = 3.1415926;

     //Vetor para poderes;
     private final double []poder = new double[4];

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Iniciando o hardware do robô (Acionadores, Vuforia e Gyro)
        hard.hardwareGeral(hardwareMap);

        //Iniciando a Thread paralela do vuforia
        Runnable vision = new Vuforia();
        Thread visionThread = new Thread(vision);
        visionThread.start();
        //Analisar qualquer erro na thread
        visionThread.setUncaughtExceptionHandler(h.handler);

        //Escolha de aliança pela seta esquerda ou direita
        //Respectivamente Azul e Vermelha
        while(!gamepad1.dpad_left ^ !gamepad1.dpad_right) {
            if(gamepad1.dpad_left) {
                h.setPointGoal("Azul");
                break;
            } else if(gamepad1.dpad_right) {
                h.setPointGoal("Vermelho");
                break;
            }
        }

        //Espera o start na Ds
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
            telemetry.addData("Motor Esquerdo %.2f", poder[0]);
            telemetry.addData("Motor EsquerdoTras %.2f", poder[1]);
            telemetry.addData("Motor Direita %.2f", poder[2]);
            telemetry.addData("Motor DireitaTras %.2f", poder[3]);

            if(gamepad1.right_bumper && !antiBumper) {
                hard.motorIntake.setPower(onOff ? 1 : 0);
                onOff = !onOff;
                antiBumper = true;
            }else if (!gamepad1.right_bumper) {
                antiBumper = false;
            }
            if (gamepad1.left_bumper) {
                hard.motorIntake.setPower(-1);
            }else{
                hard.motorIntake.setPower(0);
            }

            telemetry.update();
            }
        }

        private void processamentoGame(double driveP, double turnP) {
            double angle = gyroCalculate() * pi / 180;
            drive = driveP * Math.cos(angle) - turnP * Math.sin(angle);
            turn = driveP * Math.sin(angle) + turnP * Math.cos(angle);
        }
        private double gyroCalculate(){
        angles = HardwareClass.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
        }
    }
