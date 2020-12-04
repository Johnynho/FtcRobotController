package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;


@TeleOp(name="Teleoperado Under Ctrl 14391", group="Linear TesteOp")
public class TeleOperado extends LinearOpMode {

    boolean antiBumper = false;
    boolean onOff = true;

    ElapsedTime runtime = new ElapsedTime();

    HardwareClass hard = new HardwareClass();
    static String ladoO;
    //Thread visionThread;
    Orientation angles;

    //Respectivamente eixos do gamepad y, x, x outro analógico
    double drive, turn, giro;
    double pi = 3.1415926;
    //Vetor para poderes;
    private final double[] poder = new double[4];

    Vuforia a = new Vuforia();
    boolean targetVisible;
    OpenGLMatrix lastLocationGol;
    OpenGLMatrix lastLocationPS;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        hard.hardwareGeral(hardwareMap);
        a.configureVuforia("Azul", hardwareMap);
        a.ativeVuforia();

        runtime.reset();
        waitForStart();

        //TeleOperado g = new TeleOperado();
        //Thread t1 = new Thread(g);
        //t1.start();
        acessp();
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
            telemetry.addData("A nossa aliança é a: ", ladoO);
            acessp();
            telemetry.update();
        }

        if(gamepad1.right_bumper && !antiBumper) {
            hard.motorIntake.setPower(onOff ? 1 : 0);
            onOff = !onOff;
            antiBumper = true;
        }else if (!gamepad1.right_bumper) {
            antiBumper = false;
            if (gamepad1.left_bumper) {
                hard.motorIntake.setPower(-1);
                onOff = false;
            } else {
                hard.motorIntake.setPower(onOff ? 1 : 0);
            }
        }


    }

    private void processamentoGame(double driveP, double turnP) {
        double angle = gyroCalculate() * pi / 180;
        drive = driveP * Math.cos(angle) - turnP * Math.sin(angle);
        turn = driveP * Math.sin(angle) + turnP * Math.cos(angle);
    }

    private double gyroCalculate() {
        angles = HardwareClass.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    public void acessp() {
        telemetry.addData("none", "acesso");
        //Posições em X, Y e Z
        double[] posicaoGol = new double[3];
        double[] posicaoPS = new double[3];
        /*
         * ================================================================================
         *                                   GOLS
         * ================================================================================
         */
        //Verifica os targets visiveis
        targetVisible = false;
        for (VuforiaTrackable trackable : a.allTrackablesGol) {
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                telemetry.addData("Visible Target", trackable.getName());
                targetVisible = true;

                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocationGol = robotLocationTransform;
                }
                break;
            }
        }

        for (VuforiaTrackable trackable1 : a.allTrackablesPS) {
            if (((VuforiaTrackableDefaultListener) trackable1.getListener()).isVisible()) {

                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable1.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocationPS = robotLocationTransform;
                }
                break;
            }
        }
        //Parte do código que mostra a localização do robô
        if (targetVisible) {
            //Expressa a translação do robô em polegadas
            VectorF translation = lastLocationGol.getTranslation();
            posicaoGol[0] = translation.get(0) / Vuforia.mmPerInch; //Posição X
            posicaoGol[1] = translation.get(1) / Vuforia.mmPerInch; //Posição Y
            posicaoGol[2] = translation.get(2) / Vuforia.mmPerInch; //Posição Z
            telemetry.addData("Pos (in) Mirar Gol", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    posicaoGol[0], posicaoGol[1], posicaoGol[2]);

            //Expressa a translação do robô em polegadas
            VectorF translationPS = lastLocationGol.getTranslation();
            posicaoPS[0] = translationPS.get(0) / Vuforia.mmPerInch; //Posição X
            posicaoPS[1] = translationPS.get(1) / Vuforia.mmPerInch; //Posição Y
            posicaoPS[2] = translationPS.get(2) / Vuforia.mmPerInch; //Posição Z
            telemetry.addData("Pos (in) Mirar Power shot", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    posicaoPS[0], posicaoPS[1], posicaoPS[2]);

            //Rotação do robô em graus
            Orientation rotationGol = Orientation.getOrientation(lastLocationPS, EXTRINSIC, XYZ, DEGREES);
            telemetry.addData("Rot (deg) Power shot", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotationGol.firstAngle, rotationGol.secondAngle, rotationGol.thirdAngle);

        } else {
            telemetry.addData("Visible Target", "none");
        }
    }
}
