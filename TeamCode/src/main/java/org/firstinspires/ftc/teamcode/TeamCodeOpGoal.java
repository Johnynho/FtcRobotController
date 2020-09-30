package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="Teleoperado TeamCodeGoal", group="Linear TesteOp")
public class TeamCodeOpGoal extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    //Declaração Motores.
    DcMotor motorEsquerda, motorDireita = null;
    DcMotor motorEsquerdaTras = null;
    DcMotor motorDireitaTras = null;

    BNO055IMU imu;
    Orientation angles;
     //Respectivamente eixos do gamepad y, x, x outro analógico e temp = drive;
    double drive, turn, meca, temp;
    final double pi = 3.14;
    //Vetor para poderes;
    double []poder = new double[4];
    double power = 1;
    public void rotacao() {
        //Rotação motores tração.
        motorEsquerda.setDirection(DcMotor.Direction.FORWARD);
        motorDireita.setDirection(DcMotor.Direction.REVERSE);
        motorEsquerdaTras.setDirection(DcMotor.Direction.FORWARD);
        motorDireitaTras.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        //Configuração do gyro Hub
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";

        //Inicia os parâmetros
        //Pega o nome das variáveis no Dv.
        motorEsquerda = hardwareMap.get(DcMotor.class, "motor_Esquerda");
        motorEsquerdaTras = hardwareMap.get(DcMotor.class, "motor_EsquerdaTras");
        motorDireita = hardwareMap.get(DcMotor.class,"motor_Direita");
        motorDireitaTras = hardwareMap.get(DcMotor.class,"motor_DireitaTras");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);



        //Define a rotação dos motores.
        rotacao();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            //Variáveis gamepad
            drive = -gamepad1.left_stick_y;
            turn = gamepad1.left_stick_x * 1.5;
            meca = gamepad1.right_stick_x;

            processamentoGame(drive, turn);

            //Motor Esquerda Frente;
            poder[0] = drive + turn + meca;
            //Motor Esquerda trás;
            poder[1] = drive - turn + meca;
            //Motor Direita Frente;
            poder[2] = drive - turn - meca;
            //Motor Direita trás;
            poder[3] = drive + turn - meca;

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
            telemetry.addData("Motor Esquerdo", poder[0]);
            telemetry.addData("Motor EsquerdoTras", poder[1]);
            telemetry.addData("Motor Direita", poder[2]);
            telemetry.addData("Motor DireitaTras", poder[3]);
            telemetry.addData("Valor aleatorio teste", power);

            //Metodo setPower que manda força para os motores.
            motorEsquerda.setPower(poder[0]);
            motorEsquerdaTras.setPower(poder[1]);
            motorDireita.setPower(poder[2]);
            motorDireitaTras.setPower(poder[3]);
            telemetry.update();
            }
        }
        public void processamentoGame(double driveP, double turnP) {
            double angle = gyroCalculate() * pi/180;
            temp = driveP * Math.cos(angle) - turnP * Math.sin(angle);
            turn = driveP * Math.sin(angle) + turnP * Math.cos(angle);
            drive = temp;
        }
    public double gyroCalculate(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }
    }
