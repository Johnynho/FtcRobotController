package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.MagneticFlux;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@TeleOp(name="Teleoperado TeamCodeGoal", group="Linear TesteOp")
public class TeamCodeOpGoal extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    //Declaração Motores.
    DcMotor motorEsquerda, motorDireita = null;
    DcMotor motorEsquerdaTras = null;
    DcMotor motorDireitaTras = null;

    //Declaração objetos classe de referência a gyro.
     SensorBNO055IMU a = new SensorBNO055IMU();

     //Respectivamente y, x, x outro analógico e temp = drive;
    double drive, turn, meca, temp;
    final float pi = 3.1415926f;
    //Vetor para poderes;
    double []poder = new double[4];

    //Metodos normais de hardware
    public DcMotor getName(String name) {
        return hardwareMap.get(DcMotor.class, name);
    }

    public Servo getNameS(String name) {
        return hardwareMap.get(Servo.class, name);
    }

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

        //Calibra o gyro
        a.gyroCalibrate();

        //Pega o nome das variáveis no Dv.
        motorEsquerda = getName("motor_Esquerda");
        motorEsquerdaTras = getName("motor_EsquerdaTras");
        motorDireita = getName("motor_Direita");
        motorDireitaTras = getName("motor_DireitaTras");

        //Define a rotação dos motores.
        rotacao();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            //Variáveis gamepad
            drive = gamepad1.left_stick_y * -1;
            turn = gamepad1.left_stick_x;
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
            //Metodo setPower que manda força para os motores.
            motorEsquerda.setPower(poder[0]);
            motorEsquerdaTras.setPower(poder[1]);
            motorDireita.setPower(poder[2]);
            motorDireitaTras.setPower(poder[3]);
            }
        }
        public void processamentoGame(double driveP, double turnP) {
            double angle = (a.angleZ) * pi/180;
            this.temp = driveP * Math.cos(angle) - turnP * Math.sin(angle);
            this.turn = driveP * Math.sin(angle) + turnP * Math.cos(angle);
            this.drive = this.temp;
        }
    }
