package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="Autonomo Geral", group="Pushbot")
public class AutonomoGeral extends LinearOpMode {
    //Criando o objeto do TensorFlow
    TensorFlow tfEngine = new TensorFlow();

    //Declarar variaveis fora de um método
    double ticksPer;

    HardwareClass robot = new HardwareClass();
    Orientation angles;

    //Calculos do COUNTS_PER_INCH
    private static final double COUNTS_PER_MOTOR_GOBILDA = 537.6;   //CPR * 4 * Redução
    private static final double DRIVE_GEAR_REDUCTION = 1.0;      //Redução fora do motor
    private static final double WHEEL_CIRCUNFERENCE_INCHES = 11.87;     //Circunferência da roda em in
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_GOBILDA * DRIVE_GEAR_REDUCTION) /
            (WHEEL_CIRCUNFERENCE_INCHES);                        //Contagens por polegadas

    //Inicialização
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        //Inicia a engine do TensorFlow e avisa
        telemetry.addData("Status: ", "Iniciado");
        telemetry.update();
        robot.hardwareGeral(hardwareMap);
        tfEngine.initEngine(hardwareMap);

        robot.motorWobble.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.motorEsquerda.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.motorDireita.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.motorEsquerdaTras.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.motorDireitaTras.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "TensorFlow iniciado");
        telemetry.update();

        robot.motorPegWobble.setPower(-1);
        sleep(300);
        robot.motorPegWobble.setPower(-0.4);
        sleep(100);
        robot.motorWobble.setPower(1);
        sleep(800);
        robot.motorWobble.setPower(0);
        waitForStart();

        String quantArg = tfEngine.quantidadeDeArgolas();


        tfAutonomous(quantArg);

        //Teste Gyro
        /*double curangle = gyroCalculate();
        while (true) {
            curangle = gyroCalculate();
            telemetry.addData("Gyro:", curangle);
            telemetry.update();
        }*/

        tfEngine.deactivate();
    }

    public void tfAutonomous(String quantArg) {
        //Desliga Lanterna
        if(quantArg == "Quad"){
            //Faz a ação
            encoderDrive(0.8, 90, 90, 10);
            alinharGyro(-40, 0.3, 1);
            robot.motorWobble.setPower(-1);
            sleep(800);
            robot.motorWobble.setPower(0);
            sleep(100);
            robot.motorPegWobble.setPower(1);
            sleep(400);
            robot.motorPegWobble.setPower(0);
            sleep(1500);
            encoderDrive(0.8, -5, -5, 10);
            sleep(1500);
            alinharGyro(0, 0.4, 1);
            return;
        }

        if (quantArg == "Single") {
            //Faz a ação
            encoderDrive(0.8, 73, 73, 10);
            alinharGyro(85, 0.3, 1);
            robot.motorWobble.setPower(-1);
            sleep(800);
            robot.motorWobble.setPower(0);
            sleep(100);
            robot.motorPegWobble.setPower(1);
            sleep(400);
            robot.motorPegWobble.setPower(0);
            sleep(1500);
            encoderDrive(0.8, -5, -5, 10);
            sleep(1500);
            alinharGyro(0, 0.4, 1);
            return;
        }

        //Se for null faz a ação
        //Anda para perto do quadrado
        encoderDrive(0.4, 40, 40, 10);
        alinharGyro(-40, 0.3, 1);
        robot.motorWobble.setPower(-1);
        sleep(800);
        robot.motorWobble.setPower(0);
        sleep(100);
        robot.motorPegWobble.setPower(1);
        sleep(400);
        robot.motorPegWobble.setPower(0);
        sleep(1500);
        encoderDrive(0.8, -5, -5, 10);
        sleep(1500);
        alinharGyro(0, 0.4, 1);
    }

    public double rpmTP(int rpm){
        rpm/=60;
        int rpmMax = 6000/60;
        double rpmRev = rpm*28;
        rpmRev/=rpmMax;
        return rpmRev;
    }

    public void pegWobble(double power, int seg){

        robot.motorWobble.setPower(power);
        sleep(seg);
        robot.motorWobble.setPower(0);
    }

    //Angulo positivo == Esquerda
    //Angulo negativo == Direita
    public void alinharGyro(double angulo,double max, int timeout){
        //Declaração de variaveis
        double curangle = gyroCalculate();
        double erro = angulo-curangle;
        double out;
        double outfix = 0;
        double kp = 0.00000000000000005;
        double erroac = 1;
        //Transforma 1 milisegundo em 1 segundo (EXEMPLO PODERIA SER 5 OU 10)
        timeout*=1000;

            //Fala que enquanto o erro for maior que o menor erro que queremos chegar ele faz as comparações
            while(Math.abs(erro) > erroac) {
                //Atualiza o valor
                curangle = gyroCalculate();
                telemetry.addData("Gyro:", curangle);
                telemetry.update();

                //Deixa o output para colocar em velocidade
                out = kp * erro;

                //Faz a velocidade mínima
                if (negOrposi(out)){
                    if (Math.abs(out) < 0.2) {
                        outfix = 0.2;
                    }
                }else{
                    outfix = -0.2;
                }

                //Velocidade máxima
                if(negOrposi(out)){
                    if(Math.abs(out) > max){
                        outfix = max;
                    }
                }else{
                    outfix = -max;
                }

                //Começa o movimento
                robot.motorEsquerda.setPower(-outfix);
                robot.motorEsquerdaTras.setPower(-outfix);
                robot.motorDireita.setPower(outfix);
                robot.motorDireitaTras.setPower(outfix);
                erro = angulo - curangle;
            }
                //Para qualquer movimento
                robot.motorEsquerda.setPower(0);
                robot.motorEsquerdaTras.setPower(0);
                robot.motorDireita.setPower(0);
                robot.motorDireitaTras.setPower(0);
                sleep(timeout);
    }

    //Método para verificar se o número é negativo ou positivo
    public boolean negOrposi(double num){
        if(num > 0){
            return true;
        }
        return false;
    }

    public double gyroCalculate() {
        angles = HardwareClass.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
        int newLeftBTarget;
        int newRightBTarget;
        int newLeftTarget;
        int newRightTarget;
        double speedEsquerda;
        double speedDireita;
        double speedEsquerdaB;
        double speedDireitaB;
        speedEsquerda = leftInches > 0 ? speed : -speed;
        speedDireita = rightInches > 0 ? speed : - speed;
        speedEsquerdaB = leftInches > 0 ? speed : -speed;
        speedDireitaB = rightInches > 0 ? speed : - speed;

        int pp = (int) (robot.motorEsquerda.getCurrentPosition()/COUNTS_PER_INCH);

        newRightTarget = robot.motorDireita.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
        newLeftTarget = robot.motorEsquerda.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
        newRightBTarget = robot.motorDireita.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
        newLeftBTarget = robot.motorEsquerda.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
        robot.motorDireita.setTargetPosition(newRightTarget);
        robot.motorEsquerda.setTargetPosition(newLeftTarget);
        robot.motorDireitaTras.setTargetPosition(newRightBTarget);
        robot.motorEsquerdaTras.setTargetPosition(newLeftBTarget);

        //Confere se o opMode está ativo
        if (opModeIsActive()) {
            //Ativa o RUN_TO_POSITION
            robot.motorEsquerda.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorDireita.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorEsquerdaTras.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorDireitaTras.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //Reseta o runtime e começa o movimento
            runtime.reset();

            robot.motorEsquerda.setPower(Math.abs(speed));
            robot.motorDireita.setPower(Math.abs(speed));
            robot.motorEsquerdaTras.setPower(Math.abs(speed));
            robot.motorDireitaTras.setPower(Math.abs(speed));

            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (robot.motorEsquerda.isBusy() && robot.motorDireita.isBusy()) && (robot.motorEsquerdaTras.isBusy() && robot.motorDireitaTras.isBusy())) {

                telemetry.addData("Power Esquerda: ", robot.motorEsquerda.getPower());
                telemetry.addData("Power Direita: ", robot.motorDireita.getPower());
                telemetry.addData("Power Esquerda Tras: ", robot.motorEsquerdaTras.getPower());
                telemetry.addData("Power Direita Tras: ", robot.motorDireitaTras.getPower());
                telemetry.update();

            }

            //Para qualquer movimento
            robot.motorDireita.setPower(0);
            robot.motorEsquerda.setPower(0);
            robot.motorEsquerdaTras.setPower(0);
            robot.motorDireitaTras.setPower(0);

            //Desliga o RUN_TO_POSITION
            robot.motorEsquerda.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorDireita.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorEsquerdaTras.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorDireitaTras.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}
