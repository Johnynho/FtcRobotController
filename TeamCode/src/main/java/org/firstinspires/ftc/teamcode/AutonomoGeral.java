package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
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

    //HardwareClass robot = new HardwareClass();
    BNO055IMU imu;
    Orientation angles;

    //Calculos do COUNTS_PER_INCH
    private static final double COUNTS_PER_MOTOR_GOBILDA = 537.6;   //CPR * 4 * Redução
    private static final double DRIVE_GEAR_REDUCTION = 1.0;      //Redução fora do motor
    private static final double WHEEL_CIRCUNFERENCE_INCHES = 11.87;     //Circunferência da roda em in
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_GOBILDA * DRIVE_GEAR_REDUCTION) /
            (WHEEL_CIRCUNFERENCE_INCHES);                        //Contagens por polegadas

    //Inicialização
    private final ElapsedTime runtime = new ElapsedTime();
    //Servo servoChapa;
    DcMotor motorEsquerda, motorDireita, motorEsquerdaTras, motorDireitaTras, motorChapa1, motorChapa2, motorShooter;
    DcMotorControllerEx rpmMotor;

    @Override
    public void runOpMode() {
        //Inicia a engine do TensorFlow e avisa
        telemetry.addData("Status: ", "Iniciado");
        telemetry.update();
        tfEngine.initEngine(hardwareMap);

        telemetry.addData("Status", "TensorFlow iniciado");
        telemetry.update();

        /* ***************************
               CONFIGURAÇÃO DO GYRO
         *************************** */
        //Configura o gyro
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";

        //Imu no Drive Station
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        //Inicializa os parametros do gyro
        imu.initialize(parameters);

        /* ***************************
            CONFIGURAÇÃO DOS MOTORES
         *************************** */

        //Parte da inicialização
        //servoChapa = hardwareMap.get(Servo.class,"servo_Chapa");
        motorShooter = hardwareMap.get(DcMotor.class, "motor_Shooter");
        motorChapa1 = hardwareMap.get(DcMotor.class, "motor_Chapa1");
        motorChapa2 = hardwareMap.get(DcMotor.class, "motor_Chapa2");
        motorEsquerda = hardwareMap.get(DcMotor.class, "motor_Esquerda");
        motorDireita = hardwareMap.get(DcMotor.class, "motor_Direita");
        motorEsquerdaTras = hardwareMap.get(DcMotor.class, "motor_Esquerdatras");
        motorDireitaTras = hardwareMap.get(DcMotor.class, "motor_DireitaTras");

        //Coloca as direções
        motorEsquerda.setDirection(DcMotor.Direction.REVERSE);
        motorDireita.setDirection(DcMotor.Direction.FORWARD);
        motorEsquerdaTras.setDirection(DcMotor.Direction.REVERSE);
        motorDireitaTras.setDirection(DcMotor.Direction.FORWARD);
        motorChapa1.setDirection(DcMotor.Direction.FORWARD);
        motorChapa2.setDirection(DcMotor.Direction.FORWARD);
        //servoChapa.setDirection(Servo.Direction.FORWARD);
        motorShooter.setDirection(DcMotor.Direction.FORWARD);

        //Configuração do encoder
        motorShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorDireita.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDireita.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorEsquerda.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorEsquerda.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        String quantArg = tfEngine.quantidadeDeArgolas();

        //Liga a lanterna
        //CameraDevice.getInstance().setFlashTorchMode(true);

        int portaShooter = motorShooter.getPortNumber();
        //Teste Shooter
        /*int c = 0;
        sleep(5000);
        while(gamepad1.x) {
            telemetry.addData("Velocidade em ticks:", rpmMotor.getMotorVelocity(0));
            telemetry.update();
            if(c == 0) {
                ticksPer = rpmTP(5000);
                rpmMotor.setMotorVelocity(portaShooter, ticksPer);
                c++;
            }
        }
        motorEsquerda.setPower(0);*/

        //Chapa teste
        motorChapa1.setPower(0.7);
        motorChapa2.setPower(0.7);
        sleep(1000);
        motorChapa1.setPower(0);
        motorChapa2.setPower(0);

        //tfAutonomous(quantArg);

        tfEngine.deactivate();

        int pp = (int) (motorEsquerda.getCurrentPosition() / COUNTS_PER_INCH);
        telemetry.addData("Polegadas percorridas", pp);
        telemetry.update();
    }

    public void tfAutonomous(String quantArg) {
        //Desliga Lanterna
        CameraDevice.getInstance().setFlashTorchMode(false);
        if(quantArg == "Quad"){
            //Desliga Lanterna
            //Faz a ação
            encoderDrive(0.2, 20, -20, 5);
            return;
        }

        if (quantArg == "Single") {
            //Faz a ação
            //Levanta o pegador
            //motorChapa.setPower(0.5);
            sleep(3000);
            //motorChapa.setPower(0);
            encoderDrive(0.5, 85.63, 85.63, 5);
            //Alinha com a área B
            alinharGyro(85, 0.5, 2);
            //Levanta o pegador
            //motorChapa.setPower(-0.5);
            sleep(3000);
            //motorChapa.setPower(0);
            //"Cospe" o wobble goal
            //servoChapa.setPosition(1);
            //Fica reto novamente
            alinharGyro(2, 0.5, 2);
            //Vai pra linha
            encoderDrive(0.5, -71.88, -71.88, 5);
            sleep(1000);
            //Alinha com o wobble
            alinharGyro(85, 0.5, 2);
            //Anda até ele
            encoderDrive(0.4, 22.75, 22.75, 5);
            //Pega o wobble
            //servoChapa.setPosition(0);
            sleep(3000);
            //Volta pra linha
            encoderDrive(0.4, -22.75, -22.75, 5);
            //Alinha para ir na area B
            alinharGyro(2, 0.5, 2);
            //Vai até a area
            encoderDrive(0.5, 92.25, 92.25, 5);
            //Alinha com a area
            alinharGyro(45, 0.5, 2);
            //"Cospe" o wobble goal
            //servoChapa.setPosition(1);
            sleep(3000);
            //Alinha novamente para voltar a linha
            alinharGyro(2, 0.5, 2);
            //Volta a linha de chegada
            encoderDrive(0.5, -101.25, -101.25, 5);
            return;
        }

        //Se for null faz a ação
        //Levanta o pegador
        //motorChapa.setPower(0.5);
        //sleep(1000);
        //motorChapa.setPower(0);
        //Anda até perto do meio da quadra
        encoderDrive(0.5, 58, 62, 5);
        //Gira para mirar no quadrado
        alinharGyro(-65, 0.5, 2);
        //Abaixa o pegador
        //motorChapa.setPower(-0.5);
        sleep(3000);
        //"Cospe" o wobble goal com a roda do servo
        //motorChapa.setPower(0);
        //servoChapa.setPosition(1);
        //Alinha para andar pra trás
        alinharGyro(-2, 0.5, 2);
        //Vai até a linha para alinhar com o segundo wobble goal
        encoderDrive(0.5, -37.75, -37.75, 5);
        //Gira para ficar de cara com o segundo wobble goal
        alinharGyro(85, 0.5, 1);
        //Anda até perto dele
        encoderDrive(0.3, 22.75, 22.75, 5);
        //Pega o wobble goal
        //servoChapa.setPosition(0);
        //Levanta denovo o pegador
        //motorChapa.setPower(0.5);
        sleep(3000);
        //motorChapa.setPower(0);
        //Anda para a linha novamente
        encoderDrive(0.3, -22.75, -22.75, 5);
        //Alinha com a área de entrega
        alinharGyro(-2, 0.5, 2);
        //Anda até a área de entrega com o segundo wobble goal
        encoderDrive(0.4, 28.75, 28.75, 5);
        //Gira para poder "Cospir" o wobble goal dentro a área
        alinharGyro(-35, 0.5, 2);
        //Abaixa o pegador
        //motorChapa.setPower(-0.5);
        sleep(3000);
        //motorChapa.setPower(0);
        //Cospe o wobble goal
        //servoChapa.setPosition(1);
        //Fica reto novamente
        alinharGyro(2, 0.5, 2);
        //Vai até a linha de lançamento
        encoderDrive(0.4, -51.5, -51.5, 5);
    }

    public double rpmTP(int rpm){
        rpm/=60;
        int rpmMax = 6000/60;
        double rpmRev = rpm*28;
        rpmRev/=rpmMax;
        return rpmRev;
    }

    //Angulo positivo == Esquerda
    //Angulo negativo == Direita
    public void alinharGyro(double angulo,double max, int timeout){
        //Declaração de variaveis
        double curangle = gyroCalculate();
        double erro = angulo-curangle;
        double out;
        double outfix = 0;
        double kp = 0.001;
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
                        outfix = 0.3;
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
                motorEsquerda.setPower(-outfix);
                motorEsquerdaTras.setPower(-outfix);
                motorDireita.setPower(outfix);
                motorDireitaTras.setPower(outfix);
                erro = angulo - curangle;
            }
                //Para qualquer movimento
                motorEsquerda.setPower(0);
                motorEsquerdaTras.setPower(0);
                motorDireita.setPower(0);
                motorDireitaTras.setPower(0);
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
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        double speedEsquerda;
        double speedDireita;
        speedEsquerda = leftInches > 0 ? speed : -speed;
        speedDireita = rightInches > 0 ? speed : - speed;

        int pp = (int) (motorEsquerda.getCurrentPosition()/COUNTS_PER_INCH);

        newRightTarget = motorDireita.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
        newLeftTarget = motorEsquerda.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
        motorDireita.setTargetPosition(newRightTarget);
        motorEsquerda.setTargetPosition(newLeftTarget);

        //Confere se o opMode está ativo
        if (opModeIsActive()) {
            //Ativa o RUN_TO_POSITION
            motorEsquerda.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorDireita.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //Reseta o runtime e começa o movimento
            runtime.reset();

            motorEsquerda.setPower(Math.abs(speed));
            motorDireita.setPower(Math.abs(speed));
            motorEsquerdaTras.setPower(speedEsquerda);
            motorDireitaTras.setPower(speedDireita);

            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (motorEsquerda.isBusy() && motorDireita.isBusy())) {

                //Mostra para o piloto informações sobre o caminho
                telemetry.addData("Path2",  "Running at %7d", motorEsquerda.getCurrentPosition());
                telemetry.addData("Polegadas percorridas", pp);
                telemetry.update();
            }

            //Para qualquer movimento
            motorDireita.setPower(0);
            motorEsquerda.setPower(0);
            motorEsquerdaTras.setPower(0);
            motorDireitaTras.setPower(0);

            //Desliga o RUN_TO_POSITION
            motorEsquerda.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorDireita.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}
