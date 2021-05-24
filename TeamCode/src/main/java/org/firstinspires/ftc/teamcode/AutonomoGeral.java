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
    DcMotorControllerEx rpmMotor;

    @Override
    public void runOpMode() {
        //Inicia a engine do TensorFlow e avisa
        telemetry.addData("Status: ", "Iniciado");
        telemetry.update();
        tfEngine.initEngine(hardwareMap);

        telemetry.addData("Status", "TensorFlow iniciado");
        telemetry.update();

        waitForStart();

        String quantArg = tfEngine.quantidadeDeArgolas();

        //Liga a lanterna
        //CameraDevice.getInstance().setFlashTorchMode(true);

        //int portaShooter = robot.motorShooter.getPortNumber();
        //Teste Shooter
        /*int c = 0;
        sleep(5000);
        while(gamepad1.x) {
            telemetry.addData("Velocidade em ticks:", rpmMotor.getMotorVelocity(portaShooter));
            telemetry.update();
            if(c == 0) {
                ticksPer = rpmTP(5000);
                rpmMotor.setMotorVelocity(portaShooter, ticksPer);
                c++;
            }
        }
        robot.motorEsquerda.setPower(0);*/

        //Chapa teste
        //pegWobble(0.7, 1);

        tfAutonomous(quantArg);

        tfEngine.deactivate();

        int pp = (int) (robot.motorEsquerda.getCurrentPosition() / COUNTS_PER_INCH);
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

            encoderDrive(0.5, 85.63, 85.63, 5);
            //Alinha com a área B
            alinharGyro(85, 0.5, 2);
            //Solta o wobble
            robot.servoWobble.setPosition(1);
            //Fica reto novamente
            alinharGyro(2, 0.5, 2);
            //Vai pra linha
            encoderDrive(0.5, -71.88, -71.88, 5);
            sleep(1000);
            //Alinha com o wobble
            alinharGyro(85, 0.5, 2);
            //Anda até ele
            encoderDrive(0.4, 22.75, 22.75, 5);
            //Abaixa pegardor
            pegWobble(-0.7, 1);
            //Pega o wobble
            robot.servoWobble.setPosition(0);
            //Levanta o braço
            pegWobble(0.7, 1);
            //Volta pra linha
            encoderDrive(0.4, -22.75, -22.75, 5);
            //Alinha para ir na area B
            alinharGyro(2, 0.5, 2);
            //Vai até a area
            encoderDrive(0.5, 92.25, 92.25, 5);
            //Alinha com a area
            alinharGyro(45, 0.5, 2);
            //Solta o wobble goal
            robot.servoWobble.setPosition(1);
            //Alinha novamente para voltar a linha
            alinharGyro(2, 0.5, 2);
            //Volta a linha de chegada
            encoderDrive(0.5, -101.25, -101.25, 5);
            return;
        }

        //Se for null faz a ação
        //Anda até perto do meio da quadra
        encoderDrive(0.5, 58, 62, 5);
        //Gira para mirar no quadrado
        alinharGyro(-65, 0.5, 2);
        //Solta o wobble goal
        robot.servoWobble.setPosition(1);
        //Levanta o pegador para locomoção
        pegWobble(0.7, 1);
        //Alinha para andar pra trás
        alinharGyro(-2, 0.5, 2);
        //Vai até a linha para alinhar com o segundo wobble goal
        encoderDrive(0.5, -37.75, -37.75, 5);
        //Gira para ficar de cara com o segundo wobble goal
        alinharGyro(85, 0.5, 1);
        //Abaixa o braço
        pegWobble(-0.7, 1);
        //Anda até perto dele
        encoderDrive(0.3, 22.75, 22.75, 5);
        //Pega o wobble goal
        robot.servoWobble.setPosition(0);
        //Levanta denovo o pegador
        pegWobble(0.7, 1);
        //Anda para a linha novamente
        encoderDrive(0.3, -22.75, -22.75, 5);
        //Alinha com a área de entrega
        alinharGyro(-2, 0.5, 2);
        //Anda até a área de entrega com o segundo wobble goal
        encoderDrive(0.4, 28.75, 28.75, 5);
        //Gira para poder "Cospir" o wobble goal dentro a área
        alinharGyro(-35, 0.5, 2);
        //Solta o wobble goal
        robot.servoWobble.setPosition(1);
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

    public void pegWobble(double power, int seg){
        seg*=1000;
        robot.motorWobbleEsq.setPower(power);
        robot.motorWobbleDir.setPower(power);
        sleep(seg);
        robot.motorWobbleEsq.setPower(0);
        robot.motorWobbleDir.setPower(0);
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
        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        double speedEsquerda;
        double speedDireita;
        speedEsquerda = leftInches > 0 ? speed : -speed;
        speedDireita = rightInches > 0 ? speed : - speed;

        int pp = (int) (robot.motorEsquerda.getCurrentPosition()/COUNTS_PER_INCH);

        newRightTarget = robot.motorDireita.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
        newLeftTarget = robot.motorEsquerda.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
        robot.motorDireita.setTargetPosition(newRightTarget);
        robot.motorEsquerda.setTargetPosition(newLeftTarget);

        //Confere se o opMode está ativo
        if (opModeIsActive()) {
            //Ativa o RUN_TO_POSITION
            robot.motorEsquerda.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorDireita.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //Reseta o runtime e começa o movimento
            runtime.reset();

            robot.motorEsquerda.setPower(Math.abs(speed));
            robot.motorDireita.setPower(Math.abs(speed));
            robot.motorEsquerdaTras.setPower(speedEsquerda);
            robot.motorDireitaTras.setPower(speedDireita);

            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (robot.motorEsquerda.isBusy() && robot.motorDireita.isBusy())) {

                //Mostra para o piloto informações sobre o caminho
                telemetry.addData("Path2",  "Running at %7d", robot.motorEsquerda.getCurrentPosition());
                telemetry.addData("Polegadas percorridas", pp);
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
        }
    }
}
