package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="Teste Andar Encoder", group="Pushbot")
public class AutonomoGeral extends LinearOpMode {
    //Criando o objeto do TensorFlow
    TensorFlow tfEngine = new TensorFlow();

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
    DcMotor motorEsquerda, motorDireita, motorEsquerdaTras, motorDireitaTras;

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

        alinharGyro(90, 0.6, 2);

        //Liga a lanterna
        /*CameraDevice.getInstance().setFlashTorchMode(true);

        sleep(2000);
        tfCounter(quantArg);

        tfEngine.deactivate();

        int pp = (int) (motorEsquerda.getCurrentPosition() / COUNTS_PER_INCH);
        telemetry.addData("Polegadas percorridas", pp);
        telemetry.update();*/
    }

    public void tfCounter(String quantArg) {
        if(quantArg == "Quad"){
            //Desliga Lanterna
            CameraDevice.getInstance().setFlashTorchMode(false);
            //Faz a ação
            encoderDrive(0.2, 20, -20, 5);
            return;
        }

        if (quantArg == "Single") {
            //Desliga Lanterna
            CameraDevice.getInstance().setFlashTorchMode(false);
            //Faz a ação
            encoderDrive(0.2, -20, 20, 5);
            return;
        }

        //Se for null
        //Desliga Lanterna
        CameraDevice.getInstance().setFlashTorchMode(false);
        //Faz a ação
        encoderDrive(0.6, 20, 20, 5);
    }

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
                gyroCalculate();

                //Deixa o output para colocar em velocidade
                out = kp * erro;

                //Faz a velocidade mínima
                if (negOrposi(out)){
                    if (Math.abs(out) < 0.3) {
                        outfix = 0.3;
                    }
                }else{
                    outfix = -0.3;
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
            motorEsquerdaTras.setPower(Math.abs(speed));
            motorDireitaTras.setPower(Math.abs(speed));


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
