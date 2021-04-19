package org.firstinspires.ftc.teamcode.movimentos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Teste Andar Encoder", group="Pushbot")
public class EncoderCaboTeste extends LinearOpMode {

    //Calculos do COUNTS_PER_INCH
    private static final double     COUNTS_PER_MOTOR_GOBILDA  = 537.6;   //CPR * 4 * Redução
    private static final double     DRIVE_GEAR_REDUCTION    = 1.0;      //Redução fora do motor
    private static final double     WHEEL_DIAMETER_INCHES   = 3.7;     //Diâmetro da roda em in
    private static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_GOBILDA * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES);                        //Contagens por polegadas

    //Inicialização
    private final ElapsedTime runtime = new ElapsedTime();
    DcMotor motorEsquerda, motorDireita;

    @Override
    public void runOpMode(){

        //Parte da inicialização
        motorEsquerda = hardwareMap.get(DcMotor.class, "motor_Esquerda");
        motorDireita = hardwareMap.get(DcMotor.class,"motor_Direita");

        //Coloca as direções
        motorEsquerda.setDirection(DcMotor.Direction.REVERSE);
        motorDireita.setDirection(DcMotor.Direction.FORWARD);

        //Configuração do encoder
        motorDireita.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDireita.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorEsquerda.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorEsquerda.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        encoderDrive(0.75,10, 10,5);
        int pp = (int) (motorEsquerda.getCurrentPosition()/COUNTS_PER_INCH);
        telemetry.addData("Polegadas percorridas", pp);
        telemetry.update();
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


            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (motorEsquerda.isBusy() && motorDireita.isBusy())) {

                //Mostra para o piloto informações sobre o caminho
                telemetry.addData("Path2",  "Running at %7d", motorEsquerda.getCurrentPosition());
                telemetry.addData("Polegadas percorridas", pp);
                telemetry.update();
            }

            //Para qualquer movimento
            motorDireita.setPower(0);
            motorEsquerda.setPower(0);

            //Desliga o RUN_TO_POSITION
            motorEsquerda.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorDireita.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}
