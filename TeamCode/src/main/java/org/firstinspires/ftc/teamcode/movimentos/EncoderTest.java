package org.firstinspires.ftc.teamcode.movimentos;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Teste Encoder", group="Linear TesteOp")
public class EncoderTest extends LinearOpMode {
    private static final double     COUNTS_ENCODER          = 8192;
    private static final double     WHEEL_DIAMETER_INCHES   = 2.36;     //Diâmetro da roda em in
    private static final double     COUNTS_PER_INCH         = (COUNTS_ENCODER) /
            (WHEEL_DIAMETER_INCHES);                        //Contagens por polegadas

    public DcMotor encoder_esquerda = null;

    @Override
    public void runOpMode(){
        double encoderRoda;
        double encoder;

        encoder_esquerda = hardwareMap.get(DcMotor.class,"Encoder_Esquerda");
        encoder_esquerda.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder_esquerda.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while(opModeIsActive()) {

            encoder = encoder_esquerda.getCurrentPosition();
            encoderRoda = encoder/COUNTS_PER_INCH;

            telemetry.addData("Giro da roda","Format Encoder %.2f",encoderRoda);
            telemetry.addData("Value com roda Ominiwheel 60mm","Encoder %f", encoder);
            telemetry.update();
        }
    }
}
