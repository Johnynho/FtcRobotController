package org.firstinspires.ftc.teamcode.movimentos;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareClass;

@TeleOp(name="Teste Andar Encoder", group="Linear TesteOp")
public class EncoderCaboTeste extends LinearOpMode {

    private static final double     COUNTS_PER_MOTOR_GOBILDA  = 537.6;   //CPR * 4 * Redução
    private static final double     DRIVE_GEAR_REDUCTION    = 1.0;      //Redução fora do motor
    private static final double     WHEEL_DIAMETER_INCHES   = 3.7;     //Diâmetro da roda em in
    private static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_GOBILDA * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES);                        //Contagens por polegadas

    HardwareClass robot   = new HardwareClass();
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode(){
        waitForStart();
        encoderDrive(1.0,10,5);
        int pp = (int) (robot.motorEsquerda.getCurrentPosition()/COUNTS_PER_INCH);
        telemetry.addData("Polegadas percorridas", pp);
        telemetry.update();
    }

    public void encoderDrive(double speed, double leftInches, double timeoutS) {
        int newLeftTarget;

        newLeftTarget = robot.motorEsquerda.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
        robot.motorEsquerda.setTargetPosition(newLeftTarget);

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Turn On RUN_TO_POSITION
            robot.motorEsquerda.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.motorEsquerda.setPower(Math.abs(speed));
            robot.motorDireita.setPower(Math.abs(speed));


            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (robot.motorEsquerda.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path2",  "Running at %7d :%7d");
                        robot.motorEsquerda.getCurrentPosition();
                telemetry.update();
            }

            // Stop all motion;
            robot.motorEsquerda.setPower(0);
            robot.motorDireita.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.motorEsquerda.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}
