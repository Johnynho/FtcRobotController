package org.firstinspires.ftc.teamcode.movimentos;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareClass;

@TeleOp(name="Teste Andar Encoder", group="Linear TesteOp")
public class EncoderCaboTeste extends LinearOpMode {

    HardwareClass robot   = new HardwareClass();
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode(){
        robot.hardwareGeral(hardwareMap);
        
        waitForStart();
        while(opModeIsActive()){
            encoderDrive(50,10,10, 5);
        }
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Turn On RUN_TO_POSITION
            robot.motorEsquerda.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorDireita.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.motorEsquerda.setPower(Math.abs(speed));
            robot.motorDireita.setPower(Math.abs(speed));


            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.motorEsquerda.isBusy() && robot.motorDireita.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.motorEsquerda.getCurrentPosition(),
                        robot.motorDireita.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.motorEsquerda.setPower(0);
            robot.motorDireita.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.motorEsquerda.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorDireita.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}
