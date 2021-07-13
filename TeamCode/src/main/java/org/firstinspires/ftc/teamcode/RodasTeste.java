package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Teste Rodas", group="Linear TesteOp")
public class RodasTeste extends LinearOpMode {

    //Membros do código
    ElapsedTime runtime = new ElapsedTime();
    DcMotor motorEsquerda = null;
    DcMotor motorDireita = null;
    DcMotor motorDireitoBack = null;
    DcMotor motorEsquerdoBack = null;

    @Override
    public void runOpMode() {
        int c2;
        double poderEsquerda;
        double poderDireita;
        double max;

        motorEsquerda = hardwareMap.get(DcMotor.class, "motor_Esquerda");
        motorDireita = hardwareMap.get(DcMotor.class, "motor_Direita");
        motorEsquerdoBack = hardwareMap.get(DcMotor.class, "motor_EsquerdaTras");
        motorDireitoBack = hardwareMap.get(DcMotor.class, "motor_DireitaTras");

        motorDireita.setDirection(DcMotor.Direction.REVERSE);
        motorDireitoBack.setDirection(DcMotorSimple.Direction.REVERSE);
        motorEsquerdoBack.setDirection(DcMotorSimple.Direction.FORWARD);
        motorEsquerda.setDirection(DcMotorSimple.Direction.FORWARD);


        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            //Declaração variáveis
            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;

            //Atribuindo o valor
            poderEsquerda = drive + turn;
            poderDireita = drive - turn;

            //Escolhendo o maior valor
            max = Math.max(Math.abs(poderEsquerda), Math.abs(poderDireita));

            //Proporcionalidade para não ultrapassar +/- 1
            if (max > 1.0) {
                poderEsquerda /= max;
                poderDireita /= max;
                c2 = 0;
            } else {
                if (max == 0) {
                    motorDireita.setPower(0);
                    motorEsquerda.setPower(0);
                    motorDireitoBack.setPower(0);
                    motorEsquerdoBack.setPower(0);

                }
                c2 = 0;
            }

            //Mandando a força para os motores
            if (c2 == 0) {
                motorDireita.setPower(poderDireita);
            } else if (c2 == 1) {
                motorDireitoBack.setPower(poderDireita);
            } else if (c2 == 2) {
                motorEsquerda.setPower(poderEsquerda);
            } else if (c2 == 3) {
                motorEsquerdoBack.setPower(poderEsquerda);
            }
        }
    }
}
