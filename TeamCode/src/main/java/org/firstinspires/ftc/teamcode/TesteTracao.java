package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.movimentos.SubSistemas;


@TeleOp(name="teste tracao", group="Linear TesteOp")
public class TesteTracao extends LinearOpMode {
    //Variáveis de controle dos ifs
    boolean antiBumper = false;
    boolean onOff = true;
    int baldeC = 0;
    int c2 = 0;

    //Instanciação de objetos
    ElapsedTime runtime = new ElapsedTime();
    //Vuforia vuforiaObj = new Vuforia();
    HardwareClass hard = new HardwareClass();
    SubSistemas ali = new SubSistemas();

    //String para configurar o vuforia (variável contém a cor da aliança)
    //static String ladoO;

    //Referência de oritenação do gyro
    Orientation angles;

    //Respectivamente eixos do gamepad y, x, x outro analógico
    double drive, turn, giro;

    //Vetor para potência do motor
    private final double[] poder = new double[4];

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Inicia o hardware do robô
        hard.hardwareGeral(hardwareMap);
        //Manda o lado que estejamos jogando com base no autônomo e configura o vuforia
        //vuforiaObj.configureVuforia("Azul", hardwareMap);
        //Ativa o vuforia
        //vuforiaObj.ativeVuforia();

        runtime.reset();

        //Espera o botão start na Ds
        waitForStart();

        while (opModeIsActive()) {

            //Variáveis gamepad
            drive = -gamepad1.left_stick_y;
            turn = gamepad1.left_stick_x * 1.5;
            giro = Math.pow(gamepad1.right_stick_x, 3); //Exponenciação para arrumar sensibilidade

            //Valores para movimentação com mechanum (lados espelhados)
            //Motor Esquerda Frente;
            poder[0] = drive + turn + giro;
            //Motor Esquerda trás;
            poder[1] = drive - turn + giro;
            //Motor Direita Frente;
            poder[2] = drive - turn - giro;
            //Motor Direita trás;
            poder[3] = drive + turn - giro;

            //Verificar se algum valor é maior que 1
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
            hard.motorEsquerda.setPower(poder[0]);
            hard.motorEsquerdaTras.setPower(poder[1]);
            hard.motorDireita.setPower(poder[2]);
            hard.motorDireitaTras.setPower(poder[3]);

            //Telemetria com os valores de cada roda
            telemetry.addData("Motor Esquerdo: ", poder[0] % .2f);
            telemetry.addData("Motor EsquerdoTras: ", poder[1] % .2f);
            telemetry.addData("Motor Direita: ", poder[2] % .2f);
            telemetry.addData("Motor DireitaTras: ", poder[3] % .2f);
            //telemetry.addData("A nossa aliança é a: ", ladoO);
        }
    }
}

