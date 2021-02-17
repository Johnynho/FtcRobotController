package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.movimentos.SubSistemas;


@TeleOp(name="Teleoperado Under Ctrl 14391", group="Linear TesteOp")
public class TeleOperado extends LinearOpMode {
    //Variáveis de controle dos ifs
    boolean antiBumper = false;
    boolean onOff = true;
    int c2 = 0;

    //Instanciação de objetos
    ElapsedTime runtime = new ElapsedTime();
    Vuforia vuforiaObj = new Vuforia();
    HardwareClass hard = new HardwareClass();
    SubSistemas ali = new SubSistemas();

    //String para configurar o vuforia (variável contém a cor da aliança)
    static String ladoO;

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
        vuforiaObj.configureVuforia("Azul", hardwareMap);
        //Ativa o vuforia
        vuforiaObj.ativeVuforia();

        runtime.reset();

        //Espera o botão start na Ds
        waitForStart();

        while (opModeIsActive()) {
            //Variáveis gamepad
            drive = -gamepad1.left_stick_y;
            turn = gamepad1.left_stick_x * 1.5;
            giro = gamepad1.right_stick_x;

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
            telemetry.addData("Motor Esquerdo %.2f", poder[0]);
            telemetry.addData("Motor EsquerdoTras %.2f", poder[1]);
            telemetry.addData("Motor Direita %.2f", poder[2]);
            telemetry.addData("Motor DireitaTras %.2f", poder[3]);
            telemetry.addData("A nossa aliança é a: ", ladoO);

            //Ativação do LED para saber que pode atirar
            hard.ledShooter.enableLight(true);

            //Toggle do intake para pegar argolas assim como para soltar
            if (gamepad1.right_bumper && !antiBumper) {
                hard.motorIntake.setPower(onOff ? 1 : 0);
                onOff = !onOff;
                antiBumper = true;
            } else if (!gamepad1.right_bumper) {
                antiBumper = false;
                if (gamepad1.left_bumper) {
                    hard.motorIntake.setPower(-1);
                    onOff = false;
                    //Só desativar caso esteja rodando ao contrário, para não dar conflito com o toggle
                } else if (hard.motorIntake.getPower() == -1) {
                    hard.motorIntake.setPower(0);
                }
            }

            //No primeiro aperto do botão B apenas levanta a chapa
            if (gamepad1.b && c2 == 0) {
                hard.ledShooter.enableLight(false);
                hard.servoPivo.setPosition(SubSistemas.servoPosicao(90));
                telemetry.addData("Chapa estado", "chapa levantada 1ª", 90);
                c2++;
                //Aqui verifica se a chapa está levantada com a váriavel C2 e o botão X apertado então o servo se fecha e a chapa levanta
            } else if (gamepad1.x && c2 == 1) {
                hard.servoChapa.setPosition(1);
                sleep(500);
                hard.servoPivo.setPosition(SubSistemas.servoPosicao(180));
                telemetry.addData("Chapa estado", "chapa levantada 2ª", 180);
                c2++;
                hard.ledShooter.enableLight(true);
         /*Verifica se o botão B foi apertado duas vezes e o servo está fechado se tudo estiver certo, a chapa se abaixa um pouco
         e abre o servo assim soltando o wobble goal*/
            } else if (gamepad1.b && c2 == 2) {
                hard.ledShooter.enableLight(false);
                hard.servoPivo.setPosition(SubSistemas.servoPosicao(160));
                telemetry.addData("Chapa estado", "chapa levantada 3ª", 160);
                hard.servoChapa.setPosition(0);
                sleep(1000);
                hard.servoPivo.setPosition(0);
                telemetry.addData("Chapa estado", "chapa levantada 4ª", 0);
                hard.ledShooter.enableLight(true);
                c2 = 0;
            }

            //Chama a leitura do Vuforia (somente verificar se o alvo está visível)
            vuforiaObj.vuforiaPosi();

            if (vuforiaObj.visible() && gamepad1.x ^ gamepad1.a){
                //Variável que somente verifica se algum dos botões foi apertado
                boolean xApertado = gamepad1.x;
                while(gyroCalculate() < 90 && gyroCalculate() > 0) {
                hard.motorEsquerda.setPower(0.2);
                hard.motorEsquerdaTras.setPower(0.2);
                hard.motorDireita.setPower(-0.2);
                hard.motorDireitaTras.setPower(-0.2);
                }

                while(gyroCalculate() > -90 && gyroCalculate() < 0) {
                hard.motorEsquerda.setPower(-0.2);
                hard.motorEsquerdaTras.setPower(-0.2);
                hard.motorDireita.setPower(0.2);
                hard.motorDireitaTras.setPower(0.2);
                }

                //Verifica se botão X foi apertado
                if(xApertado) {
                    //Pega a posição Y do robô  (1 = margem de erro)
                    while(vuforiaObj.posicaoRobot[1] < 1 ^ vuforiaObj.posicaoRobot[1] > 1) {
                        //Chamada do vuforia para saber a posição Y
                        vuforiaObj.vuforiaPosi();

                        //Método para alinhar em Y (envia o Y do vuforia, força para motores)
                        ali.alinharY(vuforiaObj.posicaoRobot[1], 0.5f);
                    }
                } else { //Caso não seja o botão X só sobra o A
                    //Pega a posição Y do robô  (1 = margem de erro)
                    while(vuforiaObj.posicaoRobot[1] < 10.5 ^ vuforiaObj.posicaoRobot[1] > 12.5) {
                        //Chamada do vuforia para saber a posição Y
                        vuforiaObj.vuforiaPosi();

                        //Método para alinhar em Y (envia o Y do vuforia, força para motores)
                        ali.alinharY(vuforiaObj.posicaoRobot[1], 0.5f);

                        //Fazer código que faz ele andar 17.5" pra esquerda/direita
                    }
                }

                //Chama o método para alinhar em X (sleep ativado)
                ali.alinharX(vuforiaObj.posicaoRobot[0], 0.8);
            }
            telemetry.update();
        }
    }

    private double gyroCalculate() {
        angles = HardwareClass.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }
}
