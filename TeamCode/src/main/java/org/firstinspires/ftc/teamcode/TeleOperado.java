package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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
    int baldeC = 0;
    int c2 = 0;
    int wb = 0;
    int c;
    double ticksPer;

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

        hard.motorWobbleEsq.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hard.motorWobbleDir.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hard.motorEsquerda.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hard.motorDireita.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hard.motorEsquerdaTras.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hard.motorDireitaTras.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        runtime.reset();

        //Espera o botão start na Ds
        waitForStart();

        while (opModeIsActive()) {

            /*
             * =============================================================================
             *                                TRAÇÃO
             * =============================================================================
             */

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

            /*
             * =============================================================================
             *                                INTAKE
             * =============================================================================
             */

            //Toggle do intake para pegar argolas assim como para soltar
            if (gamepad1.right_bumper && !antiBumper) {
                hard.motorIntake.setPower(onOff ? 1 : 0);
                onOff = !onOff;
                antiBumper = true;
            } else if (!gamepad1.right_bumper) {
                antiBumper = false;
                if (gamepad1.right_trigger > 0) {
                    hard.motorIntake.setPower(-1);
                    onOff = false;
                    //Só desativar caso esteja rodando ao contrário, para não dar conflito com o toggle
                } else if (hard.motorIntake.getPower() == -1) {
                    hard.motorIntake.setPower(0);
                }
            }

            /*
             * =============================================================================
             *                              WOBBLE GOAL
             * =============================================================================
             */

            //No primeiro aperto do botão B apenas levanta a chapa
            if (gamepad1.b) {
                hard.servoWobble.setPosition(1);
            }

            if (gamepad1.a) {
                hard.servoWobble.setPosition(0);
            }


            //Controle para não atirar se o balde não estiver levantado
            /*if (gamepad1.left_bumper) {
                hard.motorWobbleEsq.setPower(-1);
                hard.motorWobbleDir.setPower(-1);
            } else if (gamepad1.left_trigger > 0) {
                hard.motorWobbleEsq.setPower(1);
                hard.motorWobbleDir.setPower(1);
            } else {
                hard.motorWobbleEsq.setPower(0);
                hard.motorWobbleDir.setPower(0);
            }
            telemetry.addData("PODER ESQUERDA: ", hard.motorWobbleEsq.getPower());
            telemetry.addData("PODER DIREITA: ", hard.motorWobbleDir.getPower());
            telemetry.update();*/

            /*
             * =============================================================================
             *                                SHOOTER
             * =============================================================================
             */

            if (gamepad1.left_trigger > 0 && baldeC == 1) {
                hard.servoShootar.setPosition(1);
                hard.servoShootar.setPosition(0);
            }

            //Teste Shooter
            while (gamepad1.x) {
                c = 0;
                telemetry.addData("Velocidade em ticks:", hard.rpmMotor.getMotorVelocity(1));
                telemetry.update();
                if (c == 0) {
                    ticksPer = rpmTP(5000);
                    hard.rpmMotor.setMotorVelocity(1, ticksPer);
                }
            }
            if (!gamepad1.x) {
                c = 1;
            }
            ticksPer = rpmTP(0);
            hard.rpmMotor.setMotorVelocity(1, ticksPer);

            /*
             * =============================================================================
             *                                VUFORIA
             * =============================================================================
             */

            //Chama a leitura do Vuforia (somente verificar se o alvo está visível)
            /*vuforiaObj.vuforiaPosi();

            if (vuforiaObj.visible() && gamepad1.x ^ gamepad1.a){
                //Variável que somente verifica se algum dos botões foi apertado
                boolean xApertado = gamepad1.x;
                aut.alinharGyro(90, 0.5, 1);

                aut.alinharGyro(2, 0.5, 1);

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
            telemetry.update();*/
        }
    }

    public void pegWobble(double power, int seg){
        seg*=1000;
        hard.motorWobbleEsq.setPower(power);
        hard.motorWobbleDir.setPower(power);
        sleep(seg);
        hard.motorWobbleEsq.setPower(0);
        hard.motorWobbleDir.setPower(0);
    }

    public double rpmTP(int rpm){
        rpm/=60;
        int rpmMax = 6000/60;
        double rpmRev = rpm*28;
        rpmRev/=rpmMax;
        return rpmRev;
    }

    private double gyroCalculate() {
        angles = HardwareClass.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }
}
