/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

/**
 * From the Audience perspective, the Red Alliance station is on the right and the
 * Blue Alliance Station is on the left.
 * There are a total of five image targets for the ULTIMATE GOAL game.
 * Three of the targets are placed in the center of the Red Alliance, Audience (Front),
 * and Blue Alliance perimeter walls.
 * Two additional targets are placed on the perimeter wall, one in front of each Tower Goal.
 * Refer to the Field Setup manual for more specific location details
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  ultimategoal/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 * =====================================================================================
 * Você está olhando da aliança vermeleha (tudo perspectiva);
 * - O eixo X é da sua esquerda para direita. (positivo do centro para direita)
 * - O eixo Y é da aliança vermelha para o outro lado do campo onde está a aliança azul
 * (Positive no centro, em direção a aliança azul)
 * - O eixo Z vai do chão para cima (Positivo é acima do chão)
**/
public class Vuforia extends Thread{

    //Variáveis genéricas de instância
    private final TeleOperado hard = new TeleOperado();
    private final HardwareClass init = new HardwareClass();

    // Constantes convertidas para converter polegadas para mm
    private static final float mmPerInch = 25.4f;
    //Definida pelo altura do centro das imagens do chão
    private static final float mmTargetHeight = (6) * mmPerInch;

    //Constante do tamanho de tatame da quadra quadra convertida
    private static final float blocks = 23f * mmPerInch;

    //Ultima posição do vuforia para Gol e Power Shot
    private OpenGLMatrix lastLocationGol = null;
    private OpenGLMatrix lastLocationPS = null;

    //Posições em X, Y e Z
    private final double [][]posicaoC = new double[2][3];

    @Override
    public void run() {
        /*
         * ================================================================================
         *                                   GOLS
         * ================================================================================
         */
        try {
            while (!isInterrupted()) {
                //Verifica os targets visiveis
                boolean targetVisibleGols = false;

                for (VuforiaTrackable trackable : init.allTrackablesGol) {
                    if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                        telemetry.addData("Visible Target", trackable.getName());
                        telemetry.addLine("Mirar Gol");
                        targetVisibleGols = true;

                        OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                        if (robotLocationTransform != null) {
                            lastLocationGol = robotLocationTransform;
                        }
                        break;
                    }
                }
                //Parte do código que mostra a localização do robô
                if (targetVisibleGols) {

                    //Expressa a translação do robô em polegadas
                    VectorF translation = lastLocationGol.getTranslation();
                    posicaoC[0][0] = translation.get(0) / mmPerInch; //Posição X
                    posicaoC[0][1] = translation.get(1) / mmPerInch; //Posição Y
                    posicaoC[0][2] = translation.get(2) / mmPerInch; //Posição Z
                    hard.telemetry.addData("Pos (in) Mirar Gol", "{X, Y, Z} = %.1f, %.1f, %.1f",
                            posicaoC[1][0], posicaoC[1][1], posicaoC[1][2]);

                    //Rotação do robô em graus
                    Orientation rotationGol = Orientation.getOrientation(lastLocationGol, EXTRINSIC, XYZ, DEGREES);
                    telemetry.addData("Rot (deg) Gol", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotationGol.firstAngle, rotationGol.secondAngle, rotationGol.thirdAngle);

                } else {
                    telemetry.addData("Visible Target Torre Gol", "none");
                }
                telemetry.update();
                /*
                 * ================================================================================
                 *                                   POWER SHOTS
                 * ================================================================================
                 */
                //Verifica os targets visiveis
                boolean targetVisiblePs = false;

                for (VuforiaTrackable trackable : init.allTrackablesPS) {
                    if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                        telemetry.addData("Visible Target", trackable.getName());
                        telemetry.addLine("Mirar Power Shot");
                        targetVisiblePs = true;

                        OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                        if (robotLocationTransform != null) {
                            lastLocationPS = robotLocationTransform;
                        }
                        break;
                    }
                }
                //Parte do código que mostra a localização do robô
                if (targetVisiblePs) {

                    //Expressa a translação do robô em polegadas
                    VectorF translation = lastLocationPS.getTranslation();
                    posicaoC[1][0] = translation.get(0) / mmPerInch; //Posição X
                    posicaoC[1][1] = translation.get(1) / mmPerInch; //Posição Y
                    posicaoC[1][2] = translation.get(2) / mmPerInch; //Posição Z
                    hard.telemetry.addData("Pos (in) Power Shot", "{X, Y, Z} = %.1f, %.1f, %.1f",
                            posicaoC[2][0], posicaoC[2][1], posicaoC[2][2]);

                    //Rotação do robô em graus
                    Orientation rotationPs = Orientation.getOrientation(lastLocationPS, EXTRINSIC, XYZ, DEGREES);
                    telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotationPs.firstAngle, rotationPs.secondAngle, rotationPs.thirdAngle);

                } else {
                    telemetry.addData("Visible Target", "none");
                }
                telemetry.update();
            }
        } catch (NullPointerException e) {
                 interrupt();
        }
        }

    public void setPointGoal(String alianca) {
        if(alianca.equals("Azul")){
            //Localização do robô em relação ao setPoint do GOL azul, lembrando que é REFERÊNCIA a imagem
            //Posição referente da Aliança Vermelha
            HardwareClass.alvosGol[2].setLocation(OpenGLMatrix
                    .translation(0, -blocks - (blocks / 2), mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
            //Posição referente da Aliança Azul
            HardwareClass.alvosGol[3].setLocation(OpenGLMatrix
                    .translation(0, blocks * 4 + (blocks / 2), mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));
            //Imagem de trás da arena (observadores)
            HardwareClass.alvosGol[4].setLocation(OpenGLMatrix
                    .translation(-blocks * 3, blocks + (blocks / 2), mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

            //Posição referente da Torre Azul
            HardwareClass.alvosGol[0].setLocation(OpenGLMatrix
                    .translation(blocks * 3, blocks * 3, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
            //Posição da Torre Vermelha
            HardwareClass.alvosGol[1].setLocation(OpenGLMatrix
                    .translation(blocks * 3, 0, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 46)));

        } else {
            //Localização do robô em relação ao GOL vermelho, lembrando que é REFERÊNCIA a imagem
            //Posição referente da Aliança Vermelha
            HardwareClass.alvosGol[2].setLocation(OpenGLMatrix
                    .translation(0, -blocks * 4 - (blocks / 2), mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
            //Posição referente da Aliança Azul
            HardwareClass.alvosGol[3].setLocation(OpenGLMatrix
                    .translation(0, blocks + (blocks / 2), mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));
            //Imagem de trás da arena (observadores)
            HardwareClass.alvosGol[4].setLocation(OpenGLMatrix
                    .translation(-blocks * 3, -blocks - (blocks / 2), mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

            //Posição referente da Torre Azul
            HardwareClass.alvosGol[0].setLocation(OpenGLMatrix
                    .translation(blocks * 3, 0, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 46)));
            //Posição da Torre Vermelha
            HardwareClass.alvosGol[1].setLocation(OpenGLMatrix
                    .translation(blocks * 3, -blocks * 3, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));


        }

        // A frente do robô está virada para o eixo X (Posição Inicial)
        // Y é o lado esquerdo de acordo com a visão de X = frente.
        // Z é em cima do robô
        for (VuforiaTrackable trackable : init.allTrackablesGol) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(init.robotFromCamera, init.parameters1.cameraDirection);
        }

         setPointPS(alianca);
    }


    private void setPointPS(String alianca) {
         if(alianca.equals("Azul")) {
             //Posição referente da Aliança Vermelha
             //18.75in é a distância entre gol azul e vermelho para o primeiro Power shot
             //7.5in é a distância entre eles
             HardwareClass.alvosPS[2].setLocation(OpenGLMatrix
                     .translation(0, -blocks * 3 - 18.75f, mmTargetHeight)
                     .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
             //Posição referente da Aliança Azul
             //6 é a distância do gol pro primeiro power shot
             HardwareClass.alvosPS[3].setLocation(OpenGLMatrix
                     .translation(0, blocks * 2 + 6, mmTargetHeight)
                     .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));
             //Imagem de trás da arena (observadores)
             HardwareClass.alvosPS[4].setLocation(OpenGLMatrix
                     .translation(-blocks * 3, -18.75f, mmTargetHeight)
                     .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

             //Posição referente da Torre Azul
             HardwareClass.alvosPS[0].setLocation(OpenGLMatrix
                     .translation(blocks * 3, (blocks / 2) - 6, mmTargetHeight)
                     .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 50)));
             //Posição da Torre Vermelha
             HardwareClass.alvosPS[1].setLocation(OpenGLMatrix
                     .translation(blocks * 3, -blocks - (blocks / 2) - 18.75f, mmTargetHeight)
                     .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

         } else {
             HardwareClass.alvosPS[2].setLocation(OpenGLMatrix
                     .translation(0, -blocks * 2 - 6, mmTargetHeight)
                     .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
             //Posição referente da Aliança Azul
             HardwareClass.alvosPS[3].setLocation(OpenGLMatrix
                     .translation(0, blocks * 3 + 18.75f, mmTargetHeight)
                     .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));
             //Imagem de trás da arena (observadores)
             HardwareClass.alvosPS[4].setLocation(OpenGLMatrix
                     .translation(-blocks * 3, 18.75f, mmTargetHeight)
                     .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

             //Posição referente da Torre Azul
             HardwareClass.alvosPS[0].setLocation(OpenGLMatrix
                     .translation(blocks * 3, blocks + (blocks / 2) + 18.75f, mmTargetHeight)
                     .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
             //Posição da Torre Vermelha
             HardwareClass.alvosPS[1].setLocation(OpenGLMatrix
                     .translation(blocks * 3, -(blocks / 2) + 6, mmTargetHeight)
                     .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 50)));
         }

        for (VuforiaTrackable trackable : init.allTrackablesPS) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(init.robotFromCamera, init.parameters1.cameraDirection);
        }
        init.targetsUltimateGoal.activate();
    }
}

