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

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

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
public class Vuforia {

    // Constantes convertidas para converter polegadas para mm
    static final float mmPerInch = 25.4f;
    //Definida pelo altura do centro das imagens do chão
    private static final float mmTargetHeight = (6) * mmPerInch;

    //Constante do tamanho de tatame da quadra quadra convertida
    private static final float blocks = 23f * mmPerInch;

    List<VuforiaTrackable> allTrackablesGol = new ArrayList<>();
    List<VuforiaTrackable> allTrackablesPS = new ArrayList<>();
    private VuforiaTrackables targetsUltimateGoal;
    private VuforiaTrackables targetsUltimatePS;
    private static final String VUFORIA_KEY =
            "AYWpo1j/////AAABmXdvyto7jU+LuXGPiPaJ7eQ4FIrujbhvZmoi " +
                    " KRcyjHFOYhPWujqUT8itJ5yl5d6xeQtRltWIaeULLDoE/zTbq+fGgveeiVmFzR45LGe6HWGjNi2twZhZqTPWFh" +
                    " 8KGHueGcpX5am/wGJGKEp25ELJ+z9laddGkm0ykwJVAJ5NP47SSdBbAb/yzDCQmAUnuNvQMgSbm8fv0wE/tukSV" +
                    " CgkhEaGuipkWgO9t6HDyh2E2UBsYeOjKwzZVsSBcn3hC2UyOimn5nkdyLqn08uu8l1eZBJWingstpU+YyRTwc0t" +
                    " VDM7mK+GnS861EiN55nBYxXM2+XH4xqtgaA+0Wpum2J04BaNtg2vgs03PIK5Gw+bmUfM  ";

    VuforiaLocalizer vuforia = null;

    // 1) Camera Source.  Valid choices are:  BACK (camêra traseira) or FRONT (camêra fronteira)
    // 2) PHONE_IS_PORTRAIT = true (portatil) or PHONE_IS_PORTRAIT = false (Deitado)
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;

    float phoneXRotate = 0;
    float phoneYRotate = 0;
    float phoneZRotate = 0;

    VuforiaLocalizer.Parameters parameters1;

    public void configureVuforia(String a, HardwareMap wMap) {
        //Vetor de String que guarda os nomes dos alvos
        String []name = new String[] {"Blue Tower Goal Target", "Red Tower Goal Target", "Red Alliance Target", "Blue Alliance Target", "Front Wall Target"};
        int cameraMonitorViewId = wMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", wMap.appContext.getPackageName());
        parameters1 = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters1.vuforiaLicenseKey = VUFORIA_KEY;

        //Direção da camêra
        parameters1.cameraDirection = CAMERA_CHOICE;

        parameters1.useExtendedTracking = false;

        //Instância o vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters1);

        VuforiaTrackable []alvosGol = new VuforiaTrackable[5];
        VuforiaTrackable []alvosPS = new VuforiaTrackable[5];

        targetsUltimateGoal = vuforia.loadTrackablesFromAsset("UltimateGoal");
        targetsUltimatePS = vuforia.loadTrackablesFromAsset("UltimateGoal");

        //Carregando trackables
        for(int i = 0; i <= 4; i++) {
            alvosGol[i] = targetsUltimateGoal.get(i);
            alvosGol[i].setName(name[i]);

            //==============================================================================

            alvosPS[i] = targetsUltimatePS.get(i);
            alvosPS[i].setName(name[i]);
        }


        // Para melhorar o uso dos trackables ele coloca em um array
        allTrackablesGol.addAll(targetsUltimateGoal);
        allTrackablesPS.addAll(targetsUltimatePS);

        if(a.equals("Azul")){
            //Localização do robô em relação ao setPoint do GOL azul, lembrando que é REFERÊNCIA a imagem
            //Posição referente da Aliança Vermelha
            alvosGol[2].setLocation(OpenGLMatrix
                    .translation(0, -blocks - (blocks / 2), mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
            //Posição referente da Aliança Azul
            alvosGol[3].setLocation(OpenGLMatrix
                    .translation(0, blocks * 4 + (blocks / 2), mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));
            //Imagem de trás da arena (observadores)
            alvosGol[4].setLocation(OpenGLMatrix
                    .translation(-blocks * 3, blocks + (blocks / 2), mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

            //Posição referente da Torre Azul
            alvosGol[0].setLocation(OpenGLMatrix
                    .translation(blocks * 3, blocks * 3, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
            //Posição da Torre Vermelha
            alvosGol[1].setLocation(OpenGLMatrix
                    .translation(blocks * 3, 0, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 46)));

        } else {
            //Localização do robô em relação ao GOL vermelho, lembrando que é REFERÊNCIA a imagem
            //Posição referente da Aliança Vermelha
            alvosGol[2].setLocation(OpenGLMatrix
                    .translation(0, -blocks * 4 - (blocks / 2), mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
            //Posição referente da Aliança Azul
            alvosGol[3].setLocation(OpenGLMatrix
                    .translation(0, blocks + (blocks / 2), mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));
            //Imagem de trás da arena (observadores)
            alvosGol[4].setLocation(OpenGLMatrix
                    .translation(-blocks * 3, -blocks - (blocks / 2), mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

            //Posição referente da Torre Azul
            alvosGol[0].setLocation(OpenGLMatrix
                    .translation(blocks * 3, 0, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 46)));
            //Posição da Torre Vermelha
            alvosGol[1].setLocation(OpenGLMatrix
                    .translation(blocks * 3, -blocks * 3, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));


        }

         if(a.equals("Azul")) {
             //Posição referente da Aliança Vermelha
             //18.75in é a distância entre gol azul e vermelho para o primeiro Power shot
             //7.5in é a distância entre eles
             alvosPS[2].setLocation(OpenGLMatrix
                     .translation(0, -blocks * 3 - 18.75f, mmTargetHeight)
                     .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
             //Posição referente da Aliança Azul
             //6 é a distância do gol pro primeiro power shot
             alvosPS[3].setLocation(OpenGLMatrix
                     .translation(0, blocks * 2 + 6, mmTargetHeight)
                     .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));
             //Imagem de trás da arena (observadores)
             alvosPS[4].setLocation(OpenGLMatrix
                     .translation(-blocks * 3, -18.75f, mmTargetHeight)
                     .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

             //Posição referente da Torre Azul
             alvosPS[0].setLocation(OpenGLMatrix
                     .translation(blocks * 3, (blocks / 2) - 6, mmTargetHeight)
                     .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 50)));
             //Posição da Torre Vermelha
             alvosPS[1].setLocation(OpenGLMatrix
                     .translation(blocks * 3, -blocks - (blocks / 2) - 18.75f, mmTargetHeight)
                     .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

         } else {
             alvosPS[2].setLocation(OpenGLMatrix
                     .translation(0, -blocks * 2 - 6, mmTargetHeight)
                     .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));
             //Posição referente da Aliança Azul
             alvosPS[3].setLocation(OpenGLMatrix
                     .translation(0, blocks * 3 + 18.75f, mmTargetHeight)
                     .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));
             //Imagem de trás da arena (observadores)
             alvosPS[4].setLocation(OpenGLMatrix
                     .translation(-blocks * 3, 18.75f, mmTargetHeight)
                     .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

             //Posição referente da Torre Azul
             alvosPS[0].setLocation(OpenGLMatrix
                     .translation(blocks * 3, blocks + (blocks / 2) + 18.75f, mmTargetHeight)
                     .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
             //Posição da Torre Vermelha
             alvosPS[1].setLocation(OpenGLMatrix
                     .translation(blocks * 3, -(blocks / 2) + 6, mmTargetHeight)
                     .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 50)));
         }

         //Corige a rotação da camêra dependendo da configuração
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        //Se o phone estiver portátil muda configuração para X
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        // A frente do robô está virada para o eixo X (Posição Inicial)
        // Y é o lado esquerdo de acordo com a visão de X = frente.
        // Z é em cima do robô
        for (VuforiaTrackable trackable : allTrackablesGol) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters1.cameraDirection);
        }
        for (VuforiaTrackable trackable : allTrackablesPS) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters1.cameraDirection);
        }
    }
    public void ativeVuforia() {
        targetsUltimateGoal.activate();
        targetsUltimatePS.activate();
    }
}

