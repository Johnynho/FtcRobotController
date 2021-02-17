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
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
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
 * Você está olhando da aliança vermeleha (perspectiva);
 * - O eixo X é da sua esquerda para direita. (positivo do centro para direita)
 * - O eixo Y é da aliança vermelha para o outro lado do campo onde está a aliança azul
 * (Positive no centro, em direção a aliança azul)
 * - O eixo Z vai do chão para cima (Positivo é acima do chão)
**/
public class Vuforia extends TeleOperado{

    // Constantes convertidas para converter polegadas para mm
    static final float mmPerInch = 25.4f;
    //Definida pelo altura do centro das imagens do chão
    private static final float mmTargetHeight = (6) * mmPerInch;

    //Constante do tamanho de tatame da quadra quadra convertida
    private static final float blocks = 23f * mmPerInch;

    List<VuforiaTrackable> allTrackables = new ArrayList<>();

    private VuforiaTrackables targetsUltimateGoal;

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

    OpenGLMatrix lastLocation;

    //Posições em X, Y e Z
    double[] posicaoRobot = new double[3];

    boolean targetVisible;

    public void configureVuforia(String a, HardwareMap wMap) {
        //Vetor de String que guarda os nomes dos alvos
        String []name = new String[] {"Blue Tower Goal Target", "Red Tower Goal Target"};
        int cameraMonitorViewId = wMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", wMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        //Direção da camêra
        parameters.cameraDirection = CAMERA_CHOICE;

        parameters.useExtendedTracking = false;

        //Instância o vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        VuforiaTrackable []alvos = new VuforiaTrackable[2];

        targetsUltimateGoal = vuforia.loadTrackablesFromAsset("UltimateGoal");

        //Carregando trackables
        for(int i = 0; i <= 1; i++) {
            alvos[i] = targetsUltimateGoal.get(i);
            alvos[i].setName(name[i]);
        }

        // Para melhorar o uso dos trackables ele coloca em um array
        allTrackables.addAll(targetsUltimateGoal);

         /*
          * Posição do setPoint em relação a imagem
          * Importante: Se jogarmos na aliança azul temos que mirar no gol vermelho e vice versa
          * 7.5 é a distância entre cada power shot
          */
        if(a.equals("Azul")){ //Se estivermos jogando na aliança azul temos que ajustar os trackable
            //Posição da Torre Vermelha
            alvos[1].setLocation(OpenGLMatrix
                    .translation(blocks * 3, 0, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

            //Posição referente da Torre Azul (Power shot)
            alvos[0].setLocation(OpenGLMatrix
                    .translation(blocks, (blocks / 2) + 6, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        } else { //Se não estivermos jogando na aliança azul, estaremos na vermelha
            //Posição referente da Torre Azul
            alvos[0].setLocation(OpenGLMatrix
                    .translation(blocks * 3, 0, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
            //Posição da Torre Vermelha (power shot)
            alvos[1].setLocation(OpenGLMatrix
                    .translation(blocks * 3, (-blocks / 2) - 6, mmTargetHeight)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
        }


         //Corrige a rotação da camêra dependendo da configuração
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        //Se o phone estiver portátil muda configuração para X
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }

        //Indicar em que localização do robô a camêra está
        final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        // A frente do robô está virada para o eixo X (Posição Inicial)
        // Y é o lado esquerdo de acordo com a visão de X = frente.
        // Z é em cima do robô
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }
    }

    public void ativeVuforia() {
        targetsUltimateGoal.activate();
    }

    public void deactivateVuforia(){
        targetsUltimateGoal.deactivate();
    }

    //Verifica os targets visiveis
    public void vuforiaPosi() {

        targetVisible = false;

        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                telemetry.addData("Visible Target", trackable.getName());
                targetVisible = true;

                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }
        //Parte do código que mostra a localização do robô
        if (targetVisible) {
            //Expressa a translação do robô em polegadas
            VectorF translation = lastLocation.getTranslation();
            posicaoRobot[0] = translation.get(0) / Vuforia.mmPerInch; //Posição X
            posicaoRobot[1] = translation.get(1) / Vuforia.mmPerInch; //Posição Y
            posicaoRobot[2] = translation.get(2) / Vuforia.mmPerInch; //Posição Z
            telemetry.addData("Posição robô (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    posicaoRobot[0], posicaoRobot[1], posicaoRobot[2]);

        } else {
            telemetry.addData("Visible Target", "none");
        }
    }

    public boolean visible() {
        return targetVisible;
    }
}

