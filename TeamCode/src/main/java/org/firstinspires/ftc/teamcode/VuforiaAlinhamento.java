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

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

/**
 * This 2020-2021 OpMode illustrates the basics of using the Vuforia localizer to determine
 * positioning and orientation of robot on the ULTIMATE GOAL FTC field.
 * The code is structured as a LinearOpMode
 *
 * When images are located, Vuforia is able to determine the position and orientation of the
 * image relative to the camera.  This sample code then combines that information with a
 * knowledge of where the target images are on the field, to determine the location of the camera.
 *
 * From the Audience perspective, the Red Alliance station is on the right and the
 * Blue Alliance Station is on the left.

 * There are a total of five image targets for the ULTIMATE GOAL game.
 * Three of the targets are placed in the center of the Red Alliance, Audience (Front),
 * and Blue Alliance perimeter walls.
 * Two additional targets are placed on the perimeter wall, one in front of each Tower Goal.
 * Refer to the Field Setup manual for more specific location details
 *
 * A final calculation then uses the location of the camera on the robot to determine the
 * robot's location and orientation on the field.
 *
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  ultimategoal/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 */


@TeleOp(name="Vuforia at Home", group ="Concept")
public class VuforiaAlinhamento{

    //Variáveis genéricas de instância
    TeleOperado hard = new TeleOperado();
    HardwareClass init = new HardwareClass();

    // Constantes convertidas para converter polegadas para mm
    private static final float mmPerInch = 25.4f;
    //Definida pelo centro das referências do chão
    private static final float mmTargetHeight = (6) * mmPerInch;

    //Constantes convertidas
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField = 36 * mmPerInch;
    private static final float blocks = 22.7f * mmPerInch;

    private OpenGLMatrix lastLocation = null;
    boolean targetVisible = false;

    //Posições em X, Y e Z
    double[] posicaoC = new double[3];

    public void vuforiaCalibre(double dx, double dy, double dz) {

        //Define os trackables
        init.targetsUltimateGoal = init.vuforia.loadTrackablesFromAsset("UltimateGoal");

        //Imagem da Torre Azul
        VuforiaTrackable blueTowerGoalTarget = init.targetsUltimateGoal.get(0);
        blueTowerGoalTarget.setName("Blue Tower Goal Target");

        //Imagem da Torre Vermelha
        VuforiaTrackable redTowerGoalTarget = init.targetsUltimateGoal.get(1);
        redTowerGoalTarget.setName("Red Tower Goal Target");

        //Imagem da Aliança Vermelha
        VuforiaTrackable redAllianceTarget = init.targetsUltimateGoal.get(2);
        redAllianceTarget.setName("Red Alliance Target");

        //Imagem da Aliança Azul
        VuforiaTrackable blueAllianceTarget = init.targetsUltimateGoal.get(3);
        blueAllianceTarget.setName("Blue Alliance Target");

        //Imagem de trás da arena (observadores)
        VuforiaTrackable frontWallTarget = init.targetsUltimateGoal.get(4);
        frontWallTarget.setName("Front Wall Target");

        // Para melhorar o uso dos trackables ele coloca em um array
        init.allTrackables.addAll(init.targetsUltimateGoal);

        //Localização do robô em relação ao setPoint (são 4), lembrando que é REFERÊNCIA a imagem
        redAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, -blocks - (blocks / 2), mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, 0, -90)));
        blueAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, halfField + blocks / 2, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, 0, 90)));
        frontWallTarget.setLocation(OpenGLMatrix
                .translation(-halfField + (blocks / 2), quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        // The tower goal targets are located a quarter field length from the ends of the back perimeter wall.
        blueTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField + blocks / 2, (blocks / 2) * 4, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
        redTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField + blocks / 2, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(init.CAMERA_FORWARD_DISPLACEMENT, init.CAMERA_LEFT_DISPLACEMENT, init.CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, init.phoneYRotate, init.phoneZRotate, init.phoneXRotate));


        // A frente do robô está virada para o eixo X (Posição Inicial)
        // Y é o lado esquerdo de acordo com a visão de cima.
        // Z é em cima do robô
        for (VuforiaTrackable trackable : init.allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, init.parameters1.cameraDirection);
        }
    }

    public void alinharGol(boolean a) {
        //Ativa os alvos
        init.targetsUltimateGoal.activate();

        //Verifica os targets visiveis
        targetVisible = false;
        for (VuforiaTrackable trackable : init.allTrackables) {
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                hard.telemetry.addData("Visible Target", trackable.getName());
                targetVisible = true;

                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }

        //Parte do código que provém a localização do robô
        if (targetVisible) {

            //Expressa a translação do robô em polegadas
            VectorF translation = lastLocation.getTranslation();
            posicaoC[0] = translation.get(0) / mmPerInch; //Posição X
            posicaoC[1] = translation.get(1) / mmPerInch; //Posição Y
            posicaoC[2] = translation.get(2) / mmPerInch; //Posição Z
            hard.telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    posicaoC[0], posicaoC[1], posicaoC[2]);

            //Rotação do robô em graus
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            hard.telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);

            } else {
            hard.telemetry.addData("Visible Target", "none");
        }
        hard.telemetry.update();
    }
}

