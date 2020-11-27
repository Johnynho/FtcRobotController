/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

public class HardwareClass {
    //Declaração dos motores
    DcMotor motorEsquerda, motorDireita = null;
    DcMotor motorEsquerdaTras = null;
    DcMotor motorDireitaTras = null;

    static BNO055IMU imu;

    HardwareMap hwMap   =  null;

    List<VuforiaTrackable> allTrackablesGol = new ArrayList<>();
    List<VuforiaTrackable> allTrackablesPS = new ArrayList<>();
    VuforiaTrackables targetsUltimateGoal;

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

    float CAMERA_FORWARD_DISPLACEMENT;
    float CAMERA_VERTICAL_DISPLACEMENT;
    float CAMERA_LEFT_DISPLACEMENT;

    VuforiaLocalizer.Parameters parameters1;

    static VuforiaTrackable []alvosGol = new VuforiaTrackable[5];
    static VuforiaTrackable []alvosPS = new VuforiaTrackable[5];

    OpenGLMatrix robotFromCamera;

    private static final float mmPerInch = 25.4f;

    public void hardwareGeral(HardwareMap ahwMap) {
        //Referência hardware
        hwMap = ahwMap;

        /*
         * =============================================================================
         *                                  GYRO
         * =============================================================================
         */

       //Configura o gyro
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";

        imu = hwMap.get(BNO055IMU.class, "imu");

        //Inicializa os parametros do gyro
        imu.initialize(parameters);

        /*
         * =============================================================================
         *                                  ACIONADORES
         * =============================================================================
         */

        //Pega o nome das variáveis no Dv.
        motorEsquerda = hwMap.get(DcMotor.class, "motor_Esquerda");
        motorEsquerdaTras = hwMap.get(DcMotor.class, "motor_EsquerdaTras");
        motorDireita = hwMap.get(DcMotor.class,"motor_Direita");
        motorDireitaTras = hwMap.get(DcMotor.class,"motor_DireitaTras");

        //Direção dos motores
        motorEsquerda.setDirection(DcMotor.Direction.FORWARD);
        motorDireita.setDirection(DcMotor.Direction.REVERSE);
        motorEsquerdaTras.setDirection(DcMotor.Direction.FORWARD);
        motorDireitaTras.setDirection(DcMotor.Direction.REVERSE);

        /*
         * =============================================================================
         *                                  VUFORIA
         * =============================================================================
         */

       int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        parameters1 = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters1.vuforiaLicenseKey = VUFORIA_KEY;

        //Direção da camêra
        parameters1.cameraDirection = CAMERA_CHOICE;

        parameters1.useExtendedTracking = false;

        //Instância o vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters1);

        targetsUltimateGoal = vuforia.loadTrackablesFromAsset("UltimateGoal");

        //Imagem da Torre Azul
        alvosGol[0] = targetsUltimateGoal.get(0);
        alvosGol[0].setName("Blue Tower Goal Target");

        //Imagem da Torre Vermelha
        alvosGol[1] = targetsUltimateGoal.get(1);
        alvosGol[1].setName("Red Tower Goal Target");

        //Imagem da Aliança Vermelha
        alvosGol[2] = targetsUltimateGoal.get(2);
        alvosGol[2].setName("Red Alliance Target");

        //Imagem da Aliança Azul
        alvosGol[3] = targetsUltimateGoal.get(3);
        alvosGol[3] .setName("Blue Alliance Target");

        //Imagem de trás da arena (observadores)
        alvosGol[4] = targetsUltimateGoal.get(4);
        alvosGol[4].setName("Front Wall Target");

        //==============================================================================

        alvosPS[0] = targetsUltimateGoal.get(0);
        alvosPS[0].setName("Blue Tower Power Shot Target");

        //Imagem da Torre Vermelha
        alvosPS[1] = targetsUltimateGoal.get(1);
        alvosPS[1].setName("Red Tower Power Shot Target");

        //Imagem da Aliança Vermelha
        alvosPS[2] = targetsUltimateGoal.get(2);
        alvosPS[2].setName("Red Alliance Power Shot Target");

        //Imagem da Aliança Azul
        alvosPS[3] = targetsUltimateGoal.get(3);
        alvosPS[3] .setName("Blue Alliance Power Shot Target");

        //Imagem de trás da arena (observadores)
        alvosPS[4] = targetsUltimateGoal.get(4);
        alvosPS[4].setName("Front Wall Power Shot Target");

        // Para melhorar o uso dos trackables ele coloca em um array
        allTrackablesGol.addAll(targetsUltimateGoal);
        allTrackablesPS.addAll(targetsUltimateGoal);

        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90;
        }

        CAMERA_FORWARD_DISPLACEMENT = 5.0f * mmPerInch;   //De acordo com o centro do robô
        CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   //Altura da camêra (referente ao chão)
        CAMERA_LEFT_DISPLACEMENT = 0;     //Deslocamento da camêra

        robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));


    }

 }

