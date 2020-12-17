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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;

public class HardwareClass {
    //Declaração dos motores
    public DcMotor motorEsquerda, motorDireita = null;
    public DcMotor motorEsquerdaTras = null;
    public DcMotor motorDireitaTras = null;
    public DcMotor motorIntake = null;
    public Servo servoChapa = null;
    public Servo servoPivo = null;
    public LED ledShooter = null;

    //Referências de hardware e gyro
    static BNO055IMU imu;
    HardwareMap hwMap   =  null;

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

        //Imu no Drive Station
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
        motorIntake = hwMap.get(DcMotor.class,"motor_Intake");
        servoPivo = hwMap.get(Servo.class,"servo_Pivo");
        servoChapa = hwMap.get(Servo.class,"servo_Chapa");
        ledShooter = hwMap.get(LED.class,"led_Shooter");

        //Direção dos motores
        motorEsquerda.setDirection(DcMotor.Direction.FORWARD);
        motorDireita.setDirection(DcMotor.Direction.REVERSE);
        motorEsquerdaTras.setDirection(DcMotor.Direction.FORWARD);
        motorDireitaTras.setDirection(DcMotor.Direction.REVERSE);
        motorIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        servoPivo.setDirection(Servo.Direction.FORWARD);
        servoChapa.setDirection(Servo.Direction.FORWARD);

        //Reseta os encoder, por que são usados em dois OpModes
        motorEsquerda.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDireita.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorEsquerdaTras.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDireitaTras.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        //Coloca para se mexer contando o encoder
        motorEsquerda.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorDireita.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorEsquerdaTras.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorDireitaTras.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

 }

