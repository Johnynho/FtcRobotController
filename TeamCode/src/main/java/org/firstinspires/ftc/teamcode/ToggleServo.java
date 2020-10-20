package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

public class ToggleServo {
    Servo servo1 = null;
    boolean buttonA = gamepad1.a;
    public double servo(double a) {
    if(buttonA){
        servo1.setPosition(1);
    a++;
    return a;
    }else if(a==1 && buttonA){
    servo1.setPosition(0);
    }
    return a;
    }
    public boolean sif(boolean b){
        if(b){
            return b;
        }
        return b;

    }
}