package org.whitneyrobotics.ftc.teamcode.subsys;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Hardware;

public class BackGate {
    Servo backGateServo;

    enum BackGateServoPosition{
        UP, DOWN;
    }
    //UP, DOWN
    private double[] backGateServoPositions = {0.725, 0.125};

    public BackGate(HardwareMap backGateMap){
        backGateServo = backGateMap.servo.get("backGateServo");
    }

    public void setPosition(BackGateServoPosition backGateServoPosition){
        backGateServo.setPosition(backGateServoPositions[backGateServoPosition.ordinal()]);
    }
    public void operate(boolean gamepadInput){
        if (gamepadInput){
            setPosition(BackGateServoPosition.DOWN);
        }else{
            setPosition(BackGateServoPosition.UP);
        }
    }
}
