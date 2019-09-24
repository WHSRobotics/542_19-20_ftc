package org.whitneyrobotics.ftc.teamcode.subsys;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class FoundationPuller {
    Servo leftServo;
    Servo rightServo;

    public enum PullerPosition{
        UP, DOWN
    }
    //UP,DOWN
    private final double[]LEFT_PULLER_POSITIONS = {0.2,0.7};
    private final double LEFT_PULLER_UP = LEFT_PULLER_POSITIONS[PullerPosition.UP.ordinal()];
    private final double LEFT_PULLER_DOWN = LEFT_PULLER_POSITIONS[PullerPosition.DOWN.ordinal()];

    private final double[] RIGHT_PULLER_POSITIONS = {0.2,0.7};
    private final double RIGHT_PULLER_UP = RIGHT_PULLER_POSITIONS[PullerPosition.UP.ordinal()];
    private final double RIGHT_PULLER_DOWN = RIGHT_PULLER_POSITIONS[PullerPosition.DOWN.ordinal()];

    public FoundationPuller(HardwareMap pullerMap){
        leftServo = pullerMap.servo.get("leftPullerServo");
        rightServo = pullerMap.servo.get("rightPullerServo");
    }

    public void setPosition(PullerPosition pullerPosition){
        leftServo.setPosition(LEFT_PULLER_POSITIONS[pullerPosition.ordinal()]);
        rightServo.setPosition(RIGHT_PULLER_POSITIONS[pullerPosition.ordinal()]);
    }
}
