package org.whitneyrobotics.ftc.teamcode.subsys;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.whitneyrobotics.ftc.teamcode.lib.util.Toggler;

public class FoundationPuller {
    Servo leftServo;
    Servo rightServo;

    Toggler operateFoundationPullerToggler = new Toggler(2);

    public enum PullerPosition {
        UP, DOWN
    }

    //UP, DOWN
    private final double[] LEFT_PULLER_POSITIONS = {0.345, 0.795};
    private final double LEFT_PULLER_UP = LEFT_PULLER_POSITIONS[PullerPosition.UP.ordinal()];
    private final double LEFT_PULLER_DOWN = LEFT_PULLER_POSITIONS[PullerPosition.DOWN.ordinal()];

    //UP, DOWN
    private final double[] RIGHT_PULLER_POSITIONS = {0.99, 0.53};
    private final double RIGHT_PULLER_UP = RIGHT_PULLER_POSITIONS[PullerPosition.UP.ordinal()];
    private final double RIGHT_PULLER_DOWN = RIGHT_PULLER_POSITIONS[PullerPosition.DOWN.ordinal()];

    public FoundationPuller(HardwareMap pullerMap) {
        leftServo = pullerMap.servo.get("foundationPullerLeft");
        rightServo = pullerMap.servo.get("foundationPullerRight");
    }

    //For Use in Auto
    public void setFoundationPullerPosition(PullerPosition pullerPosition) {
        leftServo.setPosition(LEFT_PULLER_POSITIONS[pullerPosition.ordinal()]);
        rightServo.setPosition(RIGHT_PULLER_POSITIONS[pullerPosition.ordinal()]);
    }

    //For Use in Tele
    public void operate(boolean gamepadInput) {
        operateFoundationPullerToggler.changeState(gamepadInput);
        if (operateFoundationPullerToggler.currentState() == 0) {
            setFoundationPullerPosition(PullerPosition.UP);
        } else if (operateFoundationPullerToggler.currentState() == 1) {
            setFoundationPullerPosition(PullerPosition.DOWN);
        }
    }
}
