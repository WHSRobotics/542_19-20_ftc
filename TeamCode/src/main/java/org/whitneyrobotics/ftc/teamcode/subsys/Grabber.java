package org.whitneyrobotics.ftc.teamcode.subsys;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.whitneyrobotics.ftc.teamcode.lib.util.Toggler;

public class Grabber {
    /*
     * Elbow = Two servos in the back that swing
     * Wrist = Green Wheel
     * Hand = Yellow 3d printed spatula
     * */
    private Servo leftElbowServo;
    private Servo rightElbowServo;
    private Servo wristServo;
    private Servo handServo;

    private Toggler cyclePositionsToggler = new Toggler(5);

    public enum ElbowPosition {
        INTAKE, OUTTAKE;
    }

    public enum WristPosition {
        UP, DOWN
    }

    public enum HandPosition {
        UP, DOWN, OUTTAKE_UP, OUTTAKE_DOWN
    }

    public enum GrabberPosition {
        INTAKE_UP, INTAKE_DOWN, OUTTAKE_UP, OUTTAKE_DOWN, OUTTAKE_RELEASED
    }

    //INTAKE, OUTTAKE
    private final double[] LEFT_ELBOW_POSITION = {0.065, 0.995};
    private final double LEFT_ELBOW_INTAKE_POSITION = LEFT_ELBOW_POSITION[ElbowPosition.INTAKE.ordinal()];
    private final double LEFT_ELBOW_OUTTAKE_POSITION = LEFT_ELBOW_POSITION[ElbowPosition.OUTTAKE.ordinal()];

    //INTAKE, OUTTAKE
    private final double[] RIGHT_ELBOW_POSITIONS = {0.98, 0.065};
    private final double RIGHT_ELBOW_INTAKE_POSITION = RIGHT_ELBOW_POSITIONS[ElbowPosition.INTAKE.ordinal()];
    private final double RIGHT_ELBOW_OUTTAKE_POSITION = RIGHT_ELBOW_POSITIONS[ElbowPosition.OUTTAKE.ordinal()];

    //UP, DOWN
    private final double[] WRIST_POSITIONS = {0.195, 0.455};
    private final double WRIST_INTAKE_POSITION = WRIST_POSITIONS[WristPosition.UP.ordinal()];
    private final double WRIST_OUTTAKE_POSITION = WRIST_POSITIONS[WristPosition.DOWN.ordinal()];

    //UP, DOWN, OUTTAKE_UP, OUTTAKE_DOWN
    private final double[] HAND_POSITIONS = {0.240, 0.07, 0.38, 0.75};
    private final double HAND_UP_POSITION = HAND_POSITIONS[HandPosition.UP.ordinal()];
    private final double HAND_DOWN_POSITION = HAND_POSITIONS[HandPosition.DOWN.ordinal()];

    public Grabber(HardwareMap grabberMap) {
        leftElbowServo = grabberMap.servo.get("leftElbowServo");
        rightElbowServo = grabberMap.servo.get("rightElbowServo");
        wristServo = grabberMap.servo.get("wristServo");
        handServo = grabberMap.servo.get("handServo");
    }

    public void setElbowServoPosition(ElbowPosition elbowPosition) {
        leftElbowServo.setPosition(LEFT_ELBOW_POSITION[elbowPosition.ordinal()]);
        rightElbowServo.setPosition(RIGHT_ELBOW_POSITIONS[elbowPosition.ordinal()]);
    }

    public void setWristServoPosition(WristPosition wristPosition) {
        wristServo.setPosition(WRIST_POSITIONS[wristPosition.ordinal()]);
    }

    public void setHandServoPosition(HandPosition handPosition) {
        handServo.setPosition(HAND_POSITIONS[handPosition.ordinal()]);
    }

    public void setPosition(GrabberPosition grabberPosition) {
        if (grabberPosition == GrabberPosition.INTAKE_UP) {
            // Waiting for Stone
            setElbowServoPosition(ElbowPosition.INTAKE);
            setWristServoPosition(WristPosition.UP);
            setHandServoPosition(HandPosition.UP);
        } else if (grabberPosition == GrabberPosition.INTAKE_DOWN) {
            // Grabbing the stone
            setElbowServoPosition(ElbowPosition.INTAKE);
            setWristServoPosition(WristPosition.DOWN);
            setHandServoPosition(HandPosition.DOWN);
        } else if (grabberPosition == GrabberPosition.OUTTAKE_UP) {
            // Spinning Around
            setElbowServoPosition(ElbowPosition.OUTTAKE);
            setWristServoPosition(WristPosition.DOWN);
            setHandServoPosition(HandPosition.OUTTAKE_UP);
        } else if (grabberPosition == GrabberPosition.OUTTAKE_DOWN) {
            // Make the stone parallel
            setElbowServoPosition(ElbowPosition.OUTTAKE);
            setWristServoPosition(WristPosition.DOWN);
            setHandServoPosition(HandPosition.OUTTAKE_DOWN);
        } else if (grabberPosition == GrabberPosition.OUTTAKE_RELEASED){
            // Let go of the stone
            setElbowServoPosition(ElbowPosition.OUTTAKE);
            setWristServoPosition(WristPosition.UP);
            setHandServoPosition(HandPosition.OUTTAKE_DOWN);
        }
    }

    public void cyclePositions(boolean gamepadInputUp, boolean gamepadInputDown) {
        cyclePositionsToggler.changeState(gamepadInputUp, gamepadInputDown);
        if (cyclePositionsToggler.currentState() == 0) {
            // Waiting for Stone
            setElbowServoPosition(ElbowPosition.INTAKE);
            setWristServoPosition(WristPosition.UP);
            setHandServoPosition(HandPosition.UP);
        } else if (cyclePositionsToggler.currentState() == 1) {
            // Grabbing the stone
            setElbowServoPosition(ElbowPosition.INTAKE);
            setWristServoPosition(WristPosition.DOWN);
            setHandServoPosition(HandPosition.DOWN);
        } else if (cyclePositionsToggler.currentState() == 2) {
            // Spinning Around
            setElbowServoPosition(ElbowPosition.OUTTAKE);
            setWristServoPosition(WristPosition.DOWN);
            setHandServoPosition(HandPosition.OUTTAKE_UP);
        } else if (cyclePositionsToggler.currentState() == 3) {
            // Make the stone parallel
            setElbowServoPosition(ElbowPosition.OUTTAKE);
            setWristServoPosition(WristPosition.DOWN);
            setHandServoPosition(HandPosition.OUTTAKE_DOWN);
        } else if (cyclePositionsToggler.currentState() == 4){
            // Let go of the stone
            setElbowServoPosition(ElbowPosition.OUTTAKE);
            setWristServoPosition(WristPosition.UP);
            setHandServoPosition(HandPosition.OUTTAKE_DOWN);
        }
    }
}
