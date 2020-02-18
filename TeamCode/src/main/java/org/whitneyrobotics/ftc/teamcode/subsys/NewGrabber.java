package org.whitneyrobotics.ftc.teamcode.subsys;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.whitneyrobotics.ftc.teamcode.lib.util.Toggler;

public class NewGrabber {
    private Servo leftElbowServo;
    private Servo rightElbowServo;
    private Servo wristServo;
    private Servo handServo;

    private Toggler cyclePositionsToggler = new Toggler(5);
    private Toggler capstoneToggler = new Toggler(2);

    public enum ElbowPosition {
        INTAKE, OUTTAKE, HOVER
    }

    public enum WristPosition {
        UP, DOWN
    }

    public enum HandPosition {
        UP, DOWN, OUTTAKE_DOWN
    }

    public enum GrabberPosition {
        INTAKE_UP, INTAKE_DOWN, INTAKE_DOWN_RELEASED, OUTTAKE_DOWN, OUTTAKE_RELEASED
    }

    //INTAKE, OUTTAKE, HOVER
    private final double[] LEFT_ELBOW_POSITION = {0.035, 0.965, 0.44};
    private final double LEFT_ELBOW_INTAKE_POSITION = LEFT_ELBOW_POSITION[NewGrabber.ElbowPosition.INTAKE.ordinal()];
    private final double LEFT_ELBOW_OUTTAKE_POSITION = LEFT_ELBOW_POSITION[NewGrabber.ElbowPosition.OUTTAKE.ordinal()];

    //INTAKE, OUTTAKE, HOVER
    private final double[] RIGHT_ELBOW_POSITIONS = {0.965, 0.035, 0.64};
    private final double RIGHT_ELBOW_INTAKE_POSITION = RIGHT_ELBOW_POSITIONS[NewGrabber.ElbowPosition.INTAKE.ordinal()];
    private final double RIGHT_ELBOW_OUTTAKE_POSITION = RIGHT_ELBOW_POSITIONS[NewGrabber.ElbowPosition.OUTTAKE.ordinal()];

    //UP, DOWN
    private final double[] WRIST_POSITIONS = {.08, 0.2};
    private final double WRIST_INTAKE_POSITION = WRIST_POSITIONS[NewGrabber.WristPosition.UP.ordinal()];
    private final double WRIST_OUTTAKE_POSITION = WRIST_POSITIONS[NewGrabber.WristPosition.DOWN.ordinal()];

    //UP, DOWN, OUTTAKE_UP, OUTTAKE_DOWN
    private final double[] HAND_POSITIONS = {.98, .85, 0.15};//{0.240, 0.07, 0.38, 0.75};
    private final double HAND_UP_POSITION = HAND_POSITIONS[NewGrabber.HandPosition.UP.ordinal()];
    private final double HAND_DOWN_POSITION = HAND_POSITIONS[NewGrabber.HandPosition.DOWN.ordinal()];

    public NewGrabber(HardwareMap grabberMap) {
        leftElbowServo = grabberMap.servo.get("leftElbowServo");
        rightElbowServo = grabberMap.servo.get("rightElbowServo");
        wristServo = grabberMap.servo.get("wristServo");
        handServo = grabberMap.servo.get("handServo");
    }

    public void setElbowServoPosition(NewGrabber.ElbowPosition elbowPosition) {
        leftElbowServo.setPosition(LEFT_ELBOW_POSITION[elbowPosition.ordinal()]);
        rightElbowServo.setPosition(RIGHT_ELBOW_POSITIONS[elbowPosition.ordinal()]);
    }

    public void setWristServoPosition(NewGrabber.WristPosition wristPosition) {
        wristServo.setPosition(WRIST_POSITIONS[wristPosition.ordinal()]);
    }

    public void setHandServoPosition(NewGrabber.HandPosition handPosition) {
        handServo.setPosition(HAND_POSITIONS[handPosition.ordinal()]);
    }

    public void setPosition(NewGrabber.GrabberPosition grabberPosition) {
        if (grabberPosition == NewGrabber.GrabberPosition.INTAKE_UP) {
            // Waiting for Stone
            setElbowServoPosition(NewGrabber.ElbowPosition.HOVER);
            setWristServoPosition(NewGrabber.WristPosition.UP);
            setHandServoPosition(HandPosition.DOWN);
        } else if (grabberPosition == GrabberPosition.INTAKE_DOWN_RELEASED) {
            // Right before grabbing the stone
            setElbowServoPosition(NewGrabber.ElbowPosition.INTAKE);
            setWristServoPosition(NewGrabber.WristPosition.UP);
            setHandServoPosition(HandPosition.DOWN);
        } else if (grabberPosition == NewGrabber.GrabberPosition.INTAKE_DOWN) {
            // Grabbing the stone
            setElbowServoPosition(NewGrabber.ElbowPosition.INTAKE);
            setWristServoPosition(NewGrabber.WristPosition.DOWN);
            setHandServoPosition(NewGrabber.HandPosition.DOWN);
        } else if (grabberPosition == NewGrabber.GrabberPosition.OUTTAKE_DOWN) {
            // Make the stone parallel
            setElbowServoPosition(NewGrabber.ElbowPosition.OUTTAKE);
            setWristServoPosition(NewGrabber.WristPosition.DOWN);
            setHandServoPosition(NewGrabber.HandPosition.OUTTAKE_DOWN);
        } else if (grabberPosition == NewGrabber.GrabberPosition.OUTTAKE_RELEASED) {
            // Let go of the stone
            setElbowServoPosition(NewGrabber.ElbowPosition.OUTTAKE);
            setWristServoPosition(NewGrabber.WristPosition.UP);
            setHandServoPosition(NewGrabber.HandPosition.OUTTAKE_DOWN);
        }
    }

    public void cyclePositions(boolean gamepadInputUp, boolean gamepadInputDown, boolean gamepadInputCapstone) {
        capstoneToggler.changeState(gamepadInputCapstone);
        cyclePositionsToggler.changeState(gamepadInputUp, gamepadInputDown);
        if (cyclePositionsToggler.currentState() == 0) {
            // Waiting for Stone
            setElbowServoPosition(ElbowPosition.HOVER);
            setWristServoPosition(NewGrabber.WristPosition.UP);
            setHandServoPosition(NewGrabber.HandPosition.DOWN);
        } else if (cyclePositionsToggler.currentState() == 1) {
            // Grabbing the stone
            setElbowServoPosition(NewGrabber.ElbowPosition.INTAKE);
            setWristServoPosition(NewGrabber.WristPosition.UP);
            setHandServoPosition(NewGrabber.HandPosition.DOWN);
        } else if (cyclePositionsToggler.currentState() == 2) {
            // Spinning Around
            setElbowServoPosition(NewGrabber.ElbowPosition.INTAKE);
            setWristServoPosition(NewGrabber.WristPosition.DOWN);
            setHandServoPosition(NewGrabber.HandPosition.DOWN);
        } else if (cyclePositionsToggler.currentState() == 3){
            // Let go of the stone
            setElbowServoPosition(NewGrabber.ElbowPosition.OUTTAKE);
            setWristServoPosition(NewGrabber.WristPosition.DOWN);
            setHandServoPosition(NewGrabber.HandPosition.OUTTAKE_DOWN);
        } else if (cyclePositionsToggler.currentState() == 4) {
            setElbowServoPosition(NewGrabber.ElbowPosition.OUTTAKE);
            setWristServoPosition(NewGrabber.WristPosition.UP);
            setHandServoPosition(NewGrabber.HandPosition.OUTTAKE_DOWN);
        }
    }
}
