package org.whitneyrobotics.ftc.teamcode.subsys;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.whitneyrobotics.ftc.teamcode.lib.util.Toggler;

public class Extension {
    private DcMotor leftExtension;
    private DcMotor rightExtension;
    private Servo leftElbowServo;
    private Servo rightElbowServo;
    private Servo wristServo;
    private Servo handServo;

    private Toggler shouldExtendToggler = new Toggler(2);

    private int[] extensionMotorPositions = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

    public enum ElbowPosition {
        INTAKE, OUTTAKE
    }

    public enum WristPosition {
        INTAKE, OUTTAKE
    }

    public enum HandPosition {
        UP, DOWN, OUTTAKE
    }
    private Toggler operateExtensionToggler;
    private Toggler extensionTog = new Toggler (5);

    //INTAKE, OUTTAKE
    private final double[] LEFT_ELBOW_POSITION = {0.130, 0.950};
    private final double LEFT_ELBOW_INTAKE_POSITION = LEFT_ELBOW_POSITION[ElbowPosition.INTAKE.ordinal()];
    private final double LEFT_ELBOW_OUTTAKE_POSITION = LEFT_ELBOW_POSITION[ElbowPosition.OUTTAKE.ordinal()];

    //INTAKE, OUTTAKE
    private final double[] RIGHT_ELBOW_POSITIONS = {0.935, 0.115};
    private final double RIGHT_ELBOW_INTAKE_POSITION = RIGHT_ELBOW_POSITIONS[ElbowPosition.INTAKE.ordinal()];
    private final double RIGHT_ELBOW_OUTTAKE_POSITION = RIGHT_ELBOW_POSITIONS[ElbowPosition.OUTTAKE.ordinal()];

    //INTAKE, OUTTAKE
    private final double[] WRIST_POSITIONS = {0.120, 0.370};
    private final double WRIST_INTAKE_POSITION = WRIST_POSITIONS[WristPosition.INTAKE.ordinal()];
    private final double WRIST_OUTTAKE_POSITION = WRIST_POSITIONS[WristPosition.OUTTAKE.ordinal()];

    //UP, DOWN, OUTTAKE
    private final double[] HAND_POSITIONS = {0.240, 0.135, 0.995};
    private final double HAND_UP_POSITION = HAND_POSITIONS[HandPosition.UP.ordinal()];
    private final double HAND_DOWN_POSITION = HAND_POSITIONS[HandPosition.DOWN.ordinal()];

    /*Linear Slides Part of Extension*/
    public Extension(HardwareMap extensionMap){
        operateExtensionToggler = new Toggler(11);
        leftExtension = extensionMap.dcMotor.get("leftExtension");
        rightExtension = extensionMap.dcMotor.get("rightExtension");
        leftElbowServo = extensionMap.servo.get("leftElbowServo");
        rightElbowServo = extensionMap.servo.get("rightElbowServo");
        wristServo = extensionMap.servo.get("wristServo");
        handServo = extensionMap.servo.get("handServo");
    }

    public void operateExtension(boolean gamepadInputUp, boolean gamepadInputDown, boolean gamepadInputGo) {
        operateExtensionToggler.changeState(gamepadInputUp, gamepadInputDown);
        shouldExtendToggler.changeState(gamepadInputGo);

        if (shouldExtendToggler.currentState() == 0) {
            setExtensionPosition(operateExtensionToggler.currentState());
        }else if (shouldExtendToggler.currentState() ==1){
            setExtensionPosition(0);
        }
    }

    public void setExtensionPosition (int extensionPosition){
        operateExtensionToggler.setState(extensionPosition);
        leftExtension.setTargetPosition(extensionMotorPositions[operateExtensionToggler.currentState()]);
        rightExtension.setTargetPosition(extensionMotorPositions[operateExtensionToggler.currentState()]);
    }

    /*Grabber Part of Extension*/
    public void setElbowServoPosition(ElbowPosition elbowPosition){
        leftElbowServo.setPosition(LEFT_ELBOW_POSITION[elbowPosition.ordinal()]);
        rightElbowServo.setPosition(RIGHT_ELBOW_POSITIONS[elbowPosition.ordinal()]);
    }

    public void setWristServoPosition(WristPosition wristPosition){
        wristServo.setPosition(WRIST_POSITIONS[wristPosition.ordinal()]);
    }

    public void setHandServoPosition(HandPosition handPosition) {
        handServo.setPosition(HAND_POSITIONS[handPosition.ordinal()]);
    }

    public void operateOuttake (boolean gamepadInput){
        extensionTog.changeState(gamepadInput);
        if (extensionTog.currentState() == 0){
            setHandServoPosition(Extension.HandPosition.UP);
            setWristServoPosition(Extension.WristPosition.INTAKE);
            setElbowServoPosition(Extension.ElbowPosition.INTAKE);
        }else if(extensionTog.currentState() == 1){
            setHandServoPosition(Extension.HandPosition.DOWN);
            setWristServoPosition(Extension.WristPosition.OUTTAKE);
            setElbowServoPosition(Extension.ElbowPosition.INTAKE);
        }else if (extensionTog.currentState() == 2){
            setHandServoPosition(Extension.HandPosition.DOWN);
            setWristServoPosition(Extension.WristPosition.OUTTAKE);
            setElbowServoPosition(Extension.ElbowPosition.OUTTAKE);
        }else if (extensionTog.currentState() == 3){
            setHandServoPosition(Extension.HandPosition.OUTTAKE);
            setElbowServoPosition(Extension.ElbowPosition.OUTTAKE);
        }else{
            setWristServoPosition(Extension.WristPosition.INTAKE);
        }
    }
}
