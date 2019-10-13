package org.whitneyrobotics.ftc.teamcode.subsys;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.whitneyrobotics.ftc.teamcode.lib.util.Toggler;

public class Extension {
    private DcMotor leftExtension;
    private DcMotor rightExtension;
    private Servo rightPivotServo;
    private Servo leftPivotServo;
    private Servo switchServo;
    private Servo clampServo;

    private Toggler shouldExtendToggler = new Toggler(2);

    private int[] extensionMotorPositions = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

    public enum PivotPosition{
        INTAKE, OUTTAKE
    }

    public enum SwitchPosition{
        INTAKE, OUTTAKE
    }

    public enum ClampPosition{
        UP, DOWN
    }
    private Toggler operateExtensionToggler;

    //UP, DOWN
    private final double[] CLAMP_POSITIONS = {0.0, 0.0, 0.0};
    private final double CLAMP_UP_POSITION = CLAMP_POSITIONS[ClampPosition.UP.ordinal()];
    private final double CLAMP_DOWN_POSITION = CLAMP_POSITIONS[ClampPosition.DOWN.ordinal()];


    //INTAKE, OUTTAKE, REST
    private final double[] RIGHT_PIVOT_POSITIONS = {0.0, 0.0, 0.0};
    private final double RIGHT_INTAKE_POSITION = RIGHT_PIVOT_POSITIONS[PivotPosition.INTAKE.ordinal()];
    private final double RIGHT_OUTTAKE_POSITION = RIGHT_PIVOT_POSITIONS[PivotPosition.OUTTAKE.ordinal()];

    //INTAKE, OUTTAKE, REST
    private final double[] LEFT_PIVOT_POSITION = {0.0, 0.0, 0.0};
    private final double LEFT_INTAKE_POSITION = LEFT_PIVOT_POSITION[PivotPosition.INTAKE.ordinal()];
    private final double LEFT_OUTTAKE_POSITION = LEFT_PIVOT_POSITION[PivotPosition.OUTTAKE.ordinal()];


    //INTAKE, OUTTAKE, REST
    private final double[] SWITCH_POSITIONS = {0.0, 0.0, 0.0};
    private final double SWITCH_INTAKE_POSITION = SWITCH_POSITIONS[SwitchPosition.INTAKE.ordinal()];
    private final double SWITCH_OUTTAKE_POSITION = SWITCH_POSITIONS[SwitchPosition.OUTTAKE.ordinal()];

    /*Linear Slides Part of Extension*/
    public Extension(HardwareMap extensionMap){
        operateExtensionToggler = new Toggler(11);
        /*leftExtension = extensionMap.dcMotor.get("leftExtension");
        rightExtension = extensionMap.dcMotor.get("rightExtension");*/
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
    public void setPivotServoPosition (PivotPosition pivotPosition){
        rightPivotServo.setPosition(RIGHT_PIVOT_POSITIONS[pivotPosition.ordinal()]);
        leftPivotServo.setPosition(LEFT_PIVOT_POSITION[pivotPosition.ordinal()]);
    }

    public void setSwitchServoPosition (SwitchPosition switchPosition){
        switchServo.setPosition(SWITCH_POSITIONS[switchPosition.ordinal()]);
    }

    public void setClampServoPosition (ClampPosition clampPosition) {
        clampServo.setPosition(CLAMP_POSITIONS[clampPosition.ordinal()]);
    }
}
