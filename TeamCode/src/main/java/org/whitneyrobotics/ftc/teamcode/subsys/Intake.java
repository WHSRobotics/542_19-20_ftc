package org.whitneyrobotics.ftc.teamcode.subsys;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.whitneyrobotics.ftc.teamcode.lib.util.Toggler;

public class Intake {

    private DcMotor leftIntake;
    private DcMotor rightIntake;
    private Servo intakePusher;

    public enum IntakePusherPosition {
        DOWN, UP
    }

    // DOWN, UP
    private final double[] INTAKE_PUSHER_POSITIONS = {0, 0};
    private final double INTAKE_PUSHER_DOWN = INTAKE_PUSHER_POSITIONS[IntakePusherPosition.DOWN.ordinal()];
    private final double INTAKE_PUSHER_UP = INTAKE_PUSHER_POSITIONS[IntakePusherPosition.UP.ordinal()];
    private final double INTAKE_POWER=0.7;
    public Toggler intakeSpeedToggler = new Toggler (20);
    private Toggler intakeToggler = new Toggler(2);
    public Intake(HardwareMap intakeMap){
        leftIntake = intakeMap.dcMotor.get("leftIntake");
        rightIntake = intakeMap.dcMotor.get("rightIntake");
        intakePusher = intakeMap.servo.get("intakePusher");

        leftIntake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void operateIntake(boolean gamepadInput1, boolean gamepadInput2, boolean up, boolean down){
        intakeSpeedToggler.changeState(up,down);
        //double INTAKE_POWER = intakeSpeedToggler.currentState()/20.0;
        intakeToggler.changeState(gamepadInput1);
        if (gamepadInput2) {
            leftIntake.setPower(-INTAKE_POWER);
            rightIntake.setPower(-INTAKE_POWER);
        }
        else if (intakeToggler.currentState() == 1) {
            leftIntake.setPower(INTAKE_POWER);
            rightIntake.setPower(INTAKE_POWER);
        }
        else {
            leftIntake.setPower(0.0);
            rightIntake.setPower(0.0);
        }
    }

    public void setMotorPowers(double power){
        leftIntake.setPower(power);
        rightIntake.setPower(power);
    }

    public void setIntakePusherPosition(IntakePusherPosition intakePusherPosition) {
        intakePusher.setPosition(INTAKE_PUSHER_POSITIONS[intakePusherPosition.ordinal()]);
    }
}
