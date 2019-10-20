package org.whitneyrobotics.ftc.teamcode.subsys;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.whitneyrobotics.ftc.teamcode.lib.util.Toggler;

public class Intake {
    private DcMotor leftIntake;
    private DcMotor rightIntake;

    public static final double INTAKE_POWER = 0.8;

    private Toggler intakeToggler = new Toggler(2);
    public Intake(HardwareMap intakeMap){
        leftIntake = intakeMap.dcMotor.get("leftIntake");
        rightIntake = intakeMap.dcMotor.get("rightIntake");
        leftIntake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void operateIntake(boolean gamepadInput1, boolean gamepadInput2){
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

    public void setIntakeMotorPowers(double power){
        leftIntake.setPower(power);
        rightIntake.setPower(power);
    }
}
