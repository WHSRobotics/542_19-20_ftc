package org.whitneyrobotics.ftc.teamcode.subsys;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.whitneyrobotics.ftc.teamcode.lib.util.Toggler;

public class Intake {
    private DcMotor lefIntake;
    private DcMotor rightIntake;

    public static final double INTAKE_POWER = 0.8;

    private Toggler intakeToggler = new Toggler(2);
    public Intake(HardwareMap intakeMap){
        //lefIntake = intakeMap.dcMotor.get("leftIntake");
       // rightIntake = intakeMap.dcMotor.get("rightIntake");
        rightIntake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void operateIntake(boolean gamepadInput1, boolean gamepadInput2){
        intakeToggler.changeState(gamepadInput1);
        if (gamepadInput2) {
            lefIntake.setPower(-INTAKE_POWER);
            rightIntake.setPower(-INTAKE_POWER);
        }
        else if (intakeToggler.currentState() == 1) {
            lefIntake.setPower(INTAKE_POWER);
            rightIntake.setPower(INTAKE_POWER);
        }
        else {
            lefIntake.setPower(0.0);
            rightIntake.setPower(0.0);
        }
    }

    public void setIntakeMotorPowers(double power){
        lefIntake.setPower(power);
        rightIntake.setPower(power);
    }
}
