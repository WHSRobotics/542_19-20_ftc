package org.whitneyrobotics.ftc.teamcode.subsys;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.whitneyrobotics.ftc.teamcode.lib.util.Toggler;

public class SkystoneGrabber {

    Servo skystoneArm;
    Servo skystoneClaw;

    double[] armPositions = {0.145 ,0.31, 0.96};
    double[] clawPositions = {0.145 ,0.31, 0.96};
    private Toggler operateSkystoneGrabberTog;

    public enum SkystoneArmPosition {
        REST, UP, DOWN;
    }
    public enum SkystoneClawPosition {
        NOT_GRABBED, GRABBED;
    }

    public SkystoneGrabber(HardwareMap skystoneMap){
        skystoneArm = skystoneMap.servo.get("skystoneArm");
        skystoneClaw = skystoneMap.servo.get("skystoneClaw");
        operateSkystoneGrabberTog = new Toggler(6);
        //setPosition(SkystoneArmPosition.DOWN);
    }

    public void setPosition (SkystoneArmPosition armPosition, SkystoneClawPosition clawPosition){
        skystoneArm.setPosition(armPositions[armPosition.ordinal()]);
        skystoneClaw.setPosition(clawPositions[clawPosition.ordinal()]);
    }

    public void operate (boolean gamepadInputIncrement, boolean gamepadInputDecrement){
        operateSkystoneGrabberTog.changeState(gamepadInputIncrement, gamepadInputDecrement);
        switch (operateSkystoneGrabberTog.currentState()) {
            case 0:
                setPosition(SkystoneArmPosition.REST, SkystoneClawPosition.NOT_GRABBED);
                break;
            case 1:
                setPosition(SkystoneArmPosition.DOWN, SkystoneClawPosition.NOT_GRABBED);
                break;
            case 2:
                setPosition(SkystoneArmPosition.DOWN, SkystoneClawPosition.GRABBED);
                break;
            case 3:
                setPosition(SkystoneArmPosition.UP, SkystoneClawPosition.GRABBED);
                break;
            case 4:
                setPosition(SkystoneArmPosition.DOWN, SkystoneClawPosition.GRABBED);
                break;
            case 5:
                setPosition(SkystoneArmPosition.DOWN, SkystoneClawPosition.NOT_GRABBED);
                break;
        }
    }
}
