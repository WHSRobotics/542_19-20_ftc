package org.whitneyrobotics.ftc.teamcode.subsys;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.whitneyrobotics.ftc.teamcode.lib.util.SimpleTimer;
import org.whitneyrobotics.ftc.teamcode.lib.util.Toggler;

public class TeleIntake {

    private DcMotorEx leftIntake;
    private DcMotorEx rightIntake;
    private Servo intakePusher;
    //private Rev2mDistanceSensor stoneSensor;

    public enum IntakePusherPosition {
        DOWN, UP
    }

    // DOWN, UP
    private final double[] INTAKE_PUSHER_POSITIONS = {0.125, 0.52};
    private final double INTAKE_PUSHER_DOWN = INTAKE_PUSHER_POSITIONS[IntakePusherPosition.DOWN.ordinal()];
    private final double INTAKE_PUSHER_UP = INTAKE_PUSHER_POSITIONS[IntakePusherPosition.UP.ordinal()];

    public static final double AUTO_INTAKE_POWER = 0.60;
    public static final double INTAKE_POWER = 0.60;
    public static final double INTAKE_VELOCITY_THRESHOLD = 30;
    public static final double INTAKE_VELOCITY = 1800.0;
    public static final double AUTO_INTAKE_VELOCITY = 1500.0;
    public static final double INTAKE_JAM_FIX_DURATION = 0.02;
    public static final double STONE_SENSOR_DEADBAND = 140;
    private SimpleTimer fixJamTimer = new SimpleTimer();
    public boolean intakeJammed = false;

    private Toggler intakeToggler = new Toggler(2);

    public TeleIntake(HardwareMap intakeMap) {
        leftIntake = intakeMap.get(DcMotorEx.class, "leftIntake");
        rightIntake = intakeMap.get(DcMotorEx.class,"rightIntake");
        intakePusher = intakeMap.servo.get("intakePusher");
        //stoneSensor = intakeMap.get(Rev2mDistanceSensor.class ,"stoneSensor");

        leftIntake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void operateIntake(boolean gamepadInput1, boolean gamepadInput2) {
        intakeToggler.changeState(gamepadInput1);
        if (gamepadInput2) {
            leftIntake.setPower(-INTAKE_POWER);
            rightIntake.setPower(-INTAKE_POWER);
        } else if (intakeToggler.currentState() == 1) {
            leftIntake.setPower(INTAKE_POWER);
            rightIntake.setPower(INTAKE_POWER);
        } else {
            leftIntake.setPower(0.0);
            rightIntake.setPower(0.0);
        }
    }

    public void setMotorPowers(double power) {
        leftIntake.setPower(power);
        rightIntake.setPower(power);
    }

    public void autoIntakeWithJamDetection(double intakePower) {
        if (getAvgIntakeWheelVelocities() < INTAKE_VELOCITY_THRESHOLD) {
            if (!intakeJammed) {
                intakeJammed = true;
                fixJamTimer.set(INTAKE_JAM_FIX_DURATION);
            }
        }
        if (intakeJammed) {
            if (!fixJamTimer.isExpired()) {
                setMotorPowers(-intakePower);
            } else {
                intakeJammed = false;
            }
        } else {
            setMotorPowers(intakePower);
        }
    }

    public boolean isIntakeOn() {
        return intakeToggler.currentState() == 1;
    }

    public void setIntakePusherPosition(IntakePusherPosition intakePusherPosition) {
        intakePusher.setPosition(INTAKE_PUSHER_POSITIONS[intakePusherPosition.ordinal()]);
    }

    public double getAvgIntakeWheelVelocities(){
        return Math.abs((leftIntake.getVelocity() + rightIntake.getVelocity()) / 2);
    }

    public void setVelocity(double velocity){
        leftIntake.setVelocity(velocity);
        rightIntake.setVelocity(velocity);
    }

    /*public boolean stoneSensed(){
        return stoneSensor.getDistance(DistanceUnit.MM) < STONE_SENSOR_DEADBAND;
    }*/
}
