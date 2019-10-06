package org.whitneyrobotics.ftc.teamcode.subsys;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.whitneyrobotics.ftc.teamcode.lib.subsys.MotorSubsystem;
import org.whitneyrobotics.ftc.teamcode.lib.subsys.drivetrain.MecanumDrivetrain;
import org.whitneyrobotics.ftc.teamcode.lib.util.Toggler;

/**
 * Created by Jason on 10/20/2017.
 */

public class Drivetrain implements MecanumDrivetrain, MotorSubsystem {

    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx backRight;

    private Toggler orientationSwitch = new Toggler(2);
    private Toggler fieldCentricSwitch = new Toggler(2);

    private static final double TRACK_WIDTH = 470;

    //TODO: measure actual wheel base
    private static final double WHEEL_BASE = 470;
    private static final double RADIUS_OF_WHEEL = 50;               //in mm
    private static final double CIRC_OF_WHEEL = RADIUS_OF_WHEEL * 2 * Math.PI;
    private static final double ENCODER_TICKS_PER_REV = 1120;      //Neverest 40 TODO: Change when we get 20s.
    private static final double GEAR_RATIO = 1.0;
    private static final double ENCODER_TICKS_PER_MM = ENCODER_TICKS_PER_REV / (CIRC_OF_WHEEL * GEAR_RATIO);

    private double[] encoderValues = {0.0, 0.0};

    private double vFL;
    private double vFR;
    private double vBL;
    private double vBR;
    private double rightX;
    private double robotAngle;
    private double r;


    public Drivetrain (HardwareMap driveMap) {

        frontLeft = driveMap.get(DcMotorEx.class, "driveFL");
        frontRight = driveMap.get(DcMotorEx.class, "driveFR");
        backLeft = driveMap.get(DcMotorEx.class, "driveBL");
        backRight = driveMap.get(DcMotorEx.class, "driveBR");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //For 40s. TODO: Change this when we get more 20s.

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);


        orientationSwitch.setState(1);
    }

    @Override
    public void operateWithOrientation(double leftPower, double rightPower) {
        switch (orientationSwitch.currentState()) {
            case 0:
                operateLeft(rightPower);
                operateRight(leftPower);
                break;
            case 1:
                operateLeft(-leftPower);
                operateRight(-rightPower);
                break;
        }
    }

    @Override
    public void operateWithOrientationScaled(double leftPower, double rightPower) {
        double leftScaledPower = Math.pow(leftPower, 3);
        double rightScaledPower = Math.pow(rightPower, 3);

        operateWithOrientation(leftScaledPower, rightScaledPower);
    }

    @Override
    public void operate(double leftPower, double rightPower) {
        frontLeft.setPower(leftPower);
        backLeft.setPower(leftPower);
        frontRight.setPower(rightPower);
        backRight.setPower(rightPower);
    }

    @Override
    public void operate(double[] powers) {
        if ((powers.length != 2) && (powers.length != 4)) {
            throw new IllegalArgumentException("drivetrain power array not 2 or 4 in length");
        }
        if (powers.length == 2) {
            frontLeft.setPower(powers[0]);
            backLeft.setPower(powers[0]);
            frontRight.setPower(powers[1]);
            backRight.setPower(powers[1]);
        }
        else if (powers.length == 4) {
            frontLeft.setPower(powers[0]);
            frontRight.setPower(powers[1]);
            backLeft.setPower(powers[2]);
            backRight.setPower(powers[3]);
        }
    }

    @Override
    public void operateLeft(double leftPower) {
        frontLeft.setPower(leftPower);
        backLeft.setPower(leftPower);
    }

    @Override
    public void operateRight(double rightPower) {
        frontRight.setPower(rightPower);
        backRight.setPower(rightPower);
    }

    @Override
    public void switchOrientation(boolean gamepadInput) {
        orientationSwitch.changeState(gamepadInput);
    }

    @Override
    public String getOrientation() {
        return orientationSwitch.currentState() == 0 ? "normal" : "reversed";
    }

    public static double getTrackWidth() {
        return TRACK_WIDTH;
    }

    public static double getWheelBase() {
        return WHEEL_BASE;
    }

    public double getRightEncoderPosition()
    {
        /*double rightTotal = backRight.getCurrentPosition() + frontRight.getCurrentPosition();
        return rightTotal * 0.5;*/
        return frontRight.getCurrentPosition();
    }

    public double getLeftEncoderPosition()
    {
        /*double leftTotal = backLeft.getCurrentPosition() +frontLeft.getCurrentPosition();
        return leftTotal * 0.5;*/
        return backLeft.getCurrentPosition();
    }

    public double getEncoderPosition() {
        double position = frontRight.getCurrentPosition() + frontLeft.getCurrentPosition() + backRight.getCurrentPosition() + backLeft.getCurrentPosition();
        return position * 0.25;
    }

    public double[] getEncoderDelta() {
        double currentLeft = getLeftEncoderPosition();
        double currentRight = getRightEncoderPosition();

        double[] encoderDistances = {currentLeft - encoderValues[0], currentRight - encoderValues[1]};

        encoderValues[0] = currentLeft;
        encoderValues[1] = currentRight;

        return encoderDistances;
    }

    public double[] getWheelVelocities() {
        double[] wheelVelocities = {encToMM(backLeft.getVelocity()), encToMM(frontRight.getVelocity())};
        return wheelVelocities;
    }

    public double[] getAllWheelVelocities() {
        double[] wheelVelocities = {encToMM(frontLeft.getVelocity()), encToMM(frontRight.getVelocity()), encToMM(backLeft.getVelocity()), encToMM(backRight.getVelocity())};
        return wheelVelocities;
    }

    @Override
    public double encToMM(double encoderTicks) {
        return encoderTicks * (1/ENCODER_TICKS_PER_MM);
    }

    @Override
    public void setRunMode(DcMotor.RunMode runMode) {
        frontLeft.setMode(runMode);
        frontRight.setMode(runMode);
        backLeft.setMode(runMode);
        backRight.setMode(runMode);
    }

    @Override
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        frontLeft.setZeroPowerBehavior(zeroPowerBehavior);
        frontRight.setZeroPowerBehavior(zeroPowerBehavior);
        backLeft.setZeroPowerBehavior(zeroPowerBehavior);
        backRight.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public double getAbsPowerAverage() {
        return (Math.abs(frontLeft.getPower()) + Math.abs(frontRight.getPower()))/2;
    }

    @Override
    public void operateMecanumDrive(double gamepadInputX, double gamepadInputY, double gamepadInputTurn, double heading){
        r = Math.hypot(gamepadInputX, -gamepadInputY);
        robotAngle = (Math.atan2(-gamepadInputY, gamepadInputX) - Math.PI / 4);
        if (fieldCentricSwitch.currentState() == 1) {
            robotAngle -= heading * Math.PI / 180;
        }

        rightX = gamepadInputTurn;
        vFL = r * Math.cos(robotAngle) + rightX;
        vFR = r * Math.sin(robotAngle) - rightX;
        vBL = r * Math.sin(robotAngle) + rightX;
        vBR = r * Math.cos(robotAngle) - rightX;
        frontLeft.setPower(vFL);
        frontRight.setPower(vFR);
        backLeft.setPower(vBL);
        backRight.setPower(vBR);
    }

    @Override
    public void switchFieldCentric(boolean gamepadInput) {
        fieldCentricSwitch.changeState(gamepadInput);
    }



}
