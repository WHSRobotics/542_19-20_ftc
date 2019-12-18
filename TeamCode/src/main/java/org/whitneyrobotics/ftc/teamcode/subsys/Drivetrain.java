package org.whitneyrobotics.ftc.teamcode.subsys;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Func;
import org.whitneyrobotics.ftc.teamcode.lib.subsys.MotorSubsystem;
import org.whitneyrobotics.ftc.teamcode.lib.subsys.drivetrain.MecanumDrivetrain;
import org.whitneyrobotics.ftc.teamcode.lib.util.Functions;
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

    private static final double TRACK_WIDTH = 338;

    //TODO: measure actual wheel base
    private static final double WHEEL_BASE = 450;
    private static final double RADIUS_OF_WHEEL = 50;               //in mm
    private static final double CIRC_OF_WHEEL = RADIUS_OF_WHEEL * 2 * Math.PI;
    private static final double ENCODER_TICKS_PER_REV = 537.6;      //Orbital 20
    private static final double GEAR_RATIO = 1.0;
    private static final double ENCODER_TICKS_PER_MM = ENCODER_TICKS_PER_REV / (CIRC_OF_WHEEL * GEAR_RATIO);
    public static final double X_WHEEL_TO_ROBOT_CENTER = 100.0;
    public static final double Y_WHEEL_TO_ROBOT_CENTER = 100.0;

    private double[] encoderValues = {0.0, 0.0};

    private double vFL;
    private double vFR;
    private double vBL;
    private double vBR;
    private double rightX;
    private double robotAngle;
    private double r;

    private double[] lastKnownEncoderValues = {0,0,0,0};

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
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);


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
        double rightTotal = backRight.getCurrentPosition() + frontRight.getCurrentPosition();
        return rightTotal * 0.5;
        //return backRight.getCurrentPosition();
    }

    public double getLeftEncoderPosition()
    {
        double leftTotal = backLeft.getCurrentPosition() +frontLeft.getCurrentPosition();
        return leftTotal * 0.5;
        //return frontLeft.getCurrentPosition();
    }

    public double[] getEncoderPosition() {
        return new double[] {getLeftEncoderPosition(), getRightEncoderPosition()};
    }

    public double[] getEncoderDelta() {
        double currentLeft = getLeftEncoderPosition();
        double currentRight = getRightEncoderPosition();

        double[] encoderDistances = {currentLeft - encoderValues[0], currentRight - encoderValues[1]};

        encoderValues[0] = currentLeft; //Change in the X Odometry Wheel
        encoderValues[1] = currentRight; //Change in the Y Odometry wheel

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
        r = Math.hypot(-gamepadInputX, -gamepadInputY);
        robotAngle = (Math.atan2(-gamepadInputY, -gamepadInputX) - Math.PI / 4);
        if (fieldCentricSwitch.currentState() == 1) {
            robotAngle -= heading * Math.PI / 180;
        }
        double a = WHEEL_BASE / 2;
        double b = TRACK_WIDTH / 2;
        rightX = gamepadInputTurn;
//        vFL = r * Math.cos(robotAngle) + rightX;
//        vFR = r * Math.sin(robotAngle) - rightX;
//        vBL = r * Math.sin(robotAngle) + rightX;
//        vBR = r * Math.cos(robotAngle) - rightX;
        vFL = -gamepadInputY + gamepadInputX + gamepadInputTurn;
        vFR = -gamepadInputY - gamepadInputX - gamepadInputTurn;
        vBL = -gamepadInputY - gamepadInputX + gamepadInputTurn;
        vBR = -gamepadInputY + gamepadInputX - gamepadInputTurn;
        frontLeft.setPower(vFL);
        frontRight.setPower(vFR);
        backLeft.setPower(vBL);
        backRight.setPower(vBR);
    }

    public void operateMecanumDriveScaled(double gamepadInputX, double gamepadInputY, double gamepadInputTurn, double heading){
        double scaledY = Math.pow(gamepadInputY, 3);
        double scaledX = Math.pow(gamepadInputX, 3);
        double scaledTurn = Math.pow(gamepadInputTurn, 3);

        vFL = -scaledY + scaledX + scaledTurn;
        vFR = -scaledY - scaledX - scaledTurn;
        vBL = -scaledY - scaledX + scaledTurn;
        vBR = -scaledY + scaledX - scaledTurn;
        frontLeft.setPower(vFL);
        frontRight.setPower(vFR);
        backLeft.setPower(vBL);
        backRight.setPower(vBR);
    }

    @Override
    public void switchFieldCentric(boolean gamepadInput) {
        fieldCentricSwitch.changeState(gamepadInput);
    }

    public String getFieldCentric() {
        return fieldCentricSwitch.currentState() == 0 ? "Robot Centric" : "Field Centric";
    }

    public double[] getAllEncoderValues(){
        double[] encoderValues = {frontLeft.getCurrentPosition(), frontRight.getCurrentPosition(), backLeft.getCurrentPosition(), backRight.getCurrentPosition()};
        return encoderValues;
    }

    public double[] getMecanumEncoderDelta(){
        double currentFLBR = (frontLeft.getCurrentPosition() + backRight.getCurrentPosition())/2;
        double currentFRBL = (frontRight.getCurrentPosition() + backLeft.getCurrentPosition())/2;

        double[] encoderDistances = {currentFLBR - encoderValues[0], currentFRBL - encoderValues[1]};

        encoderValues[0] = currentFLBR;
        encoderValues[1] = currentFRBL;

        return encoderDistances;
    }

    public void resetEncoders(){
        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setTargetPosition(int targetPosition, double power){
        frontLeft.setTargetPosition(targetPosition);
        frontRight.setTargetPosition(targetPosition);
        backLeft.setTargetPosition(targetPosition);
        backRight.setTargetPosition(targetPosition);

        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);

    }
}
