package org.whitneyrobotics.ftc.teamcode.subsys;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.whitneyrobotics.ftc.teamcode.lib.subsys.robot.WHSRobot;
import org.whitneyrobotics.ftc.teamcode.lib.util.Coordinate;
import org.whitneyrobotics.ftc.teamcode.lib.util.Functions;
import org.whitneyrobotics.ftc.teamcode.lib.util.PIDController;
import org.whitneyrobotics.ftc.teamcode.lib.util.Position;
import org.whitneyrobotics.ftc.teamcode.lib.util.RobotConstants;

public class WHSRobotTele implements WHSRobot {
    public Drivetrain drivetrain;
    public IMU imu;
    public TeleIntake intake;
    public FoundationPuller foundationPuller;
    public Outtake outtake;
   // public SkystoneGrabber skystoneGrabber;
    public Capstone capstone;
    public BackGate backGate;
    Coordinate currentCoord;
    private double targetHeading; //field frame
    public double angleToTargetDebug;
    public double distanceToTargetDebug = 0;
    public Position vectorToTargetDebug = new Position(542, 542);
    private double lastKnownHeading = 0.1;

    private static double DEADBAND_DRIVE_TO_TARGET = RobotConstants.DEADBAND_DRIVE_TO_TARGET; //in mm
    private static double DEADBAND_ROTATE_TO_TARGET = RobotConstants.DEADBAND_ROTATE_TO_TARGET; //in degrees

    public static double DRIVE_MIN = RobotConstants.drive_min;
    public static double DRIVE_MAX = RobotConstants.drive_max;
    public static double ROTATE_MIN = RobotConstants.rotate_min;
    public static double ROTATE_MAX = RobotConstants.rotate_max;

    private static double ROTATE_KP = RobotConstants.R_KP;
    private static double ROTATE_KI = RobotConstants.R_KI;
    private static double ROTATE_KD = RobotConstants.R_KD;

    private static double DRIVE_KP = RobotConstants.D_KP;
    private static double DRIVE_KI = RobotConstants.D_KI;
    private static double DRIVE_KD = RobotConstants.D_KD;

    public PIDController rotateController = new PIDController(ROTATE_KP, ROTATE_KI, ROTATE_KD);
    public PIDController driveController = new PIDController(DRIVE_KP, DRIVE_KI, DRIVE_KD);

    private boolean firstRotateLoop = true;
    private boolean firstDriveLoop = true;
    private boolean driveBackwards;

    private int driveSwitch = 0;

    private boolean driveToTargetInProgress = false;
    private boolean rotateToTargetInProgress = false;

    private double[] encoderDeltas;
    private double robotX;
    private double robotY;
    private double distance;

    public WHSRobotTele(HardwareMap hardwareMap) {
        DEADBAND_DRIVE_TO_TARGET = RobotConstants.DEADBAND_DRIVE_TO_TARGET; //in mm
        DEADBAND_ROTATE_TO_TARGET = RobotConstants.DEADBAND_ROTATE_TO_TARGET; //in degrees

        DRIVE_MIN = RobotConstants.drive_min;
        DRIVE_MAX = RobotConstants.drive_max;
        ROTATE_MIN = RobotConstants.rotate_min;
        ROTATE_MAX = RobotConstants.rotate_max;

        ROTATE_KP = RobotConstants.R_KP;
        ROTATE_KI = RobotConstants.R_KI;
        ROTATE_KD = RobotConstants.R_KD;

        DRIVE_KP = RobotConstants.D_KP;
        DRIVE_KI = RobotConstants.D_KI;
        DRIVE_KD = RobotConstants.D_KD;

        drivetrain = new Drivetrain(hardwareMap);
        imu = new IMU(hardwareMap);
        intake = new TeleIntake(hardwareMap);
        foundationPuller = new FoundationPuller(hardwareMap);
        outtake = new Outtake(hardwareMap);
       // skystoneGrabber = new SkystoneGrabber(hardwareMap);
        capstone = new Capstone(hardwareMap);
        backGate = new BackGate(hardwareMap);

        currentCoord = new Coordinate(0.0, 0.0, 0.0);
    }

    @Override
    public void driveToTarget(Position targetPos, boolean backwards) {
        Position vectorToTarget = Functions.Positions.subtract(targetPos, currentCoord.getPos()); //field frame
        vectorToTarget = Functions.field2body(vectorToTarget, currentCoord); //body frame
        vectorToTargetDebug = vectorToTarget;
        double distanceToTarget = vectorToTarget.getX()/*Functions.calculateMagnitude(vectorToTarget) * (vectorToTarget.getX() >= 0 ? 1 : -1)*/;
        distanceToTargetDebug = distanceToTarget;

        double degreesToRotate = Math.atan2(vectorToTarget.getY(), vectorToTarget.getX()); //from -pi to pi rad
        degreesToRotate = degreesToRotate * 180 / Math.PI;
        targetHeading = Functions.normalizeAngle(currentCoord.getHeading() + degreesToRotate); //-180 to 180 deg

        switch (driveSwitch) {
            case 0:
                driveToTargetInProgress = true;
                rotateToTarget(targetHeading, backwards);
                if (!rotateToTargetInProgress()) {
                    driveSwitch = 1;
                }
                break;
            case 1:

                if (firstDriveLoop) {
                    driveToTargetInProgress = true;
                    driveController.init(distanceToTarget);
                    firstDriveLoop = false;
                }

                driveController.setConstants(DRIVE_KP, DRIVE_KI, DRIVE_KD);
                driveController.calculate(distanceToTarget);

                double power = Functions.map(Math.abs(driveController.getOutput()), DEADBAND_DRIVE_TO_TARGET, 1500, DRIVE_MIN, DRIVE_MAX);

                // this stuff may be causing the robot to oscillate around the target position
                if (distanceToTarget < 0) {
                    power = -power;
                } else if (distanceToTarget > 0) {
                    power = Math.abs(power);
                }
                if (Math.abs(distanceToTarget) > DEADBAND_DRIVE_TO_TARGET) {
                    driveToTargetInProgress = true;
                    drivetrain.operateLeft(power);
                    drivetrain.operateRight(power);
                } else {
                    drivetrain.operateRight(0.0);
                    drivetrain.operateLeft(0.0);
                    driveToTargetInProgress = false;
                    rotateToTargetInProgress = false;
                    firstDriveLoop = true;
                    driveSwitch = 0;
                }
                // end of weird code
                break;
        }
    }

    @Override
    public void rotateToTarget(double targetHeading, boolean backwards) {

        double angleToTarget = targetHeading - currentCoord.getHeading();
        /*if (backwards && angleToTarget > 90) {
            angleToTarget = angleToTarget - 180;
            driveBackwards = true;
        }
        else if (backwards && angleToTarget < -90) {
            angleToTarget = angleToTarget + 180;
            driveBackwards = true;
        }*/
        if (backwards) {
            angleToTarget = Functions.normalizeAngle(angleToTarget + 180); //-180 to 180 deg
            driveBackwards = true;
        } else {
            angleToTarget = Functions.normalizeAngle(angleToTarget);
            driveBackwards = false;
        }

        angleToTargetDebug = angleToTarget;

        if (firstRotateLoop) {
            rotateToTargetInProgress = true;
            rotateController.init(angleToTarget);
            firstRotateLoop = false;
        }

        rotateController.setConstants(ROTATE_KP, ROTATE_KI, ROTATE_KD);
        rotateController.calculate(angleToTarget);

        double power = (rotateController.getOutput() >= 0 ? 1 : -1) * (Functions.map(Math.abs(rotateController.getOutput()), 0, 180, ROTATE_MIN, ROTATE_MAX));

        if (Math.abs(angleToTarget) > DEADBAND_ROTATE_TO_TARGET/* && rotateController.getDerivative() < 40*/) {
            drivetrain.operateLeft(-power);
            drivetrain.operateRight(power);
            rotateToTargetInProgress = true;
        } else {
            drivetrain.operateLeft(0.0);
            drivetrain.operateRight(0.0);
            rotateToTargetInProgress = false;
            firstRotateLoop = true;
        }
    }

    @Override
    public boolean driveToTargetInProgress() {
        return driveToTargetInProgress;
    }

    @Override
    public boolean rotateToTargetInProgress() {
        return rotateToTargetInProgress;
    }

    @Override
    public void estimatePosition() {
        encoderDeltas = drivetrain.getEncoderDelta();
        distance = drivetrain.encToMM((encoderDeltas[0] + encoderDeltas[1]) / 2);
        robotX += distance * Functions.cosd(getCoordinate().getHeading());
        robotY += distance * Functions.sind(getCoordinate().getHeading());
        currentCoord.setX(robotX);
        currentCoord.setY(robotY);
    }

    public void deadWheelEstimatePosition() {
        encoderDeltas = drivetrain.getEncoderDelta();
        double angleDelta = currentCoord.getHeading() - lastKnownHeading;
        double deltaX = drivetrain.encToMM(encoderDeltas[0]);
        double deltaY = drivetrain.encToMM(encoderDeltas[1]);

        Position bodyVector = new Position(deltaX, deltaY);
        Position fieldVector = Functions.body2field(bodyVector, currentCoord);
        currentCoord.setPos(Functions.Positions.add(fieldVector, currentCoord));
    }

    public void mecanumEstimatePosition() {
        encoderDeltas = drivetrain.getMecanumEncoderDelta();
        double theta = Math.atan(encoderDeltas[1]/encoderDeltas[0]);

    }

    @Override
    public void estimateHeading() {
        double currentHeading;
        currentHeading = Functions.normalizeAngle(imu.getHeading() + imu.getImuBias()); //-180 to 180 deg

        currentCoord.setHeading(currentHeading); //updates global variable
    }

    @Override
    public void setInitialCoordinate(Coordinate initCoord) {
        currentCoord = initCoord;
        robotX = initCoord.getX();
        robotY = initCoord.getY();
        imu.setImuBias(currentCoord.getHeading());
    }

    @Override
    public void setCoordinate(Coordinate coord) {
        currentCoord = coord;
        imu.setImuBias(currentCoord.getHeading());
    }

    @Override
    public Coordinate getCoordinate() {
        return currentCoord;
    }
}
