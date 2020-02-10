package org.whitneyrobotics.ftc.teamcode.subsys;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Func;
import org.openftc.easyopencv.OpenCvCamera;
import org.whitneyrobotics.ftc.teamcode.lib.subsys.robot.WHSRobot;
import org.whitneyrobotics.ftc.teamcode.lib.util.Alliance;
import org.whitneyrobotics.ftc.teamcode.lib.util.Coordinate;
import org.whitneyrobotics.ftc.teamcode.lib.util.Functions;
import org.whitneyrobotics.ftc.teamcode.lib.util.PIDController;
import org.whitneyrobotics.ftc.teamcode.lib.util.Position;
import org.whitneyrobotics.ftc.teamcode.lib.util.RobotConstants;

/**
 * Created by Jason on 10/20/2017.
 */

public class WHSRobotImpl implements WHSRobot {

    public Drivetrain drivetrain;
    public IMU imu;
    public Intake intake;
    public FoundationPuller foundationPuller;
    public Outtake outtake;
   // public SkystoneGrabber skystoneGrabber;
    public Capstone capstone;
  //  public Vuforia vuforia;
    public ImprovedSkystoneDetector skystoneDetector;
    public OpenCvCamera webcam;
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

    private double[] encoderDeltas = {0.0, 0.0};
    private double[] encoderValues = {0.0, 0.0};
    private double robotX;
    private double robotY;
    private double distance;

    public WHSRobotImpl(HardwareMap hardwareMap) {
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
        intake = new Intake(hardwareMap);
        foundationPuller = new FoundationPuller(hardwareMap);
        outtake = new Outtake(hardwareMap);
     //   skystoneGrabber = new SkystoneGrabber(hardwareMap);
        capstone = new Capstone(hardwareMap);
        //vuforia = new Vuforia(hardwareMap);
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
        encoderDeltas = drivetrain.getAllEncoderDelta();
        double leftDelta = encoderDeltas[0];
        double rightDelta = encoderDeltas[1];
        double backDelta = encoderDeltas[2];


        double angleDelta = currentCoord.getHeading() - lastKnownHeading;
        double deltaX = 0.0;
        double deltaY = 0.0;

        double X = 0.0;
        if(Math.abs(rightDelta) > Math.abs(leftDelta)){
            X = leftDelta/angleDelta;
            deltaY = Functions.sind(-angleDelta) * (X + drivetrain.L_DEAD_WHEEL_TO_ROBOT_CENTER);
            deltaX = (X + drivetrain.L_DEAD_WHEEL_TO_ROBOT_CENTER) - (Functions.cosd(angleDelta) * (X + drivetrain.L_DEAD_WHEEL_TO_ROBOT_CENTER));

        }
        if(Math.abs(leftDelta) > Math.abs(rightDelta)){
            X = rightDelta/-angleDelta;
            deltaY = Functions.sind(angleDelta) * (X + drivetrain.L_DEAD_WHEEL_TO_ROBOT_CENTER);
            deltaX = (Functions.cosd(angleDelta) * (X + drivetrain.L_DEAD_WHEEL_TO_ROBOT_CENTER)) - (X + drivetrain.L_DEAD_WHEEL_TO_ROBOT_CENTER);

        }

        double G = backDelta - (angleDelta*drivetrain.B_DEAD_WHEEL_TO_ROBOT_CENTER);
        double deltaY2 = Functions.sind(angleDelta)*G;
        double deltaX2 = Functions.cosd(angleDelta)*G;

        deltaX = deltaX + deltaX2;
        deltaY = deltaY + deltaY2;

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
        lastKnownHeading = currentCoord.getHeading();
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

    public void estimateCoordinate(){
        double[] currentEncoderValues = drivetrain.getEncoderPosition();
        encoderDeltas[0] = currentEncoderValues[0] - encoderValues[0];
        encoderDeltas[1] = currentEncoderValues[1] - encoderValues[1];
        double currentHeading = Functions.normalizeAngle(Math.toDegrees(drivetrain.encToMM((currentEncoderValues[1] - currentEncoderValues[0])/2/Drivetrain.getTrackWidth())) + imu.getImuBias()); //-180 to 180 deg
        currentCoord.setHeading(currentHeading); //updates global variable

        double deltaS = drivetrain.encToMM((encoderDeltas[0] + encoderDeltas[1])/2);
        double deltaHeading = Math.toDegrees(drivetrain.encToMM((encoderDeltas[1] - encoderDeltas[0])/Drivetrain.getTrackWidth()));
        robotX += deltaS * Functions.cosd(lastKnownHeading + deltaHeading/2);
        robotY += deltaS * Functions.sind(lastKnownHeading + deltaHeading/2);

        currentCoord.setX(robotX);
        currentCoord.setY(robotY);
        encoderValues[0] = currentEncoderValues[0];
        encoderValues[1] = currentEncoderValues[1];
        lastKnownHeading = currentCoord.getHeading();
    }
}
