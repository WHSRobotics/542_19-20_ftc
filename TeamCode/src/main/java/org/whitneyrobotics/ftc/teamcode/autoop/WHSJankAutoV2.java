package org.whitneyrobotics.ftc.teamcode.autoop;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.whitneyrobotics.ftc.teamcode.lib.geometry.Coordinate;
import org.whitneyrobotics.ftc.teamcode.lib.geometry.Position;
import org.whitneyrobotics.ftc.teamcode.lib.util.SimpleTimer;
import org.whitneyrobotics.ftc.teamcode.lib.purepursuit.SwerveToTarget;
import org.whitneyrobotics.ftc.teamcode.subsys.WHSRobotImpl;

@Autonomous(name = "WHS Stone Auto", group = "a")
public class WHSJankAutoV2 extends OpMode {

    WHSRobotImpl robot;

    static final int FOUNDATION = 0;
    static final int SKYSTONE = 1;
    static final int RED = 0;
    static final int BLUE = 1;
    static final int INSIDE = 0;
    static final int OUTSIDE = 1;

    int state = 0;

    /**
     * Starting X Guidelines
     * Robot center to edge: 9 inches = 228 mm
     */

    static final int STARTING_POSITION = SKYSTONE;
    static final int STARTING_ALLIANCE = RED;
    static final int SKYBRIDGE_CROSSING_POSITION = INSIDE;
    static final double STARTING_COORDINATE_X = -1000;
    static final boolean PARTNER_MOVED_FOUNDATION = true;

    static final double STRAFE_MOTOR_POWER = 0.6;
    SimpleTimer startToStoneStrafeTimer = new SimpleTimer();
    SimpleTimer skystoneGrabberUpToDownTimer = new SimpleTimer();
    SimpleTimer stoneToMiddleStrafeTimer = new SimpleTimer();
    SimpleTimer skystoneGrabberDownToUpTimer = new SimpleTimer();

    double startToStoneStrafeDeadman = 3.5;
    double skystoneGrabberDelay = 1.0;
    double stoneToMiddleStrafeDeadman = 2.0;

    Coordinate[] startingCoordinateArray = new Coordinate[2];
    Position[] buildingZonePositionArray = new Position[2];
    Position[] parkingPositionArray = new Position[2];

    SwerveToTarget middleToBuildingSwerve;
    SwerveToTarget buildingToParkingSwerve;

    @Override
    public void init() {
        robot = new WHSRobotImpl(hardwareMap);

        startingCoordinateArray[RED] = new Coordinate(STARTING_COORDINATE_X, -1571, 0);
        startingCoordinateArray[BLUE] = new Coordinate(STARTING_COORDINATE_X, 1571,  0);

        buildingZonePositionArray[RED] = new Position(600, -1571);
        buildingZonePositionArray[BLUE] = new Position(600, 1571);

        parkingPositionArray[RED] = new Position(0,-1571);
        parkingPositionArray[BLUE] = new Position(0,1571);

        Position[] middleToBuildingSwervePositions = {startingCoordinateArray[STARTING_ALLIANCE], buildingZonePositionArray[STARTING_ALLIANCE]};
        Position[] buildingToParkSwervePositions = {buildingZonePositionArray[STARTING_ALLIANCE], parkingPositionArray[STARTING_ALLIANCE]};

        middleToBuildingSwerve = new SwerveToTarget(0.01, 0, 0, middleToBuildingSwervePositions, 80, .5, 2.2, 220, 1000);
        buildingToParkingSwerve = new SwerveToTarget(0.01, 0, 0, buildingToParkSwervePositions, 80, .5, 2.2, 220, 1000);

    }

    @Override
    public void start() {
        robot.setInitialCoordinate(startingCoordinateArray[STARTING_ALLIANCE]);
        startToStoneStrafeTimer.set(startToStoneStrafeDeadman);
    }

    @Override
    public void loop() {
        switch(state) {
            case 0:
                if (STARTING_ALLIANCE == BLUE) {
                    double[] motorPowers = {STRAFE_MOTOR_POWER, -STRAFE_MOTOR_POWER, -STRAFE_MOTOR_POWER, STRAFE_MOTOR_POWER};
                    robot.drivetrain.operate(motorPowers);
                } else {
                    double[] motorPowers = {-STRAFE_MOTOR_POWER, STRAFE_MOTOR_POWER, STRAFE_MOTOR_POWER, -STRAFE_MOTOR_POWER};
                    robot.drivetrain.operate(motorPowers);
                }
                if (startToStoneStrafeTimer.isExpired()) {
                    robot.drivetrain.operate(0.0, 0.0);
                    skystoneGrabberUpToDownTimer.set(skystoneGrabberDelay);
                    state++;
                }
            case 1:
//                robot.skystoneGrabber.setPosition(SkystoneGrabber.SkystoneGrabberPosition.DOWN);
                if (skystoneGrabberUpToDownTimer.isExpired()) {
                    stoneToMiddleStrafeTimer.set(stoneToMiddleStrafeDeadman);
                    state++;
                }
            case 2:
                if (STARTING_ALLIANCE == RED) {
                    double[] motorPowers = {STRAFE_MOTOR_POWER, -STRAFE_MOTOR_POWER, -STRAFE_MOTOR_POWER, STRAFE_MOTOR_POWER};
                    robot.drivetrain.operate(motorPowers);
                } else {
                    double[] motorPowers = {-STRAFE_MOTOR_POWER, STRAFE_MOTOR_POWER, STRAFE_MOTOR_POWER, -STRAFE_MOTOR_POWER};
                    robot.drivetrain.operate(motorPowers);
                }
                if (stoneToMiddleStrafeTimer.isExpired()) {
                    robot.drivetrain.operate(0.0, 0.0);
                    state++;
                }
            case 3:
                robot.estimatePosition();
                robot.estimateHeading();
                double[] motorPowers = middleToBuildingSwerve.calculateMotorPowers(robot.getCoordinate(), robot.drivetrain.getWheelVelocities(), false);
                robot.drivetrain.operate(motorPowers);
                if (!middleToBuildingSwerve.inProgress()) {
                    skystoneGrabberDownToUpTimer.set(skystoneGrabberDelay);
                    state++;
                }
            case 4:
                //robot.skystoneGrabber.setPosition(SkystoneGrabber.SkystoneGrabberPosition.UP);
                if (skystoneGrabberDownToUpTimer.isExpired()) {
                    state++;
                }
            case 5:
                robot.estimateHeading();
                robot.estimatePosition();
                motorPowers = buildingToParkingSwerve.calculateMotorPowers(robot.getCoordinate(), robot.drivetrain.getWheelVelocities(), true);
                robot.drivetrain.operate(motorPowers);
                if (!buildingToParkingSwerve.inProgress()) {
                    state++;
                }
        }
        telemetry.addData("IMU", robot.imu.getHeading());
        telemetry.addData("X", robot.getCoordinate().getX());
        telemetry.addData("Y", robot.getCoordinate().getY());
        telemetry.addData("Heading", robot.getCoordinate().getHeading());
    }
}
