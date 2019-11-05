package org.whitneyrobotics.ftc.teamcode.autoop;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.whitneyrobotics.ftc.teamcode.lib.util.Coordinate;
import org.whitneyrobotics.ftc.teamcode.lib.util.Position;
import org.whitneyrobotics.ftc.teamcode.lib.util.SimpleTimer;
import org.whitneyrobotics.ftc.teamcode.lib.util.SwerveToTarget;
import org.whitneyrobotics.ftc.teamcode.subsys.Intake;
import org.whitneyrobotics.ftc.teamcode.subsys.WHSRobotImpl;
@Autonomous(name = "WHS Park Auto", group = "a")
public class WHSJankAuto extends OpMode {

    WHSRobotImpl robot;

    static final int RED = 0;
    static final int BLUE = 1;

    /**
     * Starting X Guidelines
     * Robot center to edge: 9 inches = 228 mm
     */

    static final int STARTING_ALLIANCE = RED;
    static final double STARTING_COORDINATE_X = -1000;

    Coordinate[] startingCoordinateArray = new Coordinate[2];
    Position[] parkingWaypointPositionArray = new Position[2];
    Position[] skybridgePositionArray = new Position[2];

    SimpleTimer timer = new SimpleTimer();

    double dropIntake = 2.0;


    SwerveToTarget startToParkSwerve;

    @Override
    public void init() {
        robot = new WHSRobotImpl(hardwareMap);

        startingCoordinateArray[RED] = new Coordinate(STARTING_COORDINATE_X, -1571, -90);
        startingCoordinateArray[BLUE] = new Coordinate(STARTING_COORDINATE_X, 1571,  90);

        parkingWaypointPositionArray[RED] = new Position(STARTING_COORDINATE_X, -1100);
        parkingWaypointPositionArray[BLUE] = new Position(STARTING_COORDINATE_X, 1100);

        skybridgePositionArray[RED] = new Position(0,-1100);
        skybridgePositionArray[BLUE] = new Position(0,1100);

        robot.setInitialCoordinate(startingCoordinateArray[STARTING_ALLIANCE]);

        Position[] startToParkSwervePositions = {startingCoordinateArray[STARTING_ALLIANCE], parkingWaypointPositionArray[STARTING_ALLIANCE], skybridgePositionArray[STARTING_ALLIANCE]};

        startToParkSwerve = new SwerveToTarget(.001, 0, 0, startToParkSwervePositions, 80, .9, 2.2, 220);
    }

    @Override
    public void start() {
        timer.set(dropIntake);
    }

    @Override
    public void loop() {
        robot.estimateHeading();
        robot.estimatePosition();

        robot.intake.setIntakePusherPosition(Intake.IntakePusherPosition.UP);
        if (timer.isExpired()) {
            double[] motorPowers = startToParkSwerve.calculateMotorPowers(robot.getCoordinate(), robot.drivetrain.getWheelVelocities(), true);
            robot.drivetrain.operate(motorPowers);
        }

        telemetry.addData("IMU", robot.imu.getHeading());
        telemetry.addData("X", robot.getCoordinate().getX());
        telemetry.addData("Y", robot.getCoordinate().getY());
        telemetry.addData("Heading", robot.getCoordinate().getHeading());
    }
}
