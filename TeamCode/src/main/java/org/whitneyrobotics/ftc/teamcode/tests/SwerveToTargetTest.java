package org.whitneyrobotics.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.whitneyrobotics.ftc.teamcode.lib.util.Coordinate;
import org.whitneyrobotics.ftc.teamcode.lib.util.Position;
import org.whitneyrobotics.ftc.teamcode.lib.util.SwerveToTarget;
import org.whitneyrobotics.ftc.teamcode.subsys.WHSRobotImpl;


@Autonomous(name = "Swerve to target test" )
public class SwerveToTargetTest extends OpMode {
    WHSRobotImpl robot;
    Coordinate startingCoordinate = new Coordinate(0,0,0,90);
    Position p1 = new Position(0, 300, 150);
    Position p2 = new Position(600,600,150);
    Position[] positions = {startingCoordinate.getPos(), p1, p2};
    SwerveToTarget swerve1;

    @Override
    public void init() {
        robot = new WHSRobotImpl(hardwareMap);
        robot.setInitialCoordinate(startingCoordinate);
        swerve1 = new SwerveToTarget(0.005, 0.245 ,0.542, positions, 10, 0.8542,  2, 50, robot.drivetrain.getTrackWidth());
    }

    @Override
    public void loop() {
        robot.estimateHeading();
        robot.estimatePosition();

        double[] swerve1Powers = swerve1.calculateMotorPowers(robot.getCoordinate(), robot.drivetrain.getWheelVelocities());
        robot.drivetrain.operate(swerve1Powers);

        telemetry.addData("x", robot.getCoordinate().getX());
        telemetry.addData("y", robot.getCoordinate().getY());
        telemetry.addData("heading", robot.getCoordinate().getHeading());
        telemetry.addData("index of last closest point", swerve1.lastClosestPointIndex);
        telemetry.addData("left power", swerve1Powers[0]);
        telemetry.addData("right power", swerve1Powers[1]);
        for (int i = 0; i < swerve1.smoothedPath.length; i++) {
            telemetry.addData("point" + i, swerve1.smoothedPath[i].getX() + ", " + swerve1.smoothedPath[i].getY());
        }
        for (int i = 0; i < swerve1.targetVelocities.length; i++) {
            telemetry.addData("velocity" + i, swerve1.targetVelocities[i]);
        }
        telemetry.addData("lookahead point",  swerve1.lookaheadPoint.getX() + ", " + swerve1.lookaheadPoint.getY());
    }
}
