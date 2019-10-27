package org.whitneyrobotics.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.whitneyrobotics.ftc.teamcode.lib.util.Coordinate;
import org.whitneyrobotics.ftc.teamcode.lib.util.Position;
import org.whitneyrobotics.ftc.teamcode.lib.util.SwerveToTarget;
import org.whitneyrobotics.ftc.teamcode.subsys.WHSRobotImpl;


@Autonomous(name = "Swerve to target test")
public class SwerveToTargetTest extends OpMode {
    WHSRobotImpl robot;
    Coordinate startingCoordinate = new Coordinate(0, 0, -90);
    static Position p1 = new Position(0, 0);
    static Position p2 = new Position(0, 1800);/*
    static Position p3 = new Position(1800, 2700);
    static Position p4 = new Position(-1800, 2700);
    static Position p5 = new Position(-1800, 0);*/

    Position[] positions1 = {startingCoordinate.getPos(), p1, p2};
    SwerveToTarget swerve1;
    SwerveToTarget swerve2;
    int state = 1;
    double[] swervePowers;


    @Override
    public void init() {
        robot = new WHSRobotImpl(hardwareMap);
        robot.setInitialCoordinate(startingCoordinate);
        swerve1 = new SwerveToTarget(0.001, 0, 0, positions1, 80, 0.99, 3, 250);

    }

    @Override
    public void loop() {
        robot.estimateHeading();
        robot.estimatePosition();
        swervePowers = swerve1.calculateMotorPowers(robot.getCoordinate(), robot.drivetrain.getWheelVelocities(), true);

        robot.drivetrain.operate(swervePowers[0], swervePowers[1]);

        telemetry.addData("x", robot.getCoordinate().getX());
        telemetry.addData("y", robot.getCoordinate().getY());
        telemetry.addData("heading", robot.getCoordinate().getHeading());
        telemetry.addData("index of last closest point", swerve1.lastClosestPointIndex);
        telemetry.addData("left power", swervePowers[0]);
        telemetry.addData("right power", swervePowers[1]);
        for (int i = 0; i < swerve1.smoothedPath.length; i++) {
            telemetry.addData("point" + i, swerve1.smoothedPath[i].getX() + ", " + swerve1.smoothedPath[i].getY());
        }
        for (int i = 0; i < swerve1.targetVelocities.length; i++) {
            telemetry.addData("velocity" + i, swerve1.targetVelocities[i]);
        }
        telemetry.addData("lookahead point", swerve1.lookaheadPoint.getX() + ", " + swerve1.lookaheadPoint.getY());
    }
}
