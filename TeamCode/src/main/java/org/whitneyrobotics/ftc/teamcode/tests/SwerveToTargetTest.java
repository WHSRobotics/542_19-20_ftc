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
    Coordinate startingCoordinate = new Coordinate(0,0,90);
    Position p05 = new Position(800, 152.786);
    Position p1 = new Position(1200, 600);
    Position p2 = new Position(800,1047.2);
    Position p3 = new Position(0, 1200);


    Position p4 = new Position(-800, 1352.786);
    Position p45 = new Position(-1200, 1800);
    Position p5 = new Position(-800, 2247.2);
    Position p6 = new Position(0, 2400);

    Position p13 = new Position(-800, 152.786);
    Position p12 = new Position(-1200, 600);
    Position p11 = new Position(-800,1047.2);
    Position p10 = new Position(0, 1200);


    Position p9 = new Position(800, 1352.786);
    Position p8 = new Position(1200, 1800);
    Position p7 = new Position(800, 2247.2);
    Position[] positions1 = {startingCoordinate.getPos(), p05, p1, p2, p3, p4, p45, p5, p6};
    Position[] positions2 = {p6,p7, p8, p9, p10, p11,p12,p13, startingCoordinate.getPos()};
    SwerveToTarget swerve1;
    SwerveToTarget swerve2;
    int state = 1;
    double[] swervePowers;


    @Override
    public void init() {
        robot = new WHSRobotImpl(hardwareMap);
        robot.setInitialCoordinate(startingCoordinate);
        swerve1 = new SwerveToTarget(0.001, 0,0, positions1, 80, 0.99, 3, 250);
        swerve2 = new SwerveToTarget(0.001, 0,0, positions2, 80, 0.99, 3, 250);

    }

    @Override
    public void loop() {
        robot.estimateHeading();
        robot.estimatePosition();

        switch(state) {
            case 1:
                swervePowers = swerve1.calculateMotorPowers(robot.getCoordinate(), robot.drivetrain.getWheelVelocities());
                if (!swerve1.inProgress()) {
                    state++;
                }
                break;
            case 2:
                swervePowers = swerve2.calculateMotorPowers(robot.getCoordinate(), robot.drivetrain.getWheelVelocities());

        }
        robot.drivetrain.operate(swervePowers);

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
        telemetry.addData("lookahead point",  swerve1.lookaheadPoint.getX() + ", " + swerve1.lookaheadPoint.getY());
    }
}
