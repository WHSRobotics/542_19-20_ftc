package org.whitneyrobotics.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.whitneyrobotics.ftc.teamcode.lib.util.Coordinate;
import org.whitneyrobotics.ftc.teamcode.lib.util.Position;
import org.whitneyrobotics.ftc.teamcode.lib.util.StrafeToTarget;
import org.whitneyrobotics.ftc.teamcode.subsys.WHSRobotImpl;

public class StrafeToTargetTest extends OpMode {
    WHSRobotImpl robot;
    StrafeToTarget strafe1;
    double kP = 1.0;
    double kA = 1.0;
    double kV = 1.0;
    double velocityConstant = 3.0;
    double lookaheadDistance = 2.0;
    Coordinate startingCoordinate = new Coordinate(0,0,90);
    Position p1 = new Position(300,0);
    Position p2 = new Position(600,600);

    Position[] positions = {startingCoordinate.getPos(), p1, p2};
    @Override
    public void init() {
        robot = new WHSRobotImpl(hardwareMap);
        strafe1 = new StrafeToTarget(kP, kV,kA,positions, 80, .7, 0.01, velocityConstant, lookaheadDistance );
    }

    @Override
    public void loop() {
        robot.deadWheelEstimatePosition();
        robot.drivetrain.operate(strafe1.calculateMotorPowers(robot.getCoordinate(), robot.drivetrain.getWheelVelocities()));

    }
}
