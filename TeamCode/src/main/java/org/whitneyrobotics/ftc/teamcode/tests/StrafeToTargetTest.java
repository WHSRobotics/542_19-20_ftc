package org.whitneyrobotics.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.whitneyrobotics.ftc.teamcode.lib.util.Coordinate;
import org.whitneyrobotics.ftc.teamcode.lib.util.Position;
import org.whitneyrobotics.ftc.teamcode.lib.util.StrafeToTarget;
import org.whitneyrobotics.ftc.teamcode.subsys.DeadWheelPickup;
import org.whitneyrobotics.ftc.teamcode.subsys.WHSRobotImpl;

@Autonomous(name = "StrafeToTarget Test")
public class StrafeToTargetTest extends OpMode {
    WHSRobotImpl robot;
    StrafeToTarget strafe1;
    double[] motorPowers = {0,0,0,0};
    public double kP = 0.000001;
    public double kV = 0.0000006338;
    public double kA = 0.000012;
    double velocityConstant = 3.0;
    double lookaheadDistance = 200;
    Coordinate startingCoordinate = new Coordinate(0,0,0);
    Position p1 = new Position(500,0);
    Position p2 = new Position(600,600);

    Position[] positions = {startingCoordinate.getPos(), p2};
    @Override
    public void init() {
        robot = new WHSRobotImpl(hardwareMap);
        strafe1 = new StrafeToTarget(kP, kV,kA,positions, 80, .7, 0.01, velocityConstant, lookaheadDistance, 400);
    }

    @Override
    public void loop() {
        robot.deadWheelEstimatePosition();
        robot.estimateHeading();
        robot.deadWheelPickup.setPosition(DeadWheelPickup.DeadWheelPickupPosition.DOWN);
        motorPowers = strafe1.calculateMotorPowers(robot.getCoordinate(), robot.intake.getWheelVelocities());
        robot.drivetrain.operate(motorPowers);
        telemetry.addData("Angle To Lookahead Debug", strafe1.angleToLookaheadPointDebug);
        telemetry.addData("x", robot.getCoordinate().getX());
        telemetry.addData("y", robot.getCoordinate().getY());
        telemetry.addData("heading", robot.getCoordinate().getHeading());
        telemetry.addData("lookahead point", strafe1.lookaheadPoint.getX() + ", " + strafe1.lookaheadPoint.getY());

//        telemetry.addData("Target Left Velcoities", strafe1.getCurrentTargetWheelVelocities()[0]);
//        telemetry.addData("Target Right Velcoities ", swerve1.getCurrentTargetWheelVelocities()[1]);
        telemetry.addData("Current Velocities Left", robot.drivetrain.getAllWheelVelocities()[0] +
                robot.drivetrain.getAllWheelVelocities()[3]);
        telemetry.addData("Current Velocities Right", robot.drivetrain.getAllWheelVelocities()[1]);
        telemetry.addData("Front Left Wheel Power", motorPowers[0]);
        telemetry.addData("Front Right Wheel Power", motorPowers[1]);
        telemetry.addData("Back Left Wheel Power", motorPowers[2]);
        telemetry.addData("Back Right wheel Power", motorPowers[3]);



    }
}
