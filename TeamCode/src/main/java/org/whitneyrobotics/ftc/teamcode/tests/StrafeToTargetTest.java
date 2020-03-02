package org.whitneyrobotics.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.whitneyrobotics.ftc.teamcode.lib.util.Coordinate;
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
    Coordinate p1 = new Coordinate(0,1200, 90);
    Coordinate p2 = new Coordinate(0,1800,0);

    Coordinate[] positions = {startingCoordinate, p1};
    @Override
    public void init() {
        robot = new WHSRobotImpl(hardwareMap);
        strafe1 = new StrafeToTarget(kP, kV,kA,positions, 80, .7, 0.01, velocityConstant, lookaheadDistance, 600, robot.imu.getAngularVelocity());
        telemetry.log().setCapacity(35);
        robot.setInitialCoordinate(startingCoordinate);
    }

    @Override
    public void loop() {
        robot.deadWheelEstimateCoordinate();
        //robot.estimateHeading();
        robot.deadWheelPickup.setPosition(DeadWheelPickup.DeadWheelPickupPosition.DOWN);
        motorPowers = strafe1.calculateMotorPowers(robot.getCoordinate(), robot.intake.getWheelVelocities(), robot.drivetrain.getFrontRightWheelVelocity());
        robot.drivetrain.operate(motorPowers);
        telemetry.addData("Last Closest Heading Point Index", strafe1.lastClosestHeadingIndex);
        telemetry.addData("Condition Met?", strafe1.conditionMet);
        telemetry.addData("Angle To Lookahead Debug", strafe1.angleToLookaheadPointDebug);
        telemetry.addData("x", robot.getCoordinate().getX());
        telemetry.addData("y", robot.getCoordinate().getY());
        telemetry.addData("heading", robot.getCoordinate().getHeading());
        telemetry.addData("lookahead point", strafe1.lookaheadPoint.getX() + ", " + strafe1.lookaheadPoint.getY());

        telemetry.addData("FL Position", robot.drivetrain.frontLeft.getCurrentPosition());
        telemetry.addData("BL Position", robot.drivetrain.backLeft.getCurrentPosition());
        telemetry.addData("FR Position", robot.drivetrain.frontRight.getCurrentPosition());
        telemetry.addData("BR Position", robot.drivetrain.backRight.getCurrentPosition());
        telemetry.addData("X heading: ", robot.imu.getThreeHeading()[0]);
        telemetry.addData("Y heading: ", robot.imu.getThreeHeading()[1]);
        telemetry.addData("Z heading: ", robot.imu.getThreeHeading()[2]);
        telemetry.addData("Left extension", robot.outtake.getExtensionEncoderPositions()[0]);
        telemetry.addData("Right extension", robot.outtake.getExtensionEncoderPositions()[1]);
        telemetry.addData("Left intake Velocity", robot.intake.getWheelVelocities()[0]);
        telemetry.addData("Right intake Velocity", robot.intake.getWheelVelocities()[1]);
        /*for(int i = 0; i < strafe1.smoothedPath.length; i++) {
            telemetry.log().add("Target Angular Velocities " + strafe1.targetAngularVelocities[i]);
        }
        for(int i = 0; i < strafe1.smoothedPath.length; i++){
            telemetry.log().add("Target Headings " + strafe1.smoothedPath[i].getHeading());
        }
        double[] targetAngularVelocities = new double[strafe1.smoothedPath.length];
        for (int i = strafe1.smoothedPath.length-2; i >= 0; i--){
            double deltaTheta = strafe1.smoothedPath[i+1].getHeading() - strafe1.smoothedPath[i].getHeading();
            telemetry.log().add("dTheta" + deltaTheta);
            targetAngularVelocities[i] = (targetAngularVelocities[i+1] * targetAngularVelocities[i+1]) + 2 * 5.0 * deltaTheta;
            telemetry.log().add("tAV " + targetAngularVelocities[i]);
        }*/
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
