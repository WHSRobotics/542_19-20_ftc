package org.whitneyrobotics.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.whitneyrobotics.ftc.teamcode.lib.subsys.robot.WHSRobot;
import org.whitneyrobotics.ftc.teamcode.subsys.WHSRobotImpl;
@Autonomous(name = "neddy test")
public class neddyTest extends OpMode {

    WHSRobotImpl robot;
    WHSRobotImpl sneddy;

    @Override
    public void init() {
        robot = new WHSRobotImpl(hardwareMap);
        sneddy = new WHSRobotImpl(hardwareMap);

    }

    @Override
    public void loop() {
        robot.estimatePosition();
        robot.estimateHeading();
        sneddy.estimateCoordinate();
        sneddy.drivetrain.operate(-gamepad1.left_stick_y, -gamepad1.right_stick_y);

        telemetry.addData("x", robot.getCoordinate().getX());
        telemetry.addData("y", robot.getCoordinate().getY());
        telemetry.addData("Heading", robot.getCoordinate().getHeading());
        telemetry.addData("Sneddy X", sneddy.getCoordinate().getX());
        telemetry.addData("Sneddy Y", sneddy.getCoordinate().getY());
        telemetry.addData("SneddyHeading", sneddy.getCoordinate().getHeading());
        telemetry.addData("FL Position", robot.drivetrain.frontLeft.getCurrentPosition());
        telemetry.addData("BL Position", robot.drivetrain.backLeft.getCurrentPosition());
        telemetry.addData("FR Position", robot.drivetrain.frontRight.getCurrentPosition());
        telemetry.addData("BR Position", robot.drivetrain.backRight.getCurrentPosition());
        telemetry.addData("X heading: ", robot.imu.getThreeHeading()[0]);
        telemetry.addData("Y heading: ", robot.imu.getThreeHeading()[1]);
        telemetry.addData("Z heading: ", robot.imu.getThreeHeading()[2]);

    }
}
