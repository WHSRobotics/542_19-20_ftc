package org.whitneyrobotics.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.whitneyrobotics.ftc.teamcode.subsys.WHSRobotImpl;
@TeleOp(name = "Deadwheel Pickup Test")
public class deadwheelPickupTest extends OpMode {
    WHSRobotImpl robot;
    @Override
    public void init() {
        robot = new WHSRobotImpl(hardwareMap);
    }

    @Override
    public void loop() {
        if(robot.deadwheelRetracted) {
            robot.drivetrain.operateMecanumDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, robot.getCoordinate().getHeading());
        }else{
        robot.retractDeadwheelPickup();}
    }
}
