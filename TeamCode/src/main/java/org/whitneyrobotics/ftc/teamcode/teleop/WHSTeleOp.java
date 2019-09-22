package org.whitneyrobotics.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.robot.Robot;

import org.whitneyrobotics.ftc.teamcode.subsys.WHSRobotImpl;
@TeleOp(name = "WHS TeleOp")
public class WHSTeleOp extends OpMode {
    WHSRobotImpl robot;
    @Override
    public void init() {
        robot = new WHSRobotImpl(hardwareMap);
    }

    @Override
    public void loop() {
        robot.estimateHeading();
        robot.drivetrain.operateMecanumDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x,robot.getCoordinate().getHeading());
        robot.intake.operateIntake(gamepad1.right_bumper, gamepad1.left_bumper);
    }
}
