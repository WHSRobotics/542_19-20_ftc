package org.whitneyrobotics.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.whitneyrobotics.ftc.teamcode.subsys.Drivetrain;
import org.whitneyrobotics.ftc.teamcode.subsys.Intake;
import org.whitneyrobotics.ftc.teamcode.subsys.WHSRobotImpl;

@TeleOp(name = "Intake Test")
public class IntakeTest extends OpMode {

    WHSRobotImpl robot;

    @Override
    public void init() {

        robot = new WHSRobotImpl(hardwareMap);
    }

    @Override
    public void loop() {
        robot.intake.operateIntake(gamepad1.right_trigger>0.01, gamepad1.left_trigger>0.01);
        robot.drivetrain.operateMecanumDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, robot.getCoordinate().getHeading());
    }
}
