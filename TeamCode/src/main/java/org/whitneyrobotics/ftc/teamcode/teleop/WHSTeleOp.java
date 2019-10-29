package org.whitneyrobotics.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.whitneyrobotics.ftc.teamcode.subsys.FoundationPuller;
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
        //Drivetrain
        robot.estimateHeading();
        robot.drivetrain.switchFieldCentric(gamepad1.b);
        if (gamepad1.left_bumper){
            robot.drivetrain.operateMecanumDrive(gamepad1.left_stick_x/2.54, gamepad1.left_stick_y/2.54, gamepad1.right_stick_x/2.54, robot.getCoordinate().getHeading());
        }else{
            robot.drivetrain.operateMecanumDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, robot.getCoordinate().getHeading());
        }

        //Intake
        robot.intake.operateIntake(gamepad1.right_trigger > 0.01, gamepad1.left_trigger >0.01, gamepad1.x, gamepad1.y);

        //Extension
        //robot.extension.operateExtension(gamepad2.dpad_up, gamepad2.dpad_down, gamepad2.x);
        //TODO: Add in stuff for the actual grabber

        //Foundation Puller
        robot.foundationPuller.operate(gamepad1.a);
    }
}
