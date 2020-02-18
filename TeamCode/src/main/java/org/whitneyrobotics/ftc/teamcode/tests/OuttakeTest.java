package org.whitneyrobotics.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.whitneyrobotics.ftc.teamcode.autoop.WHSAuto;
import org.whitneyrobotics.ftc.teamcode.subsys.FoundationPuller;
import org.whitneyrobotics.ftc.teamcode.subsys.SkystoneGrabber;
import org.whitneyrobotics.ftc.teamcode.subsys.WHSRobotImpl;
import org.whitneyrobotics.ftc.teamcode.subsys.WHSRobotTele;

@TeleOp(name = "Outtake Test", group = "a")
public class OuttakeTest extends OpMode {

    WHSRobotTele robot;

    @Override
    public void init() {
        robot = new WHSRobotTele(hardwareMap);
    }

    @Override
    public void loop() {


        // Outtake
        robot.outtake.operate(gamepad2.y, gamepad2.a, gamepad2.dpad_up, gamepad2.dpad_down, gamepad2.right_trigger > 0.01);

        /*//Skystone Grabber
        if (robot.intake.isIntakeOn()) {
            robot.skystoneGrabber.setPosition(SkystoneGrabber.SkystoneGrabberPosition.UP);
        }else {
            robot.skystoneGrabber.operate(gamepad2.x);
        }*/

        //Foundation Puller
        robot.foundationPuller.operate(gamepad1.a);

        //Capstone
        robot.capstone.operate(gamepad2.right_bumper, gamepad2.left_bumper);

        telemetry.addData("Outtake State",robot.outtake.getCurrentState());
        telemetry.addData("Target extension level", robot.outtake.getCurrentTargetLevel());
        telemetry.addData("Current extension level", robot.outtake.getCurrentLevel());
        telemetry.addData("Field-centric", robot.drivetrain.getFieldCentric());
        telemetry.addData("Capstone State", robot.capstone.getCapstoneState());

    }
}