package org.whitneyrobotics.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.whitneyrobotics.ftc.teamcode.autoop.WHSAuto;
import org.whitneyrobotics.ftc.teamcode.subsys.FoundationPuller;
import org.whitneyrobotics.ftc.teamcode.subsys.SkystoneGrabber;
import org.whitneyrobotics.ftc.teamcode.subsys.WHSRobotImpl;
import org.whitneyrobotics.ftc.teamcode.subsys.WHSRobotTele;

@TeleOp(name = "WHS TeleOp", group = "a")
public class WHSTeleOp extends OpMode {

    static final int RED = 0;
    static final int BLUE = 1;
    static final int STARTING_ALLIANCE = WHSAuto.STARTING_ALLIANCE;
    WHSRobotTele robot;

    @Override
    public void init() {
        robot = new WHSRobotTele(hardwareMap);
    }

    @Override
    public void loop() {
        // Drivetrain
        robot.estimateHeading();
        robot.drivetrain.switchFieldCentric(gamepad1.b);
        if (gamepad1.left_bumper){
            robot.drivetrain.operateMecanumDrive(gamepad1.left_stick_x/2.54, gamepad1.left_stick_y/2.54, gamepad1.right_stick_x/2.54, robot.getCoordinate().getHeading());
        }else{
            robot.drivetrain.operateMecanumDriveScaled(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, robot.getCoordinate().getHeading());
        }

        // Intake
        if ((!robot.intake.stoneSensed() && robot.outtake.getCurrentState() == 1) || robot.capstone.getCapstoneTogglerState() != 0) {
            robot.intake.operateIntake(gamepad1.right_trigger > 0.01, gamepad1.left_trigger > 0.01);
        }else{
            robot.intake.setMotorPowers(0.0);
        }
        // Outtake
        robot.outtake.operate(gamepad2.y, gamepad2.a, gamepad2.dpad_up, gamepad2.dpad_down/*, gamepad2.dpad_right*/);

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
