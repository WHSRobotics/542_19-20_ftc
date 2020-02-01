package org.whitneyrobotics.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.whitneyrobotics.ftc.teamcode.autoop.WHSAuto;
import org.whitneyrobotics.ftc.teamcode.subsys.WHSRobotTele;

@TeleOp(name = "WHS TeleOp", group = "a")
public class WHSTeleOp extends OpMode {

    static final int RED = 0;
    static final int BLUE = 1;
    static final int STARTING_ALLIANCE = WHSAuto.STARTING_ALLIANCE;
    WHSRobotTele robot;
    int i = 0;

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
        if (/*robot.intake.stoneSensed() && */(robot.outtake.getCurrentState() == 1) || robot.capstone.getCapstoneTogglerState() != 0 || gamepad2.left_trigger > 0.01) {
            robot.intake.operateIntake(gamepad1.right_trigger > 0.01, gamepad1.left_trigger > 0.01);
        }else if (gamepad1.right_trigger < 0.01 && gamepad1.left_trigger <0.01){
            robot.intake.setVelocity(0);
        }
        else{
            robot.intake.operateIntake(false, gamepad1.left_trigger>0.01);
        }
        // Outtake
        if (robot.capstone.getCapstoneTogglerState() == 0) {
            robot.outtake.operate(gamepad2.y, gamepad2.a, gamepad2.dpad_up, gamepad2.dpad_down/*, gamepad2.dpad_right*/);
        } else {
            robot.outtake.operate(false, false, false, false);
        }
        robot.outtake.changeExtensionErrorBias(gamepad2.left_stick_y > 0.05, gamepad2.left_stick_y < -0.05);

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

        //
        robot.backGate.operate(gamepad2.right_trigger >0.01);
        telemetry.addData("Outtake State",robot.outtake.getCurrentState());
        telemetry.addData("Target extension level", robot.outtake.getCurrentTargetLevel());
        telemetry.addData("Current extension level", robot.outtake.getCurrentLevel());
        telemetry.addData("Field-centric", robot.drivetrain.getFieldCentric());
        telemetry.addData("Capstone State", robot.capstone.getCapstoneState());
        telemetry.addData("Extension Error Bias", robot.outtake.getExtensionErrorBias());
        telemetry.addData("i", i);
        i++;
    }
}
