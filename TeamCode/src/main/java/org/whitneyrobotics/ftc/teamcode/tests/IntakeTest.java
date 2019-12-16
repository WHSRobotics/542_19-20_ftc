package org.whitneyrobotics.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.whitneyrobotics.ftc.teamcode.lib.util.Toggler;
import org.whitneyrobotics.ftc.teamcode.subsys.Intake;
import org.whitneyrobotics.ftc.teamcode.subsys.WHSRobotImpl;

@TeleOp(name = "Intake Test")
public class IntakeTest extends OpMode {

    WHSRobotImpl robot;
    private Toggler intakePusherTog = new Toggler(2);

    @Override
    public void init() {

        robot = new WHSRobotImpl(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.right_bumper) {
            robot.intake.autoIntakeWithJamDetection(Intake.AUTO_INTAKE_POWER);
        } else {
            robot.intake.operateIntake(gamepad1.right_trigger > 0.01, gamepad1.left_trigger > 0.01);
        }
        robot.drivetrain.operateMecanumDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, robot.getCoordinate().getHeading());

        intakePusherTog.changeState(gamepad1.b);
        robot.intake.setIntakePusherPosition(intakePusherTog.currentState() == 0 ? Intake.IntakePusherPosition.DOWN : Intake.IntakePusherPosition.UP);

        telemetry.addData("Avg Intake Velocity", robot.intake.getAvgIntakeWheelVelocities());
    }
}
