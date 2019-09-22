package org.whitneyrobotics.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.whitneyrobotics.ftc.teamcode.subsys.Drivetrain;
import org.whitneyrobotics.ftc.teamcode.subsys.Intake;
@TeleOp(name = "Intake Test")
public class IntakeTest extends OpMode {

    Intake intake;
    Drivetrain drivetrain;

    @Override
    public void init() {
        intake = new Intake(hardwareMap);
        drivetrain = new Drivetrain(hardwareMap);
    }

    @Override
    public void loop() {
        intake.operateIntake(gamepad1.right_bumper, gamepad1.left_bumper);
    }
}
