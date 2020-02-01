package org.whitneyrobotics.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.whitneyrobotics.ftc.teamcode.subsys.Drivetrain;
import org.whitneyrobotics.ftc.teamcode.subsys.Intake;
import org.whitneyrobotics.ftc.teamcode.subsys.TeleIntake;

@TeleOp (name = "TankIntakeTest")
public class TankIntakeTest extends OpMode {
    Drivetrain drivetrain;
    TeleIntake intake;


    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap);
        intake = new TeleIntake(hardwareMap);
    }

    @Override
    public void loop() {
        drivetrain.operate(gamepad1.left_stick_y, gamepad1.right_stick_y);
        intake.operateIntake(gamepad1.right_trigger > 0.01, gamepad1.right_trigger > 0.01);
    }
}
