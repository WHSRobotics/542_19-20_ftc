package org.whitneyrobotics.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.whitneyrobotics.ftc.teamcode.subsys.Drivetrain;

@TeleOp(name= "Demo Program")
public class DemoDrivetrain extends OpMode {
    Drivetrain demoRobot;
    @Override
    public void init() {
        demoRobot = new Drivetrain(hardwareMap);
    }

    @Override
    public void loop() {
        demoRobot.operate(-gamepad1.left_stick_y/4, -gamepad1.right_stick_y/4);
    }
}
