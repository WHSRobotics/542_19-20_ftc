package org.whitneyrobotics.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.whitneyrobotics.ftc.teamcode.subsys.Capstone;

@TeleOp(name = "Capstone Test")
public class CapstoneTest extends OpMode {
    Capstone capstone;
    @Override
    public void init() {
        capstone=new Capstone(hardwareMap);
    }

    @Override
    public void loop() {
        capstone.operate(gamepad1.dpad_up, gamepad1.dpad_down);
    }
}
