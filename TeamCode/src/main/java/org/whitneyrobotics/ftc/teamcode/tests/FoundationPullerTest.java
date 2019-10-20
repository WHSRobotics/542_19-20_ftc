package org.whitneyrobotics.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.whitneyrobotics.ftc.teamcode.subsys.FoundationPuller;
@TeleOp(name = "FoundationPullerTest")
public class FoundationPullerTest extends OpMode {
    FoundationPuller foundationPuller;
    @Override
    public void init() {
        foundationPuller = new FoundationPuller(hardwareMap);
    }

    @Override
    public void loop() {
        foundationPuller.operate(gamepad1.b);
    }
}
