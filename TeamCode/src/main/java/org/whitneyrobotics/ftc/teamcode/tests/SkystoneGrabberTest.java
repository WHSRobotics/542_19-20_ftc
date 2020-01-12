package org.whitneyrobotics.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.whitneyrobotics.ftc.teamcode.subsys.SkystoneGrabber;
@TeleOp(name = "Skystone grabber tests")
public class SkystoneGrabberTest extends OpMode {

    SkystoneGrabber skystoneGrabber;

    @Override
    public void init() {
       skystoneGrabber = new SkystoneGrabber(hardwareMap);
    }

    @Override
    public void loop() {
       skystoneGrabber.operate(gamepad1.dpad_up, gamepad1.dpad_down);
    }
}
