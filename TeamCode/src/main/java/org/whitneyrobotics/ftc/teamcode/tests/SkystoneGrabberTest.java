package org.whitneyrobotics.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.whitneyrobotics.ftc.teamcode.subsys.SkystoneGrabber;

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
