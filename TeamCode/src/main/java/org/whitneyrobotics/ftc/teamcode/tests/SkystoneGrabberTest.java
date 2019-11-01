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
        if(gamepad1.a){
            skystoneGrabber.setPosition(SkystoneGrabber.SkystoneGrabberPosition.UP);
        }
        else if(gamepad1.b){
            skystoneGrabber.setPosition(SkystoneGrabber.SkystoneGrabberPosition.DOWN);
        }
    }
}
