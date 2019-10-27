package org.whitneyrobotics.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.whitneyrobotics.ftc.teamcode.lib.util.Toggler;
import org.whitneyrobotics.ftc.teamcode.subsys.Extension;

@TeleOp(name = "Extension.outtake test", group = "Tests")
public class GrabberTest extends OpMode {
    Extension extension;

    @Override
    public void init() {
        extension = new Extension(hardwareMap);
    }

    @Override
    public void loop() {
       // extension.operateOuttake(gamepad1.y);
    }
}
