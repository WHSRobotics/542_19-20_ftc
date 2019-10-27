package org.whitneyrobotics.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.whitneyrobotics.ftc.teamcode.subsys.Extension;

public class ExtensionTest extends OpMode {
    Extension extension;

    @Override
    public void init() {
        extension = new Extension(hardwareMap);
    }

    @Override
    public void loop() {
       // extension.operateExtension(gamepad2.dpad_up, gamepad2.dpad_down, gamepad2.a);
    }
}
