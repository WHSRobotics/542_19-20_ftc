package org.whitneyrobotics.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.whitneyrobotics.ftc.teamcode.lib.util.Toggler;
import org.whitneyrobotics.ftc.teamcode.subsys.Extension;

@TeleOp(name = "Extension Test", group = "tests")
public class ExtensionTest extends OpMode {
    Extension extension;
    Toggler extendTog = new Toggler(7);
    @Override
    public void init() {
        extension=new Extension(hardwareMap);
    }

    @Override
    public void loop() {
        extendTog.changeState(gamepad1.dpad_up, gamepad1.dpad_down);
        extension.setLevel(extendTog.currentState());
        telemetry.addData("Extend Tog State", extendTog.currentState());
    }
}
