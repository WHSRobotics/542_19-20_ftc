package org.whitneyrobotics.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.whitneyrobotics.ftc.teamcode.subsys.Outtake;
@TeleOp(name="Outtake Test")
public class OuttakeTest extends OpMode {
    Outtake outtake;

    @Override
    public void init() {
        outtake = new Outtake(hardwareMap);
    }

    @Override
    public void loop() {
        outtake.operate(gamepad2.y, gamepad2.a, gamepad2.dpad_up, gamepad2.dpad_down) /*gamepad2.dpad_right)*/;

        telemetry.addData("Current level", outtake.getCurrentLevel());
        telemetry.addData("Current target level", outtake.getCurrentTargetLevel());
        telemetry.addData("Current state", outtake.getCurrentState());
    }
}
