package org.whitneyrobotics.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.whitneyrobotics.ftc.teamcode.subsys.FoundationPuller;
import org.whitneyrobotics.ftc.teamcode.subsys.NewGrabber;
import org.whitneyrobotics.ftc.teamcode.subsys.NewOuttake;
@TeleOp (name = "New Grabber Test")
public class NewGrabberTest extends OpMode {
    NewOuttake outtake;
    FoundationPuller foundationPuller;
    @Override
    public void init() {
        outtake = new NewOuttake(hardwareMap);
        foundationPuller  = new FoundationPuller(hardwareMap);
    }

    @Override
    public void loop() {
        outtake.operate(gamepad1.y, gamepad1.a, gamepad1.dpad_up, gamepad1.dpad_down, gamepad2.right_trigger > 0.01);
        foundationPuller.operate(true);
    }
}
