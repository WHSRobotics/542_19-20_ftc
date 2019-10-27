package org.whitneyrobotics.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.whitneyrobotics.ftc.teamcode.lib.util.Toggler;
import org.whitneyrobotics.ftc.teamcode.subsys.Extension;
import org.whitneyrobotics.ftc.teamcode.subsys.Grabber;

@TeleOp(name = "EncoderPositionTest")
public class EncoderPositionTest extends OpMode {

    Extension extension;
    Grabber grabber;

    int pendingTargetPosition = 0;
    int targetPosition = 0;
    Toggler setTargetPosition = new Toggler(2);
    int lastSetTargetPositionState = 0;
    int i = 0;

    Toggler upTog = new Toggler(2);
    int lastUpTogState = 0;
    Toggler downTog = new Toggler(2);
    int lastDownTogState = 0;

    @Override
    public void init() {
        extension = new Extension(hardwareMap);
        grabber = new Grabber(hardwareMap);
    }

    @Override
    public void loop() {
        upTog.changeState(gamepad1.b);
        downTog.changeState(gamepad1.a);
        setTargetPosition.changeState(gamepad1.dpad_right);

        if (upTog.currentState() != lastUpTogState) pendingTargetPosition += 5;
        if (downTog.currentState() != lastDownTogState) pendingTargetPosition -= 5;
        lastUpTogState = upTog.currentState();
        lastDownTogState = downTog.currentState();

        i++;
        if (i%5 == 0) {
            if (gamepad1.y) pendingTargetPosition += 20;
            if (gamepad1.x) pendingTargetPosition -= 20;
        }

        if (setTargetPosition.currentState() != lastSetTargetPositionState) {
            targetPosition = pendingTargetPosition;
        }
        lastSetTargetPositionState = setTargetPosition.currentState();

        extension.setTargetEncoderPosition(targetPosition);

        grabber.cyclePositions(gamepad1.dpad_up, gamepad1.dpad_down);

        telemetry.addData("Target Position", pendingTargetPosition);
        telemetry.addData("Current Position", targetPosition);
    }
}
