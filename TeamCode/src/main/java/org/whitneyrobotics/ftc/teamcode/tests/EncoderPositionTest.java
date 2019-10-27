package org.whitneyrobotics.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.whitneyrobotics.ftc.teamcode.lib.util.Toggler;
@TeleOp(name = "EncoderPositionTest")
public class EncoderPositionTest extends OpMode {

    DcMotor leftMotor;
    DcMotor rightMotor;

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
        leftMotor = hardwareMap.dcMotor.get("leftExtension");
        rightMotor = hardwareMap.dcMotor.get("rightExtension");

        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftMotor.setTargetPosition(pendingTargetPosition);
        rightMotor.setTargetPosition(pendingTargetPosition);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void loop() {
        upTog.changeState(gamepad1.b);
        downTog.changeState(gamepad1.a);
        setTargetPosition.changeState(gamepad1.dpad_up);

        if (upTog.currentState() != lastUpTogState) pendingTargetPosition += 10;
        if (downTog.currentState() != lastDownTogState) pendingTargetPosition -=10;
        lastUpTogState = upTog.currentState();
        lastDownTogState = downTog.currentState();

        i++;
        if (i%10 == 0) {
            if (gamepad1.y) pendingTargetPosition += 10;
            if (gamepad1.x) pendingTargetPosition -= 10;
        }

        if (setTargetPosition.currentState() != lastSetTargetPositionState) {
            targetPosition = pendingTargetPosition;
        }
        lastSetTargetPositionState = setTargetPosition.currentState();

        leftMotor.setTargetPosition(targetPosition);
        rightMotor.setTargetPosition(targetPosition);
        leftMotor.setPower(1.0);
        rightMotor.setPower(1.0);

        telemetry.addData("Target Position", pendingTargetPosition);
        telemetry.addData("Current Position", targetPosition);
    }
}
