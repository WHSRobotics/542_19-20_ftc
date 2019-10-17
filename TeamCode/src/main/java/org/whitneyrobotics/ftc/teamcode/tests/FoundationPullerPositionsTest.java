package org.whitneyrobotics.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.whitneyrobotics.ftc.teamcode.lib.util.Toggler;
import org.whitneyrobotics.ftc.teamcode.subsys.Extension;

@TeleOp(name = "Foundation.Puller.Positions Test", group = "Tests")
public class FoundationPullerPositionsTest extends OpMode {

    Servo leftServo;
    Servo rightServo;
    double leftServoPos = 0;
    double rightServoPos = 1.0;

    @Override
    public void init() {
        leftServo = hardwareMap.servo.get("leftFoundation");
        rightServo = hardwareMap.servo.get("rightFoundation");
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            leftServoPos += 0.01;
            leftServo.setPosition(leftServoPos);
            rightServoPos -= 0.01;
            rightServo.setPosition(rightServoPos);
        } else if (gamepad1.b) {
            leftServoPos -= 0.01;
            leftServo.setPosition(leftServoPos);
            rightServoPos += 0.01;
            rightServo.setPosition(rightServoPos);
        }
        telemetry.addData("Left Servo Current Pos: ", leftServoPos);
        telemetry.addData("Right Servo Current Pos: ", rightServoPos);
    }
    
}
