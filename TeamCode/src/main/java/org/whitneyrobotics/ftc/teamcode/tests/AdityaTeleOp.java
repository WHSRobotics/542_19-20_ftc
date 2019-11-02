package org.whitneyrobotics.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Aditya's TeleOp")
public class AdityaTeleOp extends OpMode {
    DcMotor motor1;
    Servo servo1;
    @Override
    public void init() {
        motor1=hardwareMap.dcMotor.get("First Motor");
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        servo1=hardwareMap.servo.get("servo1");
    }

    @Override
    public void loop() {
        if (gamepad1.y) {

            servo1.setPosition(0.1);
            motor1.setPower(gamepad1.right_trigger);
        }
        if(gamepad1.x){

            servo1.setPosition(0.9);
            motor1.setPower(0.9);

        }
        //motor1.setPower(gamepad1.left_stick_y);
    }
}

