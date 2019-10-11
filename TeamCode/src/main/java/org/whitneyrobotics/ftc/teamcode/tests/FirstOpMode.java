package org.whitneyrobotics.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class FirstOpMode extends OpMode {

    DcMotor motor1;
    Servo servo1;

    @Override
    public void init() {
        motor1 = hardwareMap.dcMotor.get("motor");

        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        servo1 = hardwareMap.servo.get("servo");

    }

    @Override
    public void loop() {

        if(gamepad1.a){
            motor1.setPower(1.0);
        }
        else if (gamepad1.dpad_left){
            motor1.setPower(0.5);
        } else if (gamepad1.right_bumper) {
            motor1.setPower(-.25);
        }
        else if(gamepad1.right_trigger > 0.01) {
            motor1.setPower(0.3);
        } else {
            motor1.setPower(gamepad1.left_stick_x);
        }

        telemetry.addData("Motor Encoder Ticks", motor1.getCurrentPosition());

        servo1.setPosition(1.0);

    }
}
