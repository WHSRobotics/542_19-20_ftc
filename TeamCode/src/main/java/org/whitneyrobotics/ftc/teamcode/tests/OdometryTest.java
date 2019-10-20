package org.whitneyrobotics.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Odometry Test", group = "tests")
public class OdometryTest extends OpMode {

    DcMotor leftWheel;
    DcMotor rightWheel;

    @Override
    public void init() {
        leftWheel = hardwareMap.dcMotor.get("leftOdometry");
        rightWheel = hardwareMap.dcMotor.get("rightOdometry");
    }

    @Override
    public void loop() {
        telemetry.addData("Left", leftWheel.getCurrentPosition());
        telemetry.addData("Right", rightWheel.getCurrentPosition());
    }
}
