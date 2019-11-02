package org.whitneyrobotics.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.whitneyrobotics.ftc.teamcode.subsys.Drivetrain;
import org.whitneyrobotics.ftc.teamcode.subsys.FoundationPuller;
import org.whitneyrobotics.ftc.teamcode.subsys.IMU;

@TeleOp(name = "FoundationPullerTest")
public class FoundationPullerTest extends OpMode {
    FoundationPuller foundationPuller;
    Drivetrain drivetrain;
    IMU imu;
    @Override
    public void init() {
        foundationPuller = new FoundationPuller(hardwareMap);
        drivetrain = new Drivetrain(hardwareMap);
        imu = new IMU(hardwareMap);
    }

    @Override
    public void loop() {
        foundationPuller.operate(gamepad1.b);
        drivetrain.operateMecanumDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, imu.getHeading());
    }
}
