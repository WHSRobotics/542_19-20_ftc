package org.whitneyrobotics.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.whitneyrobotics.ftc.teamcode.subsys.Drivetrain;
import org.whitneyrobotics.ftc.teamcode.subsys.IMU;

/**
 * Created by ivanm on 11/4/2017.
 */
@Autonomous(name = "IMUTest", group = "tests")
public class IMUTest extends OpMode {

    IMU imu;
    Drivetrain drivetrain;
    double alpha;
    double lastAngularVelocity = 0;
    double lastTime;
    double maxAlpha = 0;

    @Override
    public void init() {
        imu = new IMU(hardwareMap);
        drivetrain = new Drivetrain(hardwareMap);
    }

    @Override
    public void start() {
        lastTime = System.nanoTime() * 1E9;
    }

    @Override
    public void loop() {
        double currentAngularVelocity = imu.getAngularVelocity();
        double currentTime = System.nanoTime()*1E9;
        alpha = (currentAngularVelocity - lastAngularVelocity)/(currentTime - lastTime);
        drivetrain.operateMecanumDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, imu.getHeading());

        lastTime = currentTime;
        lastAngularVelocity = currentAngularVelocity;

        if (Math.abs(alpha) > Math.abs(maxAlpha)) {
            maxAlpha = alpha;
        }

        telemetry.addData("Current Z accel:", alpha);
        telemetry.addData("Max alpha:", maxAlpha);

        /*
        double heading = imu.getHeading();
        double[] threeHeading = imu.getThreeHeading();

        telemetry.addData("Heading: ", heading);
        telemetry.addData("x", threeHeading[0]);
        telemetry.addData("y", threeHeading[1]);
        telemetry.addData("z", threeHeading[2]);
        */
    }
}
