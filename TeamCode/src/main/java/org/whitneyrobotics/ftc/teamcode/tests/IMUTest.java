package org.whitneyrobotics.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.whitneyrobotics.ftc.teamcode.subsys.IMU;

/**
 * Created by ivanm on 11/4/2017.
 */
@Autonomous(name = "IMUTest", group = "tests")
public class IMUTest extends OpMode {

    IMU imu;
    double zAccel;
    double maxZAccel = 0;

    @Override
    public void init() {
        imu = new IMU(hardwareMap);
    }

    @Override
    public void loop() {
        zAccel = imu.getZAcceleration();
        if (Math.abs(zAccel) > Math.abs(maxZAccel)) {
            maxZAccel = zAccel;
        }

        telemetry.addData("Current Z accel:", zAccel);
        telemetry.addData("Max Z accel:", maxZAccel);

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
