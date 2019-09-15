package org.whitneyrobotics.ftc.teamcode.tests;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@TeleOp(name = "distance sensor test")
public class DistanceSensorTest extends OpMode {
    private DistanceSensor distanceSensor;
    @Override
    public void init() {
        // you can use this as a regular DistanceSensor.
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) distanceSensor;    }

    @Override
    public void loop() {
        telemetry.addData("Distance Sensor Output", distanceSensor.getDistance(DistanceUnit.MM));
    }
}
