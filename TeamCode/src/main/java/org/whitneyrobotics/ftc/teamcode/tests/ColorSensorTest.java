package org.whitneyrobotics.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
@Autonomous (name = "Color Sensor Test")
public class ColorSensorTest extends OpMode {
    ColorSensor leftColorSensor;
    ColorSensor rightColorSensor;
    @Override
    public void init() {
        leftColorSensor = hardwareMap.colorSensor.get("leftColorSensor");
        rightColorSensor = hardwareMap.colorSensor.get("rightColorSensor");

    }

    @Override
    public void loop() {

        telemetry.addData("Right,How Red?", rightColorSensor.red());
        telemetry.addData("Right, How Green?", rightColorSensor.green());
        telemetry.addData("Right, How Blue?", rightColorSensor.blue());
        telemetry.addData("Right, How Alpha?", rightColorSensor.alpha());
        telemetry.addData("Right, How Hue?", rightColorSensor.argb());

        telemetry.addData("Left,How Red?", leftColorSensor.red());
        telemetry.addData("Left, How Green?", leftColorSensor.green());
        telemetry.addData("Left, How Blue?", leftColorSensor.blue());
        telemetry.addData("Left, How Alpha?", leftColorSensor.alpha());
        telemetry.addData("Left, How Hue?", leftColorSensor.argb());
    }
}
