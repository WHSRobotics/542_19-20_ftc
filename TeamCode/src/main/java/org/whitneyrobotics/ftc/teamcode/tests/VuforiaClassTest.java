package org.whitneyrobotics.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.whitneyrobotics.ftc.teamcode.subsys.Vuforia;
@Autonomous(name = "Vuforia Class Test")
public class VuforiaClassTest extends OpMode {
    Vuforia vuforia;
    @Override
    public void init() {
        vuforia = new Vuforia(hardwareMap);
    }

    @Override
    public void loop() {
        telemetry.addData("Skystone Detected:", vuforia.skystoneDetected());
    }
}
