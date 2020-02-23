package org.whitneyrobotics.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.whitneyrobotics.ftc.teamcode.lib.subsys.robot.WHSRobot;
import org.whitneyrobotics.ftc.teamcode.subsys.WHSRobotImpl;
@Autonomous(name = "neddy test")
public class neddyTest extends OpMode {

    WHSRobotImpl sneddy;

    @Override
    public void init() {
        sneddy = new WHSRobotImpl(hardwareMap);

    }

    @Override
    public void loop() {
        sneddy.estimateCoordinate();

        telemetry.addData("Sneddy X", sneddy.getCoordinate().getX());
        telemetry.addData("Sneddy Y", sneddy.getCoordinate().getY());


    }
}
