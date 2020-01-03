package org.whitneyrobotics.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.whitneyrobotics.ftc.teamcode.lib.util.Toggler;
import org.whitneyrobotics.ftc.teamcode.subsys.Extension;

@TeleOp(name = "Extension Test", group = "tests")
public class ExtensionTest extends OpMode {
    Extension extension;
    Toggler extendTog = new Toggler(7);


    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    @Override
    public void init() {
        extension=new Extension(hardwareMap);
        TelemetryPacket packet = new TelemetryPacket();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        dashboard.sendTelemetryPacket(packet);
    }

    @Override
    public void loop() {
        extendTog.changeState(gamepad1.dpad_up, gamepad1.dpad_down);
        extension.setLevel(extendTog.currentState());
        telemetry.addData("Extend Tog State", extendTog.currentState());
        telemetry.addData("Current error", extension.getError());
        telemetry.addData("Current Output", extension.getPIDOutput());
    }
}
