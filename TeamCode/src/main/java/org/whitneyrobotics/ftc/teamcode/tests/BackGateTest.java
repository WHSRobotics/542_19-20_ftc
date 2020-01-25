package org.whitneyrobotics.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.whitneyrobotics.ftc.teamcode.subsys.BackGate;
@TeleOp (name = "Back Gate Test" )
public class BackGateTest extends OpMode {
    public BackGate backGate;
    @Override
    public void init() {
        backGate = new BackGate(hardwareMap);
    }

    @Override
    public void loop() {
        backGate.operate(gamepad2.right_trigger >0.01);
    }
}
