package org.whitneyrobotics.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.whitneyrobotics.ftc.teamcode.subsys.WHSRobotTele;

@Autonomous(name = "autoOuttake Test")
public class AutoOuttakeTest extends OpMode {

    WHSRobotTele robot;
    boolean firstLoopCycle = true;

    @Override
    public void init() {
        robot = new WHSRobotTele(hardwareMap);
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        if (firstLoopCycle) {
            robot.outtake.grabStone();
            firstLoopCycle = false;
        }
        robot.foundationPuller.operate(false);
        robot.outtake.autoOuttake();
    }
}
