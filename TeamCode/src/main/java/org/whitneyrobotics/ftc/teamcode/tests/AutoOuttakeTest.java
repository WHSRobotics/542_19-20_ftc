package org.whitneyrobotics.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.whitneyrobotics.ftc.teamcode.lib.util.SimpleTimer;
import org.whitneyrobotics.ftc.teamcode.subsys.WHSRobotImpl;
import org.whitneyrobotics.ftc.teamcode.subsys.WHSRobotTele;

@Autonomous(name = "autoOuttake Test")
public class AutoOuttakeTest extends OpMode {

    WHSRobotTele robot;
    SimpleTimer hoverTimer = new SimpleTimer();
    SimpleTimer grabTimer = new SimpleTimer();
    SimpleTimer intakeToOuttakeTimer = new SimpleTimer();
    SimpleTimer releaseStoneTimer = new SimpleTimer();
    SimpleTimer outtakeToIntakeTimer = new SimpleTimer();

    static final double HOVER_DURATION = 3.0;
    static final double GRAB_DURATION = 3.0;
    static final double INTAKE_TO_OUTTAKE_DURATION = 2.0;
    static final double RELEASE_STONE_DURATION = 0.75;
    static final double OUTTAKE_TO_INTAKE_DURATION = 2.0;

    int state = 0;
    boolean gamepadInput = false;

    @Override
    public void init() {
        robot = new WHSRobotTele(hardwareMap);
    }

    @Override
    public void loop()
    {robot.foundationPuller.operate(false);
      switch (state) {
            case 0:
                hoverTimer.set(HOVER_DURATION);
                state++;
                break;
            case 1:
                gamepadInput = false;
                robot.outtake.setOperateOuttakeTogglerState(1);
                if (hoverTimer.isExpired()) {
                    grabTimer.set(GRAB_DURATION);
                    gamepadInput = true;
                    state++;
                }
                break;
            case 2:
                gamepadInput = false;
                robot.outtake.setOperateOuttakeTogglerState(2);
                if (grabTimer.isExpired()) {
                    gamepadInput = true;
                    intakeToOuttakeTimer.set(INTAKE_TO_OUTTAKE_DURATION);
                    state++;
                }
                break;
            case 3:
                gamepadInput = false;
                robot.outtake.setOperateOuttakeTogglerState(3);
                if (intakeToOuttakeTimer.isExpired()) {
                    gamepadInput = true;
                    releaseStoneTimer.set(RELEASE_STONE_DURATION);
                    state++;
                }
                break;
            case 4:
                gamepadInput = false;
                robot.outtake.setOperateOuttakeTogglerState(6);
                if (releaseStoneTimer.isExpired()) {
                    gamepadInput = true;
                    outtakeToIntakeTimer.set(OUTTAKE_TO_INTAKE_DURATION);
                    state++;
                }
                break;
            case 5:
                gamepadInput = false;
                robot.outtake.setOperateOuttakeTogglerState(7);
                if (outtakeToIntakeTimer.isExpired()) {
                    state++;
                }
        }
        telemetry.addData("Debug count", robot.outtake.debugCount);
    }
}
