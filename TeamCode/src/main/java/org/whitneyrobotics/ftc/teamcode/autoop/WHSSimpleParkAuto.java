package org.whitneyrobotics.ftc.teamcode.autoop;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.whitneyrobotics.ftc.teamcode.lib.subsys.robot.WHSRobot;
import org.whitneyrobotics.ftc.teamcode.lib.util.SimpleTimer;
import org.whitneyrobotics.ftc.teamcode.subsys.Intake;
import org.whitneyrobotics.ftc.teamcode.subsys.WHSRobotImpl;

@Autonomous(name = "SUPERJANKAUTO")
public class WHSSimpleParkAuto extends OpMode {

    SimpleTimer timer = new SimpleTimer();
    double AUTO_DELAY = 20.0;
    SimpleTimer driveTimer = new SimpleTimer();
    WHSRobotImpl robot;
    int state = 0;
    @Override
    public void init() {
        robot = new WHSRobotImpl(hardwareMap);
    }

    @Override
    public void start(){
        timer.set(AUTO_DELAY);
    }

    @Override
    public void loop() {
        switch (state) {
            case 0:
                if(timer.isExpired()) {
                    driveTimer.set(1.3);
                    robot.drivetrain.operate(0.4, 0.4);
                    robot.intake.setIntakePusherPosition(Intake.IntakePusherPosition.UP);
                    state++;
                }
                break;
            case 1:
                if (driveTimer.isExpired()) {
                    robot.drivetrain.operate(0.0, 0.0);
                }
                break;
        }
    }
}
