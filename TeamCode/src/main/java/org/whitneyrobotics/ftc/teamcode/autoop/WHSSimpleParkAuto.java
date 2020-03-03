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
    double AUTO_DELAY = 1.3;
    WHSRobotImpl robot;
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
        if(!timer.isExpired()) {
            robot.drivetrain.operate(0.4, 0.4);
            robot.intake.setIntakePusherPosition(Intake.IntakePusherPosition.UP);
        }
        else{
            robot.drivetrain.operate(0.0, 0.0);
            robot.intake.setIntakePusherPosition(Intake.IntakePusherPosition.DOWN);

        }
    }
}
