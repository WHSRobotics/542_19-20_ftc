package org.whitneyrobotics.ftc.teamcode.subsys;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.whitneyrobotics.ftc.teamcode.lib.util.SimpleTimer;
import org.whitneyrobotics.ftc.teamcode.lib.util.Toggler;

public class Capstone {
    Servo intakeBlocker;
    Servo lock;
    Servo dump;
    Toggler capstoneTog = new Toggler(5);

    //up,down
    final double [] INTAKE_BLOCKER_POSITIONS = {0.84, 0.10};
    final double [] LOCK_POSITIONS = {0.58, 0.02};
    final double [] DUMP_POSITIONS = {0.14, 0.97};




    public Capstone(HardwareMap capstoneMap){
        intakeBlocker = capstoneMap.servo.get("capstoneIntakeBlocker");
        lock = capstoneMap.servo.get("capstoneLock");
        dump = capstoneMap.servo.get("capstone");

    }
    public void operate(boolean incrementState, boolean decrementState){
        capstoneTog.changeState(incrementState, decrementState);
        switch (capstoneTog.currentState()){
            case 0:
                intakeBlocker.setPosition(INTAKE_BLOCKER_POSITIONS[0]);
                lock.setPosition(LOCK_POSITIONS[1]);
                dump.setPosition(DUMP_POSITIONS[0]);
                break;
            case 1:
                intakeBlocker.setPosition(INTAKE_BLOCKER_POSITIONS[1]);
                break;
            case 2:
                lock.setPosition(LOCK_POSITIONS[0]);
                break;
            case 3:
                dump.setPosition(DUMP_POSITIONS[1]);
                break;
            case 4:
                dump.setPosition((DUMP_POSITIONS[0]));
                break;
        }


    }

}
