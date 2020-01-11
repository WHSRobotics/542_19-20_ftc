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
    final double [] LOCK_POSITIONS = {0.944, 0.465};
    final double [] DUMP_POSITIONS = {0.14, 0.97};

    SimpleTimer unlockToCapstoneDownTimer = new SimpleTimer();
    SimpleTimer capstoneDownToCapstoneUpTimer = new SimpleTimer();
    SimpleTimer capstoneDownToResetTimer = new SimpleTimer();

    private double unlockToCapstonDownDuration = 0.75;
    private double capstoneDownToCapstoneUpDelay = 1.75;
    private double capstondeDownToResetDelay = 0.75;

    private boolean cycledThroughAlready = false;
    String capstoneState = "";

    private int substate = 0;


    public Capstone(HardwareMap capstoneMap){
        intakeBlocker = capstoneMap.servo.get("capstoneIntakeBlocker");
        lock = capstoneMap.servo.get("capstoneLock");
        dump = capstoneMap.servo.get("capstone");

    }
    public void operate(boolean incrementState, boolean decrementState){
        capstoneTog.changeState(incrementState, decrementState);
        switch (capstoneTog.currentState()){
            case 0:
                capstoneState = "Pre Capstone";
                intakeBlocker.setPosition(INTAKE_BLOCKER_POSITIONS[0]);
                lock.setPosition(LOCK_POSITIONS[1]);
                dump.setPosition(DUMP_POSITIONS[0]);
                break;
            case 1:
                capstoneState = "Ready to intake";
                intakeBlocker.setPosition(INTAKE_BLOCKER_POSITIONS[1]);
                unlockToCapstoneDownTimer.set(unlockToCapstonDownDuration);
                substate = 0;
                break;
            case 2:
                switch (substate) {
                    case 0:
                        capstoneState = "Unlocked";
                        lock.setPosition(LOCK_POSITIONS[0]);
                        if (unlockToCapstoneDownTimer.isExpired()){
                            capstoneDownToCapstoneUpTimer.set(capstoneDownToCapstoneUpDelay);
                            substate++;
                        }
                        break;
                    case 1:
                        capstoneState = "Capstone down";
                        dump.setPosition(DUMP_POSITIONS[1]);
                        if (capstoneDownToCapstoneUpTimer.isExpired()){
                            capstoneDownToResetTimer.set(capstondeDownToResetDelay);
                            substate++;
                        }
                        break;
                    case 2:
                        capstoneState = "Capstone dumper up";
                        dump.setPosition((DUMP_POSITIONS[0]));
                        if (capstoneDownToResetTimer.isExpired()){
                            substate = 0;
                            capstoneTog.setState(0);
                        }
                        break;
                }
        }

    }

    public int getCapstoneTogglerState(){
        return capstoneTog.currentState();
    }

    public String getCapstoneState(){
        return capstoneState;
    }

}
