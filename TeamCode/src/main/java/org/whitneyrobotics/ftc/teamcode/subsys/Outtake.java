package org.whitneyrobotics.ftc.teamcode.subsys;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.whitneyrobotics.ftc.teamcode.lib.util.SimpleTimer;
import org.whitneyrobotics.ftc.teamcode.lib.util.Toggler;

public class Outtake {

    Extension extension;
    Grabber grabber;

    private static final int CLEARANCE_LEVEL = 3;

    private Toggler operateExtensionToggler = new Toggler(6);
    int operateExtensionSubState = 0;
    private Toggler extensionLevelTog = new Toggler(10);

    private SimpleTimer intakeToOuttakeTimer = new SimpleTimer();
    private double intakeToOuttakeDelay = 2.0;

    public Outtake(HardwareMap outtakeMap) {
        extension = new Extension(outtakeMap);
        grabber = new Grabber(outtakeMap);
    }

    public void operateExtension(boolean gamepadInputUp, boolean gamepadInputDown, boolean gamepadInputExtensionLevelUp, boolean gamepadInputExtensionLevelDown) {
        extension.estimateLevel(); // Finds the closest Extension Level
        operateExtensionToggler.changeState(gamepadInputUp, gamepadInputDown); //Changes the State in the Method
        extensionLevelTog.changeState(gamepadInputExtensionLevelUp, gamepadInputExtensionLevelDown); // Changes the target Extension Level

        switch (operateExtensionToggler.currentState()) {
            case 0: // Wait For Stone
                extension.setLevel(CLEARANCE_LEVEL); //sets the linear slides to the intermediate state where it waits for the stone
                grabber.setPosition(Grabber.GrabberPosition.INTAKE_UP); //Elbow = Intake, Hand = Up, Wrist = Up
                break;
            case 1: // Grab the stone
                extension.setLevel(0); //Brings linear slides all the way down
                if (extension.getCurrentLevel() == 0) {
                    // Brings down the Hand and Wrist Servos if the Extension is all the way down
                    grabber.setPosition(Grabber.GrabberPosition.INTAKE_DOWN);
                }
                operateExtensionSubState = 0;
                break;
            case 2: //Swing around
                switch (operateExtensionSubState) {
                    case 0: //Go To clearance Position
                        extension.setLevel(CLEARANCE_LEVEL);
                        if (extension.getCurrentLevel() == CLEARANCE_LEVEL) {
                            intakeToOuttakeTimer.set(intakeToOuttakeDelay); //Sets a timer for moving the elbow servos
                            operateExtensionSubState++;
                        }
                        break;
                    case 1: // Moves the Elbow Servos
                        grabber.setPosition(Grabber.GrabberPosition.OUTTAKE_UP); //Sets the Grabber to swing out
                        if (extensionLevelTog.currentState() < CLEARANCE_LEVEL) {
                            //If the target position is below the clearance level, then do not go on until the outtake has completely swung around
                            if (intakeToOuttakeTimer.isExpired()) {
                                operateExtensionSubState++;
                            }
                        } else {
                            operateExtensionSubState++;
                        }
                        break;
                    case 2: //Goes to Target Linear Slide Position
                        extension.setLevel(extensionLevelTog.currentState());
                        break;
                }
            case 3: //Makes Stone Parallel
                grabber.setPosition(Grabber.GrabberPosition.OUTTAKE_DOWN);
                operateExtensionSubState = 0;
                break;
            case 4: // Releases Stone
                grabber.setPosition(Grabber.GrabberPosition.OUTTAKE_RELEASED);
                break;
            case 5: // Return extension to hover (clearance) for grabber to swing back in
                extension.setLevel(CLEARANCE_LEVEL);
                if (extension.getCurrentLevel() >= CLEARANCE_LEVEL) {
                    grabber.setPosition(Grabber.GrabberPosition.INTAKE_UP);
                }
                break;
            default:
                break;
        }
    }
}
