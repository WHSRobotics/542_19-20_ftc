package org.whitneyrobotics.ftc.teamcode.subsys;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.whitneyrobotics.ftc.teamcode.lib.util.SimpleTimer;
import org.whitneyrobotics.ftc.teamcode.lib.util.Toggler;

public class Outtake {

    Extension extension;
    Grabber grabber;

    private static final int CLEARANCE_LEVEL = 4;

    private Toggler operateOuttakeToggler = new Toggler(5);
    int operateExtensionSubState = 0;
    private Toggler extensionLevelTog = new Toggler(7);

    private SimpleTimer intakeToOuttakeTimer = new SimpleTimer();

    private int autoOuttakeState = 0;
    private int autoOuttakeSubState = 0;

    private SimpleTimer autoIntakeToOuttakeTimer = new SimpleTimer();
    private SimpleTimer autoOuttakeUpToOuttakeDownTimer = new SimpleTimer();
    private SimpleTimer autoReleaseOuttakeTimer = new SimpleTimer();
    private SimpleTimer autoOuttakeToIntakeTimer = new SimpleTimer();

    private double intakeToOuttakeDelay = 2.0;
    private double outtakeUpToOuttakeDownDelay = 1.5;
    private double releaseOuttakeDelay = 1.0;

    public Outtake(HardwareMap outtakeMap) {
        extension = new Extension(outtakeMap);
        grabber = new Grabber(outtakeMap);
    }

    public void operate(boolean gamepadInputUp, boolean gamepadInputDown, boolean gamepadInputExtensionLevelUp, boolean gamepadInputExtensionLevelDown) {
        extension.estimateLevel(); // Finds the closest Extension Level
        operateOuttakeToggler.changeState(gamepadInputUp, gamepadInputDown); //Changes the State in the Method
        extensionLevelTog.changeState(gamepadInputExtensionLevelUp, gamepadInputExtensionLevelDown); // Changes the target Extension Level

        switch (operateOuttakeToggler.currentState()) {
            case 0: // Wait For Stone
                extension.setLevel(CLEARANCE_LEVEL); //sets the linear slides to the intermediate state where it waits for the stone
                if (extension.getCurrentLevel() >= CLEARANCE_LEVEL) {
                    grabber.setPosition(Grabber.GrabberPosition.INTAKE_UP); //Elbow = Intake, Hand = Up, Wrist = Up
                }
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
                        grabber.setPosition(Grabber.GrabberPosition.INTAKE_DOWN);
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
                        grabber.setPosition(Grabber.GrabberPosition.OUTTAKE_UP); //Sets the Grabber to swing out
                        extension.setLevel(extensionLevelTog.currentState());
                        break;
                    default:
                        break;
                }
                break;
            case 3: //Makes Stone Parallel
                grabber.setPosition(Grabber.GrabberPosition.OUTTAKE_DOWN);
                extension.setLevel(extensionLevelTog.currentState());
                break;
            case 4: // Releases Stone
                grabber.setPosition(Grabber.GrabberPosition.OUTTAKE_RELEASED);
                extension.setLevel(extensionLevelTog.currentState());
                break;
            default:
                break;
        }
    }

    public void hover() {
        extension.setLevel(CLEARANCE_LEVEL);
        grabber.setPosition(Grabber.GrabberPosition.INTAKE_UP);
    }

    public void grabStone() {
        extension.estimateLevel();
        extension.setLevel(0);
        if (extension.getCurrentLevel() == 0) {
            grabber.setPosition(Grabber.GrabberPosition.INTAKE_DOWN);
        }
    }

    public void autoOuttake(int targetLevel) {
        extension.estimateLevel();

        switch (autoOuttakeState) {
            case 0: //Swing around
                switch (autoOuttakeSubState) {
                    case 0: //Go To clearance Position
                        extension.setLevel(CLEARANCE_LEVEL);
                        if (extension.getCurrentLevel() == CLEARANCE_LEVEL) {
                            autoIntakeToOuttakeTimer.set(intakeToOuttakeDelay); //Sets a timer for moving the elbow servos
                            autoOuttakeSubState++;
                        }
                        break;
                    case 1: // Moves the Elbow Servos
                        grabber.setPosition(Grabber.GrabberPosition.OUTTAKE_UP); //Sets the Grabber to swing out
                        if (extensionLevelTog.currentState() < CLEARANCE_LEVEL) {
                            //If the target position is below the clearance level, then do not go on until the outtake has completely swung around
                            if (autoIntakeToOuttakeTimer.isExpired()) {
                                autoOuttakeSubState++;
                            }
                        } else {
                            autoOuttakeSubState++;
                        }
                        break;
                    case 2: //Goes to Target Linear Slide Position
                        extension.setLevel(targetLevel);
                        if (extension.getCurrentLevel() == targetLevel) {
                            autoOuttakeSubState = 0;
                            autoOuttakeUpToOuttakeDownTimer.set(outtakeUpToOuttakeDownDelay);
                            autoOuttakeState++;
                        }
                        break;
                }
            case 1: //Makes Stone Parallel
                grabber.setPosition(Grabber.GrabberPosition.OUTTAKE_DOWN);
                if (autoOuttakeUpToOuttakeDownTimer.isExpired()) {
                    autoReleaseOuttakeTimer.set(releaseOuttakeDelay);
                    autoOuttakeState++;
                }
                break;
            case 2: // Releases Stone
                grabber.setPosition(Grabber.GrabberPosition.OUTTAKE_RELEASED);
                if (autoReleaseOuttakeTimer.isExpired()) {
                    autoOuttakeState++;
                }
                break;
            case 3: // Set extension to clearance for grabber to swing back in
                extension.setLevel(CLEARANCE_LEVEL);
                if (extension.getCurrentLevel() >= CLEARANCE_LEVEL) {
                    autoOuttakeToIntakeTimer.set(intakeToOuttakeDelay);
                    autoOuttakeState++;
                }
                break;
            case 4: // Swing grabber back into robot
                grabber.setPosition(Grabber.GrabberPosition.INTAKE_UP);
                if (autoOuttakeToIntakeTimer.isExpired()) {
                    autoOuttakeState++;
                }
            case 5:
                extension.setLevel(0);
            default:
                break;
        }
    }

    public int getCurrentLevel() {
        return extension.getCurrentLevel();
    }

    public int getCurrentTargetLevel() {
        return extensionLevelTog.currentState();
    }

    public int getCurrentState() {
        return operateOuttakeToggler.currentState();
    }
}
