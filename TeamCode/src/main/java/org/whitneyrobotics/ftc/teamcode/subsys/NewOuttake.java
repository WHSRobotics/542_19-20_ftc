package org.whitneyrobotics.ftc.teamcode.subsys;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.whitneyrobotics.ftc.teamcode.lib.util.SimpleTimer;
import org.whitneyrobotics.ftc.teamcode.lib.util.Toggler;

public class NewOuttake {

    Extension extension;
    NewGrabber grabber;
    BackGate backGate;

    private static final int HOVER_LEVEL = 3;
    private static final int CLEARANCE_LEVEL = 4;

    private Toggler operateOuttakeToggler = new Toggler(6);
    int operateExtensionSubState = 0;
    private Toggler extensionLevelTog = new Toggler(9);
    private Toggler capstoneTog = new Toggler(2);

    private SimpleTimer intakeToOuttakeTimer = new SimpleTimer();

    private int autoOuttakeState = 0;
    private int autoOuttakeSubState = 0;

    private SimpleTimer teleOuttakeToIntakeTimer = new SimpleTimer();
    private SimpleTimer autoIntakeToOuttakeTimer = new SimpleTimer();
    private SimpleTimer autoReleaseOuttakeTimer = new SimpleTimer();
    private SimpleTimer autoOuttakeToIntakeTimer = new SimpleTimer();
    private SimpleTimer intakeUpToIntakeDownTimer = new SimpleTimer();
    private SimpleTimer resetSlidesTimer = new SimpleTimer();

    boolean autoOuttakeInProgress = false;
    boolean setOuttakeToIntakeTimer = true;

    private double intakeToOuttakeDelay = 1.5;
    private double outtakeToIntakeDelay = 0.75;
    private double releaseOuttakeDelay = 0.15;
    private double grabStoneDeadmanDuration = 0.5;
    private double resetSlidesDelay = 0.5;

    public int debugCount = 0;

    public NewOuttake(HardwareMap outtakeMap) {
        extension = new Extension(outtakeMap);
        grabber = new NewGrabber(outtakeMap);
        backGate = new BackGate(outtakeMap);
        extensionLevelTog.setState(1);
    }

    public void operate(boolean gamepadInputUp, boolean gamepadInputDown, boolean gamepadInputExtensionLevelUp, boolean gamepadInputExtensionLevelDown, boolean gamepadInputBackGate) {
        extension.estimateLevel(); // Finds the closest Extension Level
        operateOuttakeToggler.changeState(gamepadInputUp, gamepadInputDown); //Changes the State in the Method
        extensionLevelTog.changeState(gamepadInputExtensionLevelUp, gamepadInputExtensionLevelDown); // Changes the target Extension Level
        //capstoneTog.changeState(gamepadInputCapstone);

        if (gamepadInputBackGate) {
            backGate.setPosition(BackGate.BackGateServoPosition.DOWN);
            extension.setLevel(3);
        } else {
            backGate.setPosition(BackGate.BackGateServoPosition.UP);
            switch (operateOuttakeToggler.currentState()) {
                case 0:
                    setOuttakeToIntakeTimer = true;
                    if (resetSlidesTimer.isExpired()) {
                        extension.setLevel(0);
                    }
                    grabber.setPosition(NewGrabber.GrabberPosition.INTAKE_DOWN);
                    break;
                case 1: // Wait For Stone
                    extension.setLevel(0);
                    grabber.setPosition(NewGrabber.GrabberPosition.INTAKE_UP); //Elbow = Intake, Hand = Up, Wrist = Up
                    intakeUpToIntakeDownTimer.set(grabStoneDeadmanDuration);
                    break;
                case 2: // Grab the stone
                    extension.setLevel(0); //Brings linear slides all the way down
                    grabber.setPosition(NewGrabber.GrabberPosition.INTAKE_DOWN_RELEASED);
                    if (intakeUpToIntakeDownTimer.isExpired()) {
                        // Brings down the Hand and Wrist Servos if the Extension is all the way down
                        grabber.setPosition(NewGrabber.GrabberPosition.INTAKE_DOWN);
                    }
                    break;
                case 3: //Swing around
                    extension.setHigherLevel(extensionLevelTog.currentState());
                    grabber.setPosition(NewGrabber.GrabberPosition.OUTTAKE_DOWN); //Sets the Grabber to swing out
                    break;
                case 4://hovering over the stone
                    extension.setLevel(extensionLevelTog.currentState());
                    grabber.setPosition(NewGrabber.GrabberPosition.OUTTAKE_DOWN);
                    break;
                case 5: // Releases Stone
                    setOuttakeToIntakeTimer = true;
                    grabber.setPosition(NewGrabber.GrabberPosition.OUTTAKE_RELEASED);
                    extension.setLevel(extensionLevelTog.currentState());
                    resetSlidesTimer.set(resetSlidesDelay);
                    break;
                default:
                    break;
            }
        }
    }
    public void setOperateOuttakeTogglerState(int targetState){
        extension.estimateLevel(); // Finds the closest Extension Level
        operateOuttakeToggler.setState(targetState);
        switch (operateOuttakeToggler.currentState()) {
            case 0:
                setOuttakeToIntakeTimer = true;
                extension.setLevel(0);
                if (capstoneTog.currentState() == 0) {
                    grabber.setPosition(NewGrabber.GrabberPosition.INTAKE_DOWN);
                } /*else {
                    grabber.setPosition(NewGrabber.GrabberPosition.CAPSTONE_INTAKE_UP);
                }*/
                break;
            case 1: // Wait For Stone
                extension.setLevel(HOVER_LEVEL); //sets the linear slides to the intermediate state where it waits for the stone
                if (extension.getCurrentLevel() >= HOVER_LEVEL) {
                    if (capstoneTog.currentState() == 0) {
                        grabber.setPosition(NewGrabber.GrabberPosition.INTAKE_UP); //Elbow = Intake, Hand = Up, Wrist = Up
                    } /*else {
                        grabber.setPosition(NewGrabber.GrabberPosition.CAPSTONE_INTAKE_UP);
                    }*/
                }
                break;
            case 2: // Grab the stone
                extension.setLevel(0); //Brings linear slides all the way down
                if (extension.getCurrentLevel() == 0) {
                    // Brings down the Hand and Wrist Servos if the Extension is all the way down
                    if (capstoneTog.currentState() == 0) {
                        grabber.setPosition(NewGrabber.GrabberPosition.INTAKE_DOWN);
                    } /*else {
                        grabber.setPosition(NewGrabber.GrabberPosition.CAPSTONE_INTAKE_DOWN);
                    }*/
                }
                operateExtensionSubState = 0;
                break;
            case 3: //Swing around
                switch (operateExtensionSubState) {
                    case 0: //Go To clearance Position
                        if (capstoneTog.currentState() == 0) {
                            grabber.setPosition(NewGrabber.GrabberPosition.INTAKE_DOWN);
                        } /*else {
                            grabber.setPosition(NewGrabber.GrabberPosition.CAPSTONE_INTAKE_DOWN);
                        }*/
                        extension.setLevel(CLEARANCE_LEVEL);
                        if (extension.getCurrentLevel() == CLEARANCE_LEVEL) {
                            intakeToOuttakeTimer.set(intakeToOuttakeDelay); //Sets a timer for moving the elbow servos
                            operateExtensionSubState++;
                        }
                        break;
                    case 1: // Moves the Elbow Servos
                        grabber.setPosition(NewGrabber.GrabberPosition.OUTTAKE_DOWN); //Sets the Grabber to swing out
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
                        grabber.setPosition(NewGrabber.GrabberPosition.OUTTAKE_DOWN); //Sets the Grabber to swing out
                        if(extensionLevelTog.currentState()!= extensionLevelTog.howManyStates()-1) {
                            extension.setLevel(extensionLevelTog.currentState());
                        }else{
                            extension.setLevel(extensionLevelTog.currentState());
                        }
                        break;
                    default:
                        break;
                }
                break;
            case 4://hovering over the stone
                extension.setLevel(extensionLevelTog.currentState());
                break;
            case 5: // Releases Stone
                setOuttakeToIntakeTimer = true;
                grabber.setPosition(NewGrabber.GrabberPosition.OUTTAKE_RELEASED);
                extension.setLevel(extensionLevelTog.currentState());
                break;
            case 6:
                grabber.setPosition(NewGrabber.GrabberPosition.OUTTAKE_RELEASED);
                if (extensionLevelTog.currentState() != extensionLevelTog.howManyStates()-1) {
                    extension.setLevel(extensionLevelTog.currentState());
                }else{
                    extension.setLevel(extensionLevelTog.currentState());
                }
                break;
            case 7:
                extension.setLevel(CLEARANCE_LEVEL);
                if (extension.getCurrentLevel() >= CLEARANCE_LEVEL) {
                    if (capstoneTog.currentState() == 0) {
                        grabber.setPosition(NewGrabber.GrabberPosition.INTAKE_UP); //Elbow = Intake, Hand = Up, Wrist = Up
                    } /*else {
                        grabber.setPosition(NewGrabber.GrabberPosition.CAPSTONE_INTAKE_UP);
                    }*/
                    if (setOuttakeToIntakeTimer) {
                        teleOuttakeToIntakeTimer.set(outtakeToIntakeDelay);
                        setOuttakeToIntakeTimer = false;
                    }
                    if (teleOuttakeToIntakeTimer.isExpired()) {
                        operateOuttakeToggler.setState(0);
                    }
                }
                break;
            default:
                break;
        }
    }
    public void autoOperate(boolean gamepadInputUp, boolean gamepadInputDown, boolean gamepadInputExtensionLevelUp, boolean gamepadInputExtensionLevelDown/*, boolean gamepadInputCapstone*/) {
        extension.estimateLevel(); // Finds the closest Extension Level
        operateOuttakeToggler.changeState(gamepadInputUp, gamepadInputDown); //Changes the State in the Method
        extensionLevelTog.changeState(gamepadInputExtensionLevelUp, gamepadInputExtensionLevelDown); // Changes the target Extension Level
        //capstoneTog.changeState(gamepadInputCapstone);

        switch (operateOuttakeToggler.currentState()) {
            case 0: // Wait For Stone
                extension.setLevel(HOVER_LEVEL); //sets the linear slides to the intermediate state where it waits for the stone
                if (extension.getCurrentLevel() >= HOVER_LEVEL) {
                    if (capstoneTog.currentState() == 0) {
                        grabber.setPosition(NewGrabber.GrabberPosition.INTAKE_UP); //Elbow = Intake, Hand = Up, Wrist = Up
                    } /*else {
                        grabber.setPosition(NewGrabber.GrabberPosition.CAPSTONE_INTAKE_UP);
                    }*/
                }
                break;
            case 1: // Grab the stone
                extension.setLevel(0); //Brings linear slides all the way down
                if (extension.getCurrentLevel() == 0) {
                    // Brings down the Hand and Wrist Servos if the Extension is all the way down
                    if (capstoneTog.currentState() == 0) {
                        grabber.setPosition(NewGrabber.GrabberPosition.INTAKE_DOWN);
                    } /*else {
                        grabber.setPosition(NewGrabber.GrabberPosition.CAPSTONE_INTAKE_DOWN);
                    }*/
                }
                operateExtensionSubState = 0;
                break;
            case 2: //Swing around
                switch (operateExtensionSubState) {
                    case 0: //Go To clearance Position
                        if (capstoneTog.currentState() == 0) {
                            grabber.setPosition(NewGrabber.GrabberPosition.INTAKE_DOWN);
                        } /*else {
                            grabber.setPosition(NewGrabber.GrabberPosition.CAPSTONE_INTAKE_DOWN);
                        }*/
                        extension.setLevel(CLEARANCE_LEVEL);
                        if (extension.getCurrentLevel() == CLEARANCE_LEVEL) {
                            intakeToOuttakeTimer.set(intakeToOuttakeDelay); //Sets a timer for moving the elbow servos
                            operateExtensionSubState++;
                        }
                        break;
                    case 1: // Moves the Elbow Servos
                        grabber.setPosition(NewGrabber.GrabberPosition.OUTTAKE_DOWN); //Sets the Grabber to swing out
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
                        grabber.setPosition(NewGrabber.GrabberPosition.OUTTAKE_DOWN); //Sets the Grabber to swing out
                        extension.setLevel(extensionLevelTog.currentState());
                        break;
                    default:
                        break;
                }
                break;
            case 3: // Releases Stone
                setOuttakeToIntakeTimer = true;
                grabber.setPosition(NewGrabber.GrabberPosition.OUTTAKE_RELEASED);
                extension.setLevel(extensionLevelTog.currentState());
                break;
            case 4:
                extension.setLevel(CLEARANCE_LEVEL);
                if (extension.getCurrentLevel() >= CLEARANCE_LEVEL) {
                    if (capstoneTog.currentState() == 0) {
                        grabber.setPosition(NewGrabber.GrabberPosition.INTAKE_UP); //Elbow = Intake, Hand = Up, Wrist = Up
                    } /*else {
                        grabber.setPosition(NewGrabber.GrabberPosition.CAPSTONE_INTAKE_UP);
                    }*/
                    if (setOuttakeToIntakeTimer) {
                        teleOuttakeToIntakeTimer.set(outtakeToIntakeDelay);
                        setOuttakeToIntakeTimer = false;
                    }
                    if (teleOuttakeToIntakeTimer.isExpired()) {
                        operateOuttakeToggler.setState(5);
                    }
                }
                break;
            case 5:
                setOuttakeToIntakeTimer = true;
                extension.setLevel(0);
                grabber.setPosition(NewGrabber.GrabberPosition.INTAKE_DOWN);
                break;
            default:
                break;
        }
    }

    public void hover() {
        extension.estimateLevel();
        grabber.setPosition(NewGrabber.GrabberPosition.INTAKE_UP);
        intakeUpToIntakeDownTimer.set(grabStoneDeadmanDuration);
    }

    public void grabStone() {
        extension.estimateLevel();
        if (intakeUpToIntakeDownTimer.isExpired()) {
            grabber.setPosition(NewGrabber.GrabberPosition.INTAKE_DOWN);
        } else {
            grabber.setPosition(NewGrabber.GrabberPosition.INTAKE_DOWN_RELEASED);
        }
        intakeToOuttakeTimer.set(intakeToOuttakeDelay);
    }

    public void autoOuttake() {
        switch (autoOuttakeState) {
            case 0: //Swing around
                autoOuttakeInProgress = true;
                grabber.setPosition(NewGrabber.GrabberPosition.OUTTAKE_DOWN); //Sets the Grabber to swing out
                if (intakeToOuttakeTimer.isExpired()) {
                    autoOuttakeState++;
                }
                autoReleaseOuttakeTimer.set(releaseOuttakeDelay);
                break;
            case 1: // Releases Stone
                grabber.setPosition(NewGrabber.GrabberPosition.OUTTAKE_RELEASED);
                if (autoReleaseOuttakeTimer.isExpired()) {
                    autoOuttakeState++;
                }
                break;
            case 2: // Swing grabber back into robot
                grabber.setPosition(NewGrabber.GrabberPosition.INTAKE_DOWN_RELEASED);
                if (autoOuttakeToIntakeTimer.isExpired()) {
                    autoOuttakeState++;
                }
                break;
            case 3:
                autoOuttakeInProgress = false;
                break;
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

    public boolean autoOuttakeInProgress() {
        return autoOuttakeInProgress;
    }

    public void changeExtensionErrorBias(boolean gamepadInputUp, boolean gamepadInputDown) {
        if (gamepadInputUp) {
            extension.errorBias += 5;
        }
        if (gamepadInputDown) {
            extension.errorBias -= 5;
        }
    }

    public int[] getExtensionEncoderPositions() {
        return extension.getEncoderPositions();
    }

    public int getExtensionErrorBias() {
        return extension.errorBias;
    }

}
