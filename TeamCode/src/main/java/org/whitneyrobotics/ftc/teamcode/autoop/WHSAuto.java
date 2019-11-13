package org.whitneyrobotics.ftc.teamcode.autoop;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.whitneyrobotics.ftc.teamcode.lib.util.Coordinate;
import org.whitneyrobotics.ftc.teamcode.lib.util.Position;
import org.whitneyrobotics.ftc.teamcode.lib.util.SimpleTimer;
import org.whitneyrobotics.ftc.teamcode.lib.util.SwerveConstants;
import org.whitneyrobotics.ftc.teamcode.lib.util.SwerveToTarget;
import org.whitneyrobotics.ftc.teamcode.subsys.FoundationPuller;
import org.whitneyrobotics.ftc.teamcode.subsys.Intake;
import org.whitneyrobotics.ftc.teamcode.subsys.SkystoneGrabber;
import org.whitneyrobotics.ftc.teamcode.subsys.WHSRobotImpl;

import java.util.List;

@Autonomous(name = "WHS Auto", group = "auto")
public class WHSAuto extends OpMode {
    WHSRobotImpl robot;

    static final int FOUNDATION = 0;
    static final int SKYSTONE = 1;
    static final int RED = 0;
    static final int BLUE = 1;
    static final int INSIDE = 0;
    static final int OUTSIDE = 1;

    /**
     * Starting X Guidelines
     * Robot center to edge: 9 inches = 228 mm
     */

    static final int STARTING_POSITION = FOUNDATION;
    static final int STARTING_ALLIANCE = BLUE;
    static final int SKYBRIDGE_CROSSING_POSITION = OUTSIDE;
    static final double STARTING_COORDINATE_X = 1200;
    static final boolean PARTNER_MOVED_FOUNDATION = true;

    static final int LEFT = 0;
    static final int CENTER = 1;
    static final int RIGHT = 2;

    int skystonePosition = CENTER;

    Coordinate[] startingCoordinateArray = new Coordinate[2];
    Position[][] skystonePositionArray = new Position[2][6];
    Position[] foundationMovedPositionArray = new Position[2];
    Position[] foundationStartingPositionArray = new Position[2];
    Position[] slideOutFromFoundationMidpointArray = new Position[2];
    Position[][] skybridgePositionArray = new Position[2][2];

    SwerveToTarget startToFoundationSwerve;
    SwerveToTarget foundationToWallSwerve;
    SwerveToTarget wallToSkystoneSwerve;
    SwerveToTarget startToSkystoneSwerve;
    SwerveToTarget skystoneToMovedFoundationSwerve;
    SwerveToTarget skystoneToUnmovedFoundationSwerve;
    SwerveToTarget movedFoundationToParkSwerve;
    SwerveToTarget movedFoundationToSecondSkystoneSwerve;
    SwerveToTarget secondSkystonetoMovedFoundationSwerve;
    SwerveToTarget wallToParkSwerve;

    boolean isStrafing = false;
    /**
     * werve to target instantiations
     */
    private void instantiateSwerveToTargets() {
        Position[] startToFoundationSwervePositions = {startingCoordinateArray[STARTING_ALLIANCE], foundationStartingPositionArray[STARTING_ALLIANCE]};
        Position[] foundationToWallSwervePositions = {foundationStartingPositionArray[STARTING_ALLIANCE], startingCoordinateArray[STARTING_ALLIANCE]};
        Position[] wallToSkystoneSwervePositions = {startingCoordinateArray[STARTING_ALLIANCE], slideOutFromFoundationMidpointArray[STARTING_ALLIANCE], skybridgePositionArray[STARTING_ALLIANCE][SKYBRIDGE_CROSSING_POSITION], skystonePositionArray[STARTING_ALLIANCE][3]};
        Position[] startToSkystoneSwervePositions = {startingCoordinateArray[STARTING_ALLIANCE], skystonePositionArray[STARTING_ALLIANCE][skystonePosition]};
        Position[] movedFoundationToParkingSwervePositions = {foundationMovedPositionArray[STARTING_ALLIANCE], skybridgePositionArray[STARTING_ALLIANCE][SKYBRIDGE_CROSSING_POSITION]};
        Position[] wallToParkingSwervePositions = {startingCoordinateArray[STARTING_ALLIANCE], skybridgePositionArray[STARTING_ALLIANCE][SKYBRIDGE_CROSSING_POSITION]};

        startToFoundationSwerve = new SwerveToTarget(SwerveConstants.StartToFoundationSwerveConstants.kP,
                SwerveConstants.StartToFoundationSwerveConstants.kV,
                SwerveConstants.StartToFoundationSwerveConstants.kA,
                startToFoundationSwervePositions,
                80,
                .5,
                SwerveConstants.StartToFoundationSwerveConstants.velocityConstant,
                SwerveConstants.StartToFoundationSwerveConstants.lookaheadDistance);

        foundationToWallSwerve = new SwerveToTarget(SwerveConstants.FoundationToWallSwerveConstants.kP,
                SwerveConstants.FoundationToWallSwerveConstants.kV,
                SwerveConstants.FoundationToWallSwerveConstants.kA,
                foundationToWallSwervePositions,
                80,
                .5,
                SwerveConstants.FoundationToWallSwerveConstants.velocityConstant,
                SwerveConstants.FoundationToWallSwerveConstants.lookaheadDistance);

        wallToSkystoneSwerve = new SwerveToTarget(SwerveConstants.WallToSkystoneSwerveConstants.kP,
                SwerveConstants.WallToSkystoneSwerveConstants.kV,
                SwerveConstants.WallToSkystoneSwerveConstants.kA,
                wallToSkystoneSwervePositions,
                80,
                .8,
                SwerveConstants.WallToSkystoneSwerveConstants.velocityConstant,
                SwerveConstants.WallToSkystoneSwerveConstants.lookaheadDistance);

        startToSkystoneSwerve = new SwerveToTarget(SwerveConstants.StartToSkystoneSwerveConstants.kP,
                SwerveConstants.StartToSkystoneSwerveConstants.kV,
                SwerveConstants.StartToSkystoneSwerveConstants.kA,
                startToSkystoneSwervePositions,
                80,
                .5,
                SwerveConstants.StartToSkystoneSwerveConstants.velocityConstant,
                SwerveConstants.StartToSkystoneSwerveConstants.lookaheadDistance);

        movedFoundationToParkSwerve = new SwerveToTarget(SwerveConstants.MovedFoundationToParkingSwerveConstants.kP,
                SwerveConstants.MovedFoundationToParkingSwerveConstants.kV,
                SwerveConstants.MovedFoundationToParkingSwerveConstants.kA,
                movedFoundationToParkingSwervePositions,
                80,
                .5,
                SwerveConstants.MovedFoundationToParkingSwerveConstants.velocityConstant,
                SwerveConstants.MovedFoundationToParkingSwerveConstants.lookaheadDistance);

        wallToParkSwerve = new SwerveToTarget(SwerveConstants.WallToParkingSwerveConstants.kP,
                SwerveConstants.WallToParkingSwerveConstants.kV,
                SwerveConstants.WallToParkingSwerveConstants.kA,
                wallToParkingSwervePositions,
                80,
                .5,
                SwerveConstants.WallToParkingSwerveConstants.velocityConstant,
                SwerveConstants.WallToParkingSwerveConstants.lookaheadDistance);
    }

    /**
     * State Definitions
     */
    static final int INIT = 0;
    static final int INITIAL_MOVE_FOUNDATION = 1;
    static final int SCAN_SKYSTONE = 2;
    static final int GRAB_SKYSTONE = 3;
    static final int DRIVE_TO_FOUNDATION = 4;
    static final int OUTTAKE_SKYSTONE = 5;
    static final int SECONDARY_MOVE_FOUNDATION = 6;
    static final int GRAB_SECOND_SKYSTONE = 7;
    static final int PARK = 8;
    static final int END = 9;

    static final int NUM_OF_STATES = 10;

    boolean[] stateEnabled = new boolean[NUM_OF_STATES];

    int state = INIT;
    int subState = 0;
    String stateDesc;
    String subStateDesc;

    /**
     * Advances the state, skipping ones that have been disabled.
     */
    public void advanceState() {
        if (stateEnabled[(state + 1)]) {
            state++;
            subState = 0;
        } else {
            state++;
            advanceState();
        }
    }

    /**
     * Determines which states will be run.
     */
    public void defineStateEnabledStatus() {
        stateEnabled[INIT] = true;
        stateEnabled[INITIAL_MOVE_FOUNDATION] = (STARTING_POSITION == FOUNDATION);
        stateEnabled[SCAN_SKYSTONE] = true;
        stateEnabled[GRAB_SKYSTONE] = true;
        stateEnabled[DRIVE_TO_FOUNDATION] = true;
        stateEnabled[OUTTAKE_SKYSTONE] = true;
        stateEnabled[SECONDARY_MOVE_FOUNDATION] = (STARTING_POSITION == SKYSTONE && !PARTNER_MOVED_FOUNDATION);
        stateEnabled[GRAB_SECOND_SKYSTONE] = true;
        stateEnabled[PARK] = true;
        stateEnabled[END] = true;
    }

    /**
     * Timers
     */
    SimpleTimer foundationPullerUpToDownTimer = new SimpleTimer();
    SimpleTimer foundationPullerDownToUpTimer = new SimpleTimer();
    SimpleTimer scanSkystoneTimer = new SimpleTimer();
    SimpleTimer deadmanStrafeToSkystoneTimer = new SimpleTimer();
    SimpleTimer deadmanStrafeFromSkystoneTimer = new SimpleTimer();
    SimpleTimer grabSkystoneTimer = new SimpleTimer();
    SimpleTimer outtakeSkystoneTimer = new SimpleTimer();
    SimpleTimer moveSkystoneGrabberTimer = new SimpleTimer();

    private static final double STRAFE_TO_SKYSTONE_POWER = 0.7542;

    private final double GRAB_FOUNDATION_DELAY = 1.0;
    private final double SCAN_SKYSTONE_DURATION = 1.0;
    private final double STRAFE_SKYSTONE_TIME = 0.8;
    private final double GRAB_SKYSTONE_DELAY = 0.8;
    private final double OUTTAKE_SKYSTONE_DELAY = 1.0;
    private final double MOVE_SKYSTONE_GRABBER_DELAY = 1.0;

    double[] motorPowers;

    /*Skystone Detection Tensorflow Variables*/
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    private static final String VUFORIA_KEY =
            "AWYX8QX/////AAABmQ8w8KJJuEsNlO9fxNmHDg1BoH/L5lzniFIDqLd+XlCF9gXWlYeddle27IIm9DH8mtLY2CLX9LW3uAzD8IH5Stmf+NoLjfm+m4jnj7KmR+v+xGuUEgP3Aj8sez5uhtsKarKiv94URMVnf39sjHW3xhiUBI30M762Ee6bEy69ZHQSOHLNxMwm9lnETo0O13vhmtZvI44HtEjIvXbW71p/+jdZw/33i6q//G4O3h5Ej+MQ3UCgUe9ERfh9L/v/lgLmekgYdFNaUZi8C1z+O4Jb/8MbHmpJ4Hu9XtA8pI2MLZMRNgOrnFwgXTukIhyHhJ2/2wVi6gwfyxzkJMU27jyGY/gLX9NtjwqhL4NaMWG6t/6m";
    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    private void initVuforia() {

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
    private void detectSkystonePosition(){
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                    if (recognition.getLabel() == LABEL_SECOND_ELEMENT ){

                    }
                }

                telemetry.update();
            }
        }
    }
    @Override
    public void init() {
        robot = new WHSRobotImpl(hardwareMap);
        defineStateEnabledStatus();

        startingCoordinateArray[RED] = new Coordinate(STARTING_COORDINATE_X, -1571, -90);
        startingCoordinateArray[BLUE] = new Coordinate(STARTING_COORDINATE_X, 1571,  90);

        // 0 = next to wall
        skystonePositionArray[RED][0] = new Position(-1088, -606);
        skystonePositionArray[RED][1] = new Position(-885,-606);
        skystonePositionArray[RED][2] = new Position(-682,-606);
        skystonePositionArray[RED][3] = new Position(-1088, -606);
        skystonePositionArray[RED][4] = new Position(-885,-606);
        skystonePositionArray[RED][5] = new Position(-682,-606);

        skystonePositionArray[BLUE][0] = new Position(-1088,606);
        skystonePositionArray[BLUE][1] = new Position(-885,606);
        skystonePositionArray[BLUE][2] = new Position(-682,606);
        skystonePositionArray[BLUE][3] = new Position(-1073,1300);
        skystonePositionArray[BLUE][4] = new Position(-870,1300);
        skystonePositionArray[BLUE][5] = new Position(-667,1300);

        foundationStartingPositionArray[RED] = new Position(STARTING_COORDINATE_X, -780);
        foundationStartingPositionArray[BLUE] = new Position(STARTING_COORDINATE_X,760);

        foundationMovedPositionArray[RED] = new Position(1024,-1108);
        foundationMovedPositionArray[BLUE] = new Position(1024, 1108);

        slideOutFromFoundationMidpointArray[RED] = new Position(600, -1571);
        slideOutFromFoundationMidpointArray[BLUE] = new Position(600, 1571);

        skybridgePositionArray[RED][INSIDE] = new Position(0,-1000);
        skybridgePositionArray[RED][OUTSIDE] = new Position(0,-1450);

        skybridgePositionArray[BLUE][INSIDE] = new Position(0,1000);
        skybridgePositionArray[BLUE][OUTSIDE] = new Position(0,1450);

        instantiateSwerveToTargets();
    }

    @Override
    public void loop() {
        if (!isStrafing) {
            robot.estimateHeading();
            robot.estimatePosition();
        }
        switch (state) {
            case INIT:
                stateDesc = "Starting auto";
                robot.setInitialCoordinate(startingCoordinateArray[STARTING_ALLIANCE]);
                advanceState();
                break;
            case INITIAL_MOVE_FOUNDATION:
                stateDesc = "Moving Foundation";
                switch (subState) {
                    case 0:
                        subStateDesc = "Entry";
                        subState++;
                        break;
                    case 1:
                        subStateDesc = "Driving to foundation";
                        motorPowers = startToFoundationSwerve.calculateMotorPowers(robot.getCoordinate(), robot.drivetrain.getWheelVelocities(), true);
                        robot.drivetrain.operate(motorPowers);
                        if (!startToFoundationSwerve.inProgress()) {
                            foundationPullerUpToDownTimer.set(GRAB_FOUNDATION_DELAY);
                            subState++;
                        }
                        break;
                    case 2:
                        subStateDesc = "Grabbing foundation";
                        robot.foundationPuller.setFoundationPullerPosition(FoundationPuller.PullerPosition.DOWN);
                        if (foundationPullerUpToDownTimer.isExpired()) {
                            subState++;
                        }
                        break;
                    case 3:
                        subStateDesc = "Driving to wall";
                        motorPowers = foundationToWallSwerve.calculateMotorPowers(robot.getCoordinate(), robot.drivetrain.getWheelVelocities(), false);
                        robot.drivetrain.operate(motorPowers);
                        if (!foundationToWallSwerve.inProgress()) {
                            foundationPullerDownToUpTimer.set(GRAB_FOUNDATION_DELAY);
                            subState++;
                        }
                        break;
                    case 4:
                        subStateDesc = "Releasing foundation";
                        //nrobot.foundationPuller.setFoundationPullerPosition(FoundationPuller.PullerPosition.UP);
                        if (foundationPullerDownToUpTimer.isExpired()) {
                            robot.intake.setIntakePusherPosition(Intake.IntakePusherPosition.UP);
                            subState++;
                        }
                        break;
                    case 5:
                        subStateDesc = "Exit";
                        advanceState();
                        break;
                }
                break;
            case SCAN_SKYSTONE:
                stateDesc = "Scanning skystone";
                switch (subState) {
                    case 0:
                        subStateDesc = "Entry";
                        moveSkystoneGrabberTimer.set(MOVE_SKYSTONE_GRABBER_DELAY);
                        subState++;
                        break;
                    case 1:
                        subStateDesc = "Driving to skystone";
                        if (STARTING_POSITION == SKYSTONE) {
                            motorPowers = startToSkystoneSwerve.calculateMotorPowers(robot.getCoordinate(), robot.drivetrain.getWheelVelocities(), false);
                        } else {
                            if (180 - Math.abs(robot.getCoordinate().getHeading()) < 30) {
                                robot.foundationPuller.setFoundationPullerPosition(FoundationPuller.PullerPosition.UP);
                            }

                            if (moveSkystoneGrabberTimer.isExpired()) {
                                robot.skystoneGrabber.setPosition(SkystoneGrabber.SkystoneGrabberPosition.REST);
                            }
                            motorPowers = wallToSkystoneSwerve.calculateMotorPowers(robot.getCoordinate(), robot.drivetrain.getWheelVelocities(), false);
                        }
                        /*if (robot.getCoordinate().getX() < -450) {
                            robot.intake.setMotorPowers(Intake.INTAKE_POWER);
                            //robot.outtake.hover();
                        }*/
                        robot.drivetrain.operate(motorPowers);
                        if (!startToSkystoneSwerve.inProgress() && !wallToSkystoneSwerve.inProgress()) {
                            subState++;
                        }
                        break;
                    case 2:
                        subStateDesc = "Rotating";
                        robot.rotateToTarget(180, false);
                        if (!robot.rotateToTargetInProgress()) {
                            scanSkystoneTimer.set(SCAN_SKYSTONE_DURATION);
                            subState++;
                        }
                    case 3:
                        subStateDesc = "Scanning first skystone";
                        if (robot.vuforia.skystoneDetected()) {
                            skystonePosition = 3;
                            subState = 7;
                        }
                        if (scanSkystoneTimer.isExpired()) {
                            subState++;
                        }
                        break;
                    case 4:
                        subStateDesc = "Driving to second stone";
                        robot.driveToTarget(skystonePositionArray[STARTING_ALLIANCE][4], true);
                        if (!robot.driveToTargetInProgress() && !robot.rotateToTargetInProgress()) {
                            scanSkystoneTimer.set(SCAN_SKYSTONE_DURATION);
                            subState++;
                        }
                        break;
                    case 5:
                        subStateDesc = "Scanning second skystone";
                        if (robot.vuforia.skystoneDetected()) {
                            skystonePosition = 4;
                            subState = 7;
                        }
                        if (scanSkystoneTimer.isExpired()) {
                            subState++;
                        }
                        break;
                    case 6:
                        subStateDesc = "Driving to third skystone";
                        robot.driveToTarget(skystonePositionArray[STARTING_ALLIANCE][5], true);
                        if (!robot.driveToTargetInProgress() && !robot.rotateToTargetInProgress()) {
                            skystonePosition = 5;
                            subState++;
                        }
                        break;
                    case 7:
                        subStateDesc = "Exit";
                        Position[] skystoneToMovedFoundationSwervePositions = {skystonePositionArray[STARTING_ALLIANCE][skystonePosition], skybridgePositionArray[STARTING_ALLIANCE][SKYBRIDGE_CROSSING_POSITION], foundationMovedPositionArray[STARTING_ALLIANCE]};
                        Position[] skystoneToUnmovedFoundationSwervePositions = {skystonePositionArray[STARTING_ALLIANCE][skystonePosition], skybridgePositionArray[STARTING_ALLIANCE][SKYBRIDGE_CROSSING_POSITION], foundationStartingPositionArray[STARTING_ALLIANCE]};
                        skystoneToMovedFoundationSwerve = new SwerveToTarget(SwerveConstants.SkystoneToMovedFoundationSwerveConstants.kP,
                                SwerveConstants.SkystoneToMovedFoundationSwerveConstants.kV,
                                SwerveConstants.SkystoneToMovedFoundationSwerveConstants.kA,
                                skystoneToMovedFoundationSwervePositions,
                                80,
                                0.8,
                                SwerveConstants.SkystoneToMovedFoundationSwerveConstants.velocityConstant,
                                SwerveConstants.SkystoneToMovedFoundationSwerveConstants.lookaheadDistance);

                        skystoneToUnmovedFoundationSwerve = new SwerveToTarget(SwerveConstants.SkystoneToUnmovedFoundationSwerveConstants.kP,
                                SwerveConstants.SkystoneToUnmovedFoundationSwerveConstants.kV,
                                SwerveConstants.SkystoneToUnmovedFoundationSwerveConstants.kA,
                                skystoneToUnmovedFoundationSwervePositions,
                                80,
                                0.8,
                                SwerveConstants.SkystoneToUnmovedFoundationSwerveConstants.velocityConstant,
                                SwerveConstants.SkystoneToUnmovedFoundationSwerveConstants.lookaheadDistance);

                        advanceState();
                        break;
                }
                break;
            case GRAB_SKYSTONE:
                stateDesc = "Grabbing Skystone";
                switch (subState) {
                    case 0:
                        subStateDesc = "Entry";
                        deadmanStrafeToSkystoneTimer.set(STRAFE_SKYSTONE_TIME);
                        isStrafing = true;
                        //robot.drivetrain.operateMecanumDrive(-STRAFE_TO_SKYSTONE_POWER, 0.0, 0.0, 0.0 /*<- Don't care about heading*/);
                        subState++;
                        break;
                    case 1:
                        subStateDesc = "Moving to skystone";
                        robot.drivetrain.operateMecanumDrive(-STRAFE_TO_SKYSTONE_POWER, 0.0, 0.0, 0.0 /*<- Don't care about heading*/);
                        if (deadmanStrafeToSkystoneTimer.isExpired()) {
                            robot.drivetrain.operate(0.0, 0.0);
                            grabSkystoneTimer.set(GRAB_SKYSTONE_DELAY);
                            subState++;
                        }

                        break;
                    case 2:
                        subStateDesc = "Grabbing Skystone";
                        robot.skystoneGrabber.setPosition(SkystoneGrabber.SkystoneGrabberPosition.DOWN);
                        if(grabSkystoneTimer.isExpired()){
                            deadmanStrafeFromSkystoneTimer.set(STRAFE_SKYSTONE_TIME);
                            //robot.drivetrain.operateMecanumDrive(STRAFE_TO_SKYSTONE_POWER, 0.0, 0.0, 0.0 /*<- Don't care about heading*/);
                            subState++;
                        }
                        break;
                    case 3:
                        subStateDesc = "Driving away from Skystone";
                        robot.drivetrain.operateMecanumDrive(STRAFE_TO_SKYSTONE_POWER, 0.0, 0.0, 0.0 /*<- Don't care about heading*/);
                        if(deadmanStrafeFromSkystoneTimer.isExpired()){
                            robot.setInitialCoordinate(new Coordinate(skystonePositionArray[STARTING_ALLIANCE][skystonePosition], 180));
                            isStrafing = false;
                            robot.drivetrain.operate(0.0, 0.0);
                            subState++;
                        }
                        break;
                    case 4:
                        subStateDesc = "Exit";
                        advanceState();
                        break;
                }
                break;
            case DRIVE_TO_FOUNDATION:
                stateDesc = "Driving to foundation";
                switch (subState) {
                    case 0:
                        subStateDesc = "Entry";
                        subState++;
                        break;
                    case 1:
                        subStateDesc = "Driving to foundation";
                        //robot.outtake.grabStone();
                        if (stateEnabled[SECONDARY_MOVE_FOUNDATION]) {
                            motorPowers = skystoneToUnmovedFoundationSwerve.calculateMotorPowers(robot.getCoordinate(), robot.drivetrain.getWheelVelocities(), true);
                        } else {
                            motorPowers = skystoneToMovedFoundationSwerve.calculateMotorPowers(robot.getCoordinate(), robot.drivetrain.getWheelVelocities(), true);
                        }
                        robot.drivetrain.operate(motorPowers);
                        if (!skystoneToUnmovedFoundationSwerve.inProgress() && !skystoneToMovedFoundationSwerve.inProgress()) {
                            subState++;
                        }
                        break;
                    case 2:
                        subStateDesc = "Exit";
                        advanceState();
                        break;
                }
                break;
            case OUTTAKE_SKYSTONE:
                stateDesc = "Outtaking skystone";
                switch (subState) {
                    case 0:
                        subStateDesc = "Entry";
                        outtakeSkystoneTimer.set(OUTTAKE_SKYSTONE_DELAY);
                        subState++;
                        break;
                    case 1:
                        subStateDesc = "Outtaking skystone";
                        robot.skystoneGrabber.setPosition(SkystoneGrabber.SkystoneGrabberPosition.REST);
                        //robot.outtake.autoOuttake(1);
                        if (outtakeSkystoneTimer.isExpired()) {
                            subState++;
                        }
                        break;
                    case 2:
                        subStateDesc = "Exit";
                        advanceState();
                        break;
                }
                break;
            case SECONDARY_MOVE_FOUNDATION:
                stateDesc = "Secondary moving foundation";
                switch (subState) {
                    case 0:
                        subStateDesc = "Entry";
                        subState++;
                        foundationPullerUpToDownTimer.set(GRAB_FOUNDATION_DELAY);
                        break;
                    case 1:
                        subStateDesc = "Grabbing foundation";
                        //robot.foundationPuller.setFoundationPullerPosition(FoundationPuller.PullerPosition.DOWN);
                        if (foundationPullerUpToDownTimer.isExpired()) {
                            subState++;
                        }
                        break;
                    case 2:
                        subStateDesc = "Moving foundation";
                        motorPowers = foundationToWallSwerve.calculateMotorPowers(robot.getCoordinate(), robot.drivetrain.getWheelVelocities(), false);
                        robot.drivetrain.operate(motorPowers);
                        if (!foundationToWallSwerve.inProgress()) {
                            foundationPullerDownToUpTimer.set(GRAB_FOUNDATION_DELAY);
                            subState++;
                        }
                        break;
                    case 3:
                        subStateDesc = "Releasing foundation";
                        //robot.foundationPuller.setFoundationPullerPosition(FoundationPuller.PullerPosition.UP);
                        if (foundationPullerDownToUpTimer.isExpired()) {
                            subState++;
                        }
                        break;
                    case 4:
                        subStateDesc = "Exit";
                        advanceState();
                        break;
                }
                break;
            case GRAB_SECOND_SKYSTONE
                switch (subState){
                    case 0:
                        subStateDesc ="initialzing new swerves";
                        Position[] movedFoundationToSecondSkystonePosition = {foundationMovedPositionArray[STARTING_ALLIANCE], skybridgePositionArray[STARTING_ALLIANCE][SKYBRIDGE_CROSSING_POSITION], skystonePositionArray[STARTING_ALLIANCE][(skystonePosition - 3)]};
                        movedFoundationToSecondSkystoneSwerve = new SwerveToTarget(SwerveConstants.SkystoneToUnmovedFoundationSwerveConstants.kP,
                            SwerveConstants.SkystoneToUnmovedFoundationSwerveConstants.kV,
                            SwerveConstants.SkystoneToUnmovedFoundationSwerveConstants.kA,
                            movedFoundationToSecondSkystonePosition,
                            80,
                            0.8,
                            SwerveConstants.SkystoneToUnmovedFoundationSwerveConstants.velocityConstant,
                            SwerveConstants.SkystoneToUnmovedFoundationSwerveConstants.lookaheadDistance);
                        subState++;
                        break;
                    case 1:
                        subStateDesc = "driving to second skystone";
                        motorPowers = movedFoundationToSecondSkystoneSwerve.calculateMotorPowers(robot.getCoordinate(), robot.drivetrain.getWheelVelocities(), false);
                        if (!movedFoundationToSecondSkystoneSwerve.inProgress()){
                            subState++;
                        }
                        break;
                    case 2: subStateDesc ="rotating";
                        robot.rotateToTarget(180, false);
                        if (!robot.rotateToTargetInProgress()){
                            subState++;
                        }
                        break;
                    case 3:
                        subStateDesc = "Setting Timer";
                        deadmanStrafeToSkystoneTimer.set(STRAFE_SKYSTONE_TIME);
                        isStrafing = true;
                        //robot.drivetrain.operateMecanumDrive(-STRAFE_TO_SKYSTONE_POWER, 0.0, 0.0, 0.0 /*<- Don't care about heading*/);
                        subState++;
                        break;
                    case 4:
                        subStateDesc = "Moving to skystone";
                        robot.drivetrain.operateMecanumDrive(-STRAFE_TO_SKYSTONE_POWER, 0.0, 0.0, 0.0 /*<- Don't care about heading*/);
                        if (deadmanStrafeToSkystoneTimer.isExpired()) {
                            robot.drivetrain.operate(0.0, 0.0);
                            grabSkystoneTimer.set(GRAB_SKYSTONE_DELAY);
                            subState++;
                        }

                        break;
                    case 5:
                        subStateDesc = "Grabbing Skystone";
                        robot.skystoneGrabber.setPosition(SkystoneGrabber.SkystoneGrabberPosition.DOWN);
                        if(grabSkystoneTimer.isExpired()){
                            deadmanStrafeFromSkystoneTimer.set(STRAFE_SKYSTONE_TIME);
                            //robot.drivetrain.operateMecanumDrive(STRAFE_TO_SKYSTONE_POWER, 0.0, 0.0, 0.0 /*<- Don't care about heading*/);
                            subState++;
                        }
                        break;
                    case 6:
                        subStateDesc = "Driving away from Skystone";
                        robot.drivetrain.operateMecanumDrive(STRAFE_TO_SKYSTONE_POWER, 0.0, 0.0, 0.0 /*<- Don't care about heading*/);
                        if(deadmanStrafeFromSkystoneTimer.isExpired()){
                            robot.setInitialCoordinate(new Coordinate(skystonePositionArray[STARTING_ALLIANCE][skystonePosition], 180));
                            isStrafing = false;
                            robot.drivetrain.operate(0.0, 0.0);
                            subState++;
                        }
                        break;
                    case 7:
                        subStateDesc ="Initializing Swerve to Foundation";
                        Position[] secondSkystonetoMovedFoundationPositons = {skystonePositionArray[STARTING_ALLIANCE][skystonePosition - 3]};
                        secondSkystonetoMovedFoundationSwerve =  new SwerveToTarget(SwerveConstants.SkystoneToUnmovedFoundationSwerveConstants.kP,
                                SwerveConstants.SkystoneToUnmovedFoundationSwerveConstants.kV,
                                SwerveConstants.SkystoneToUnmovedFoundationSwerveConstants.kA,
                                secondSkystonetoMovedFoundationPositons,
                                80,
                                0.8,
                                SwerveConstants.SkystoneToUnmovedFoundationSwerveConstants.velocityConstant,
                                SwerveConstants.SkystoneToUnmovedFoundationSwerveConstants.lookaheadDistance);
                        subState++;
                        break;
                    case 8:
                        subStateDesc ="Swerving to foundation and letting go"
                        motorPowers = secondSkystonetoMovedFoundationSwerve.calculateMotorPowers(robot.getCoordinate(), robot.drivetrain.getWheelVelocities(), true);
                        if (!secondSkystonetoMovedFoundationSwerve.inProgress()){
                            robot.skystoneGrabber.setPosition(SkystoneGrabber.SkystoneGrabberPosition.REST);
                            subState++;
                        }
                        break;
                    case 9:
                        subStateDesc = "Exit";
                        advanceState();
                        break;
                }
            case PARK:
                stateDesc = "Parking";
                switch (subState) {
                    case 0:
                        subStateDesc = "Entry";
                        subState++;
                        break;
                    case 1:
                        subStateDesc = "Parking";
                        if (stateEnabled[SECONDARY_MOVE_FOUNDATION]) {
                            motorPowers = wallToParkSwerve.calculateMotorPowers(robot.getCoordinate(), robot.drivetrain.getWheelVelocities(), false);
                        } else {
                            motorPowers = movedFoundationToParkSwerve.calculateMotorPowers(robot.getCoordinate(), robot.drivetrain.getWheelVelocities(),false);
                        }
                        robot.drivetrain.operate(motorPowers);
                        if (!wallToParkSwerve.inProgress() && !movedFoundationToParkSwerve.inProgress()) {
                            subState++;
                        }
                        break;
                    case 2:
                        subStateDesc = "Exit";
                        advanceState();
                        break;
                }
                break;
            case END:
                break;
            default:
                break;
        }

        telemetry.addData("State: ", stateDesc);
        telemetry.addData("Substate: ", subStateDesc);
        telemetry.addData("IMU", robot.imu.getHeading());
        telemetry.addData("X", robot.getCoordinate().getX());
        telemetry.addData("Y", robot.getCoordinate().getY());
        telemetry.addData("Heading", robot.getCoordinate().getHeading());
    }
}
