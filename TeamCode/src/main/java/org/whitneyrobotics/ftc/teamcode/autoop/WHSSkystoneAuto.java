package org.whitneyrobotics.ftc.teamcode.autoop;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import org.whitneyrobotics.ftc.teamcode.lib.util.Alliance;
import org.whitneyrobotics.ftc.teamcode.lib.util.Coordinate;
import org.whitneyrobotics.ftc.teamcode.lib.util.Position;
import org.whitneyrobotics.ftc.teamcode.lib.util.SimpleTimer;
import org.whitneyrobotics.ftc.teamcode.lib.util.SwerveConstants;
import org.whitneyrobotics.ftc.teamcode.lib.util.SwerveToTarget;
import org.whitneyrobotics.ftc.teamcode.subsys.BackGate;
import org.whitneyrobotics.ftc.teamcode.subsys.Capstone;
import org.whitneyrobotics.ftc.teamcode.subsys.DeadWheelPickup;
import org.whitneyrobotics.ftc.teamcode.subsys.FoundationPuller;
import org.whitneyrobotics.ftc.teamcode.subsys.ImprovedSkystoneDetector;
import org.whitneyrobotics.ftc.teamcode.subsys.Intake;
import org.whitneyrobotics.ftc.teamcode.subsys.WHSRobotImpl;

import java.util.List;
import java.util.Locale;

@Autonomous(name = "WHS Skystone Auto", group = "auto")
public class WHSSkystoneAuto extends OpMode {
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

    static final int STARTING_POSITION = SKYSTONE;
    public static final int STARTING_ALLIANCE = RED;
    static final int SKYBRIDGE_CROSSING_POSITION = INSIDE;
    static final double STARTING_COORDINATE_X = -900;
    static final boolean PARTNER_MOVED_FOUNDATION = false;

    static final double LEFT_MAX = 80;
    static final double CENTER_MAX = 165;

    static final int LEFT = 0;
    static final int CENTER = 1;
    static final int RIGHT = 2;

    int skystonePosition = CENTER;

    Coordinate[] startingCoordinateArray = new Coordinate[2];
    Position[][] skystoneMidpointArray = new Position[2][6];
    Position[][] skystonePositionArray = new Position[2][6];
    Position[][] skystoneToFoundationPositionArray = new Position[2][6];
    Position[] foundationMovedPositionArray = new Position[2];
    Position[] foundationStartingPositionArray = new Position[2];
    Position[] slideOutFromFoundationMidpointArray = new Position[2];
    Position[][] skybridgePositionArray = new Position[2][2];
    Position[][] skybridgeSecondCrossingPositionArray = new Position[2][2];
    Position[] skystoneToFoundationMidpointArray = new Position[2];
    Position[] pullFoundationMidpointArray = new Position[2];
    Position[] parkingMidpointArray = new Position[2];
    Position[] parkingPositions = new Position[2];

    SwerveToTarget startToFoundationSwerve;
    SwerveToTarget foundationToWallSwerve;
    SwerveToTarget wallToMidpointSwerve;
    SwerveToTarget midpointToSkystoneSwerve;
    SwerveToTarget startToSkystoneSwerve;
    SwerveToTarget skystoneToMidpointJerkSwerve;
    SwerveToTarget midPointToSkyStoneJerkSwerve;
    SwerveToTarget foundationMidpointToFoundationMidpointTwoSwerve;

    SwerveToTarget skystoneToMovedFoundationSwerve;
    SwerveToTarget skystoneToUnmovedFoundationSwerve;
    SwerveToTarget movedFoundationToParkSwerve;
    SwerveToTarget movedFoundationToSecondSkystoneSwerve;
    SwerveToTarget secondSkystonetoMovedFoundationSwerve;
    SwerveToTarget wallToParkSwerve;
    SwerveToTarget secondSkystoneToJerkPositionSwerve;
    SwerveToTarget startToParkingSwerve;

    boolean isStrafing = false;
    boolean startOuttaking = false;
    boolean operatingFoundationPullers = false;
    String outtakeState = "";

    /**
     * werve to target instantiations
     */
    private void instantiateSwerveToTargets() {
        Position[] startToFoundationSwervePositions = {startingCoordinateArray[STARTING_ALLIANCE], foundationStartingPositionArray[STARTING_ALLIANCE]};
        Position[] foundationToWallSwervePositions = {foundationStartingPositionArray[STARTING_ALLIANCE], foundationMovedPositionArray[STARTING_ALLIANCE]};
        Position[] wallToMidpointPositions = {startingCoordinateArray[STARTING_ALLIANCE], slideOutFromFoundationMidpointArray[STARTING_ALLIANCE]};
        Position[] midpointToSkystonePositions = {skybridgePositionArray[STARTING_ALLIANCE][SKYBRIDGE_CROSSING_POSITION], skystonePositionArray[STARTING_ALLIANCE][2]};
        Position[] movedFoundationToParkingSwervePositions = {pullFoundationMidpointArray[STARTING_ALLIANCE],parkingMidpointArray[STARTING_ALLIANCE], parkingPositions[STARTING_ALLIANCE]};
        Position[] wallToParkingSwervePositions = {startingCoordinateArray[STARTING_ALLIANCE], skybridgePositionArray[STARTING_ALLIANCE][SKYBRIDGE_CROSSING_POSITION]};
        Position[] foundationMidpointToFoundationMidpointTwoPositions = {foundationMovedPositionArray[STARTING_ALLIANCE], pullFoundationMidpointArray[STARTING_ALLIANCE]};
        Position[] startToParkingSwervePositions = {pullFoundationMidpointArray[STARTING_ALLIANCE], parkingPositions[STARTING_ALLIANCE]};

        startToFoundationSwerve = new SwerveToTarget(SwerveConstants.StartToFoundationSwerveConstants.kP,
                SwerveConstants.StartToFoundationSwerveConstants.kV,
                SwerveConstants.StartToFoundationSwerveConstants.kA,
                startToFoundationSwervePositions,
                80,
                .5,
                SwerveConstants.StartToFoundationSwerveConstants.velocityConstant,
                SwerveConstants.StartToFoundationSwerveConstants.lookaheadDistance,
                600);

        foundationToWallSwerve = new SwerveToTarget(SwerveConstants.FoundationToWallSwerveConstants.kP,
                SwerveConstants.FoundationToWallSwerveConstants.kV,
                SwerveConstants.FoundationToWallSwerveConstants.kA,
                foundationToWallSwervePositions,
                80,
                .5,
                SwerveConstants.FoundationToWallSwerveConstants.velocityConstant,
                SwerveConstants.FoundationToWallSwerveConstants.lookaheadDistance,
                800);

        wallToMidpointSwerve = new SwerveToTarget(SwerveConstants.WallToSkystoneSwerveConstants.kP,
                SwerveConstants.WallToSkystoneSwerveConstants.kV,
                SwerveConstants.WallToSkystoneSwerveConstants.kA,
                wallToMidpointPositions,
                80,
                .8,
                SwerveConstants.WallToSkystoneSwerveConstants.velocityConstant,
                SwerveConstants.WallToSkystoneSwerveConstants.lookaheadDistance,
                1000);

        midpointToSkystoneSwerve = new SwerveToTarget(SwerveConstants.WallToSkystoneSwerveConstants.kP,
                SwerveConstants.WallToSkystoneSwerveConstants.kV,
                SwerveConstants.WallToSkystoneSwerveConstants.kA,
                midpointToSkystonePositions,
                80,
                .8,
                SwerveConstants.WallToSkystoneSwerveConstants.velocityConstant,
                SwerveConstants.WallToSkystoneSwerveConstants.lookaheadDistance,
                1000);

        movedFoundationToParkSwerve = new SwerveToTarget(SwerveConstants.MovedFoundationToParkingSwerveConstants.kP,
                SwerveConstants.MovedFoundationToParkingSwerveConstants.kV,
                SwerveConstants.MovedFoundationToParkingSwerveConstants.kA,
                movedFoundationToParkingSwervePositions,
                120,
                .5,
                SwerveConstants.MovedFoundationToParkingSwerveConstants.velocityConstant,
                SwerveConstants.MovedFoundationToParkingSwerveConstants.lookaheadDistance,
                900);

        wallToParkSwerve = new SwerveToTarget(SwerveConstants.WallToParkingSwerveConstants.kP,
                SwerveConstants.WallToParkingSwerveConstants.kV,
                SwerveConstants.WallToParkingSwerveConstants.kA,
                wallToParkingSwervePositions,
                80,
                .5,
                SwerveConstants.WallToParkingSwerveConstants.velocityConstant,
                SwerveConstants.WallToParkingSwerveConstants.lookaheadDistance,
                700);


        startToParkingSwerve = new SwerveToTarget(SwerveConstants.FoundationToWallSwerveConstants.kP,
                SwerveConstants.FoundationToWallSwerveConstants.kV,
                SwerveConstants.FoundationToWallSwerveConstants.kA,
                startToParkingSwervePositions,
                80,
                .9,
                SwerveConstants.FoundationToWallSwerveConstants.velocityConstant,
                SwerveConstants.FoundationToWallSwerveConstants.lookaheadDistance,
                600);
        foundationMidpointToFoundationMidpointTwoSwerve = new SwerveToTarget(SwerveConstants.FoundationToWallSwerveConstants.kP,
                SwerveConstants.FoundationToWallSwerveConstants.kV,
                SwerveConstants.FoundationToWallSwerveConstants.kA,
                foundationMidpointToFoundationMidpointTwoPositions,
                80,
                .9,
                SwerveConstants.FoundationToWallSwerveConstants.velocityConstant,
                SwerveConstants.FoundationToWallSwerveConstants.lookaheadDistance,
                650);
    }

    /**
     * State Definitions
     */
    static final int INIT = 0;
    static final int INITIAL_MOVE_FOUNDATION = 1;
    static final int SCAN_SKYSTONE = 2;
    static final int INTAKE_SKYSTONE = 3;
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
        stateEnabled[INTAKE_SKYSTONE] = true;
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
    SimpleTimer dropIntakeTimer = new SimpleTimer();
    SimpleTimer intakeSystoneTimer = new SimpleTimer();
    SimpleTimer jerkSkystoneTimer = new SimpleTimer();
    SimpleTimer pullFoundationPauseTimer = new SimpleTimer();
    SimpleTimer secondStoneTimer = new SimpleTimer();

    private static final double STRAFE_TO_SKYSTONE_POWER = 0.7542;

    private final double GRAB_FOUNDATION_DELAY = 1.0;
    private final double DROP_INTAKE_DELAY = 1.0;
    private final double SCAN_SKYSTONE_DURATION = 0.0;
    private final double STRAFE_SKYSTONE_TIME = 1.1;
    private final double STRAFE_SECOND_SKYSTONE_TIME = 1.1;
    private final double GRAB_SKYSTONE_DELAY = 1.0;
    private final double OUTTAKE_SKYSTONE_DELAY = 1.0;
    private final double MOVE_SKYSTONE_GRABBER_DELAY = 1.0;
    private final double JERK_DELAY = 0.75;
    private final double PULL_FOUNDATION_PAUSE_DURATION = 0;

    double[] motorPowers = {0.0, 0.0};

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

    private void detectSkystonePosition() {
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
                    if (recognition.getLabel() == LABEL_SECOND_ELEMENT) {

                    }
                }

                telemetry.update();
            }
        }
    }

    @Override
    public void init() {
        robot = new WHSRobotImpl(hardwareMap);
        robot.drivetrain.resetEncoders();
        defineStateEnabledStatus();

        startingCoordinateArray[RED] = new Coordinate(STARTING_COORDINATE_X, -1571, 90);
        startingCoordinateArray[BLUE] = new Coordinate(STARTING_COORDINATE_X, 1571, -90);

        skystoneMidpointArray[RED][0] = new Position(-760, -1175);
        skystoneMidpointArray[RED][1] = new Position(-795, -1175);
        skystoneMidpointArray[RED][2] = new Position(-975, -1000);
        skystoneMidpointArray[RED][3] = new Position(-1205, -600);
        skystoneMidpointArray[RED][4] = new Position(-1500, -600);
        skystoneMidpointArray[RED][5] = new Position(-1700, -600);


        skystoneMidpointArray[BLUE][0] = new Position(-760, 1175);
        skystoneMidpointArray[BLUE][1] = new Position(-735, 1175);
        skystoneMidpointArray[BLUE][2] = new Position(-985, 1000);
        skystoneMidpointArray[BLUE][3] = new Position(-605, 1300);
        skystoneMidpointArray[BLUE][4] = new Position(-1500, 1300);
        skystoneMidpointArray[BLUE][5] = new Position(-1700, 1300);

        // 0 = farthest from wall
        skystonePositionArray[RED][0] = new Position(-650, -560);
        skystonePositionArray[RED][1] = new Position(-940, -542);
        skystonePositionArray[RED][2] = new Position(-1130, -560);
        skystonePositionArray[RED][3] = new Position(-1205, -600);
        skystonePositionArray[RED][4] = new Position(-1500, -600);
        skystonePositionArray[RED][5] = new Position(-1700, -600);

        skystonePositionArray[BLUE][0] = new Position(-650, 560);
        skystonePositionArray[BLUE][1] = new Position(-920, 560);
        skystonePositionArray[BLUE][2] = new Position(-1145, 560);
        skystonePositionArray[BLUE][3] = new Position(-880, 600);
        skystonePositionArray[BLUE][4] = new Position(-1500, 600);
        skystonePositionArray[BLUE][5] = new Position(-1700, 600);

        skystoneToFoundationPositionArray[RED][0] = new Position(-800, -960);
        skystoneToFoundationPositionArray[RED][1] = new Position(-710, -910);
        skystoneToFoundationPositionArray[RED][2] = new Position(-940, -930);
        skystoneToFoundationPositionArray[RED][3] = new Position(-1205, -600);
        skystoneToFoundationPositionArray[RED][4] = new Position(-1500, -600);
        skystoneToFoundationPositionArray[RED][5] = new Position(-1700, -600);

        skystoneToFoundationPositionArray[BLUE][0] = new Position(-800, 960);
        skystoneToFoundationPositionArray[BLUE][1] = new Position(-710, 930);
        skystoneToFoundationPositionArray[BLUE][2] = new Position(-940, 930);
        skystoneToFoundationPositionArray[BLUE][3] = new Position(-1205, 600);
        skystoneToFoundationPositionArray[BLUE][4] = new Position(-1500, 600);
        skystoneToFoundationPositionArray[BLUE][5] = new Position(-1700, 600);

        foundationStartingPositionArray[RED] = new Position(1230, -640);
        foundationStartingPositionArray[BLUE] = new Position(1345, 660);

        foundationMovedPositionArray[RED] = new Position(950, -1250);
        foundationMovedPositionArray[BLUE] = new Position(950, 1310);

        slideOutFromFoundationMidpointArray[RED] = new Position(600, -1571);
        slideOutFromFoundationMidpointArray[BLUE] = new Position(600, 1571);

        skybridgePositionArray[RED][INSIDE] = new Position(0, -860);
        skybridgePositionArray[RED][OUTSIDE] = new Position(0, -900);

        skybridgePositionArray[BLUE][INSIDE] = new Position(320, 925);
        skybridgePositionArray[BLUE][OUTSIDE] = new Position(0, 930);


        skybridgeSecondCrossingPositionArray[RED][INSIDE] = new Position(0, -860);
        skybridgeSecondCrossingPositionArray[RED][OUTSIDE] = new Position(0, -900);

        skybridgeSecondCrossingPositionArray[BLUE][INSIDE] = new Position(320, 1025);
        skybridgeSecondCrossingPositionArray[BLUE][OUTSIDE] = new Position(0, 950);


        parkingMidpointArray[RED] = new Position(855,-780);
        parkingMidpointArray[BLUE] = new Position(855, 945);

        parkingPositions[RED] = new Position(180, -820);
        parkingPositions[BLUE] = new Position(100, 975);

        skystoneToFoundationMidpointArray[RED] = new Position(920, -1180);
        skystoneToFoundationMidpointArray[BLUE] = new Position(920, 1180);

        pullFoundationMidpointArray[RED] = new Position(840, -1255);
        pullFoundationMidpointArray[BLUE] = new Position(840, 1255);

        instantiateSwerveToTargets();
        robot.setInitialCoordinate(startingCoordinateArray[STARTING_ALLIANCE]);

        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        robot.webcam = new OpenCvWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        //webcam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        robot.webcam.openCameraDevice();
        robot.skystoneDetector = new ImprovedSkystoneDetector(STARTING_ALLIANCE == RED ? Alliance.RED : Alliance.BLUE);
        robot.skystoneDetector.useDefaults();
        robot.webcam.setPipeline(robot.skystoneDetector);
        robot.webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
    }
    @Override
    public void init_loop(){
        if (STARTING_ALLIANCE == BLUE) {
            if (robot.skystoneDetector.getScreenPosition().x < LEFT_MAX) {
                skystonePosition = 0;
            } else if (robot.skystoneDetector.getScreenPosition().x < CENTER_MAX) {
                skystonePosition = 1;
            } else {
                skystonePosition = 2;
            }
        } else if (STARTING_ALLIANCE == RED) {
            if (robot.skystoneDetector.getScreenPosition().x < LEFT_MAX) {
                skystonePosition = 2;
            } else if (robot.skystoneDetector.getScreenPosition().x < CENTER_MAX) {
                skystonePosition = 1;
            } else {
                skystonePosition = 0;
            }
        }

        telemetry.addData("Alliance", STARTING_ALLIANCE);
        telemetry.addData("Skystone Position", skystonePosition);
        telemetry.addData("Skystone X position", robot.skystoneDetector.getScreenPosition().x);
    }

    @Override
    public void loop() {
        robot.estimateHeading();
        robot.estimatePosition();
        robot.capstone.setLockPosition(Capstone.LockPosition.LOCKED);
        robot.capstone.setDumpPosition(Capstone.DumpPosition.UP);
        robot.capstone.setIntakeBlockerPosition(Capstone.IntakeBlockerPosition.UP);
        robot.deadWheelPickup.setPosition(DeadWheelPickup.DeadWheelPickupPosition.UP);
        if (!operatingFoundationPullers){
            robot.foundationPuller.operate(false);
        }
        if (!robot.driveToTargetInProgress() && !robot.rotateToTargetInProgress()) {
            robot.drivetrain.operate(motorPowers);
        }
        switch (outtakeState) {
            case "hover":
                robot.outtake.hover();
                break;
            case "grab":
                robot.outtake.grabStone();
                break;
            case "outtake":
                robot.outtake.autoOuttake();
                break;
            default:
                break;
        }

        switch (state) {
            case INIT:
                stateDesc = "Starting auto";
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
                        if (!startToFoundationSwerve.inProgress()) {
                            foundationPullerUpToDownTimer.set(GRAB_FOUNDATION_DELAY);
                            subState++;
                        }
                        break;
                    case 2:
                        subStateDesc = "Grabbing foundation";
                        robot.foundationPuller.setPosition(FoundationPuller.PullerPosition.DOWN);
                        operatingFoundationPullers = true;
                        if (foundationPullerUpToDownTimer.isExpired()) {
                            subState++;
                        }
                        break;
                    case 3:
                        subStateDesc = "Driving to wall";
                        motorPowers = foundationToWallSwerve.calculateMotorPowers(robot.getCoordinate(), robot.drivetrain.getWheelVelocities(), false);
                        if (!foundationToWallSwerve.inProgress()) {
                            subState++;
                        }
                        break;
                    case 4:
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
                        scanSkystoneTimer.set(SCAN_SKYSTONE_DURATION);
                        subState++;
                        break;
                    case 1:
                        subStateDesc = "Scanning Skystone";
                        if (scanSkystoneTimer.isExpired()) {
                            if (STARTING_ALLIANCE == BLUE) {
                                if (robot.skystoneDetector.getScreenPosition().x < LEFT_MAX) {
                                    skystonePosition = 0;
                                } else if (robot.skystoneDetector.getScreenPosition().x < CENTER_MAX) {
                                    skystonePosition = 1;
                                } else {
                                    skystonePosition = 2;
                                }
                            } else if (STARTING_ALLIANCE == RED) {
                                if (robot.skystoneDetector.getScreenPosition().x < LEFT_MAX) {
                                    skystonePosition = 2;
                                } else if (robot.skystoneDetector.getScreenPosition().x < CENTER_MAX) {
                                    skystonePosition = 1;
                                } else {
                                    skystonePosition = 0;
                                }
                            }
                            dropIntakeTimer.set(DROP_INTAKE_DELAY);
                            subState++;
                        }
                        break;
                    case 2:
                        subStateDesc = "dropping";
                        robot.intake.setIntakePusherPosition(Intake.IntakePusherPosition.UP);
                        if (dropIntakeTimer.isExpired()){
                            subState++;
                        }
                        break;
                    case 3:
                        subStateDesc = "Exit";
                        Position[] startToSkystoneSwervePositions = {startingCoordinateArray[STARTING_ALLIANCE], skystoneMidpointArray[STARTING_ALLIANCE][skystonePosition], skystonePositionArray[STARTING_ALLIANCE][skystonePosition]};
                        Position[] skystoneToMidpointJerkSwervePositions = {skystonePositionArray[STARTING_ALLIANCE][skystonePosition], skystoneToFoundationPositionArray[STARTING_ALLIANCE][skystonePosition]};
                        Position[] midPointToSkystoneJerkSwervePositions = {skystoneToFoundationPositionArray[STARTING_ALLIANCE][skystonePosition], skystonePositionArray[STARTING_ALLIANCE][skystonePosition]};

                        Position[] skystoneToMovedFoundationSwervePositions = {skystonePositionArray[STARTING_ALLIANCE][skystonePosition], skybridgePositionArray[STARTING_ALLIANCE][SKYBRIDGE_CROSSING_POSITION], foundationMovedPositionArray[STARTING_ALLIANCE]};
                        Position[] skystoneToUnmovedFoundationSwervePositions = {skystoneToFoundationPositionArray[STARTING_ALLIANCE][skystonePosition], skybridgePositionArray[STARTING_ALLIANCE][SKYBRIDGE_CROSSING_POSITION], pullFoundationMidpointArray[STARTING_ALLIANCE], foundationStartingPositionArray[STARTING_ALLIANCE]};
                        startToSkystoneSwerve = new SwerveToTarget(SwerveConstants.StartToSkystoneSwerveConstants.kP,
                                SwerveConstants.StartToSkystoneSwerveConstants.kV,
                                SwerveConstants.StartToSkystoneSwerveConstants.kA,
                                startToSkystoneSwervePositions,
                                80,
                                .2,
                                SwerveConstants.StartToSkystoneSwerveConstants.velocityConstant,
                                SwerveConstants.StartToSkystoneSwerveConstants.lookaheadDistance,
                                300);

                        skystoneToMidpointJerkSwerve = new SwerveToTarget(SwerveConstants.StartToSkystoneSwerveConstants.kP,
                                SwerveConstants.StartToSkystoneSwerveConstants.kV,
                                SwerveConstants.StartToSkystoneSwerveConstants.kA,
                                skystoneToMidpointJerkSwervePositions,
                                80,
                                .5,
                                SwerveConstants.StartToSkystoneSwerveConstants.velocityConstant,
                                SwerveConstants.StartToSkystoneSwerveConstants.lookaheadDistance,
                                800);

                        midPointToSkyStoneJerkSwerve = new SwerveToTarget(SwerveConstants.StartToSkystoneSwerveConstants.kP,
                                SwerveConstants.StartToSkystoneSwerveConstants.kV,
                                SwerveConstants.StartToSkystoneSwerveConstants.kA,
                                midPointToSkystoneJerkSwervePositions,
                                80,
                                .5,
                                SwerveConstants.StartToSkystoneSwerveConstants.velocityConstant,
                                SwerveConstants.StartToSkystoneSwerveConstants.lookaheadDistance,
                                800);

                        skystoneToMovedFoundationSwerve = new SwerveToTarget(SwerveConstants.SkystoneToMovedFoundationSwerveConstants.kP,
                                SwerveConstants.SkystoneToMovedFoundationSwerveConstants.kV,
                                SwerveConstants.SkystoneToMovedFoundationSwerveConstants.kA,
                                skystoneToMovedFoundationSwervePositions,
                                80,
                                0.8,
                                SwerveConstants.SkystoneToMovedFoundationSwerveConstants.velocityConstant,
                                SwerveConstants.SkystoneToMovedFoundationSwerveConstants.lookaheadDistance,
                                800);

                        skystoneToUnmovedFoundationSwerve = new SwerveToTarget(SwerveConstants.SkystoneToUnmovedFoundationSwerveConstants.kP,
                                SwerveConstants.SkystoneToUnmovedFoundationSwerveConstants.kV,
                                SwerveConstants.SkystoneToUnmovedFoundationSwerveConstants.kA,
                                skystoneToUnmovedFoundationSwervePositions,
                                80,
                                0.8,
                                SwerveConstants.SkystoneToUnmovedFoundationSwerveConstants.velocityConstant,
                                SwerveConstants.SkystoneToUnmovedFoundationSwerveConstants.lookaheadDistance - 110,
                                600);

                        advanceState();
                        break;
                }
                break;
            case INTAKE_SKYSTONE:
                stateDesc = "Grabbing Skystone";
                switch (subState) {
                    case 0:
                        subStateDesc = "Entry";
                        robot.backGate.setPosition(BackGate.BackGateServoPosition.INTAKING);
                        deadmanStrafeToSkystoneTimer.set(STRAFE_SKYSTONE_TIME);
                        //isStrafing = true;
                        subState++;
                        break;
                    case 1:
                        subStateDesc = "Intaking";
                        //robot.intake.setIntakePusherPosition(Intake.IntakePusherPosition.DOWN);
                        outtakeState = "hover";
                        if (robot.intake.stoneSensed()){
                            robot.intake.setMotorPowers(0);
                        }else{
                            robot.intake.setMotorPowers(Intake.AUTO_INTAKE_POWER);
                        }
                        motorPowers = startToSkystoneSwerve.calculateMotorPowers(robot.getCoordinate(), robot.drivetrain.getWheelVelocities(), false);
                        if (!startToSkystoneSwerve.inProgress()) {
                            //robot.intake.setMotorPowers();(0.0);
                            intakeSystoneTimer.set(JERK_DELAY);
                            subState++;
                        }
                        break;
                    case 2:
                        subStateDesc = "Waiting";
                        if (intakeSystoneTimer.isExpired()){
                            robot.backGate.setPosition(BackGate.BackGateServoPosition.UP);
                            jerkSkystoneTimer.set(JERK_DELAY);
                            subState++;
                        }
                        break;
                    /*case 2:
                        subStateDesc = "Jerking backwards";
                        motorPowers = skystoneToMidpointJerkSwerve.calculateMotorPowers(robot.getCoordinate(), robot.drivetrain.getWheelVelocities(), true);

                        if (!skystoneToMidpointJerkSwerve.inProgress()) {
                            subState++;
                        }
                        break;
                    case 3:
                        subStateDesc = "Jerking forwards";
                        motorPowers = midPointToSkyStoneJerkSwerve.calculateMotorPowers(robot.getCoordinate(), robot.drivetrain.getWheelVelocities(), false);

                        if (!midPointToSkyStoneJerkSwerve.inProgress()) {
                            subState++;
                        }
                        break;*/
                    case 3:
                        subStateDesc = "Jerking backwards again";
                        motorPowers = skystoneToMidpointJerkSwerve.calculateMotorPowers(robot.getCoordinate(), robot.drivetrain.getWheelVelocities(), true);
                        if (!skystoneToMidpointJerkSwerve.inProgress()) {
                            subState++;
                        }
                        break;
                    case 4:
                        subStateDesc = "waiting";
                        robot.intake.setMotorPowers(0.0);
                        if (jerkSkystoneTimer.isExpired()){
                            subState++;
                        }
                        break;
                    case 5:
                        subStateDesc = "Rotating like a neddy";
                        outtakeState = "grab";
                        robot.rotateToTarget(180, false);
                        if (!robot.rotateToTargetInProgress()) {
                            subState++;
                        }
                        break;
                    case 6:
                        subStateDesc = "Exit";
                        robot.intake.setIntakePusherPosition(Intake.IntakePusherPosition.DOWN);
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
                        if (stateEnabled[SECONDARY_MOVE_FOUNDATION]) {
                            motorPowers = skystoneToUnmovedFoundationSwerve.calculateMotorPowers(robot.getCoordinate(), robot.drivetrain.getWheelVelocities(), true);
                        } else {
                            motorPowers = skystoneToMovedFoundationSwerve.calculateMotorPowers(robot.getCoordinate(), robot.drivetrain.getWheelVelocities(), STARTING_ALLIANCE == BLUE);
                        }

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
                        foundationPullerUpToDownTimer.set(GRAB_FOUNDATION_DELAY);
                        subState++;
                        break;
                    case 1:
                        subStateDesc = "Moving foundation pullers";
                        robot.foundationPuller.setPosition(FoundationPuller.PullerPosition.DOWN);
                        operatingFoundationPullers = true;
                        outtakeState = "outtake";
                        if (foundationPullerUpToDownTimer.isExpired()) {
                            subState++;
                        }
                        break;
                    case 2:
                        subStateDesc = "Outtaking skystone and pulling foundation";
                        robot.intake.setMotorPowers(-Intake.AUTO_INTAKE_POWER);
                        motorPowers = foundationToWallSwerve.calculateMotorPowers(robot.getCoordinate(), robot.drivetrain.getWheelVelocities(), false);
                        if (!foundationToWallSwerve.inProgress()) {
                            robot.intake.setMotorPowers(0);
                            pullFoundationPauseTimer.set(PULL_FOUNDATION_PAUSE_DURATION);
                            subState++;
                        }
                        break;
                    case 3:
                        subStateDesc = "Waiting";
                        if (pullFoundationPauseTimer.isExpired()) {
                            subState++;
                        }
                        break;
                    case 4:
                        subStateDesc = "rotating";
                        if (STARTING_ALLIANCE == BLUE){
                            robot.rotateToTarget(-170,false);
                        }else if (STARTING_ALLIANCE == RED){
                            robot.rotateToTarget(170, false);
                        }
                        if (!robot.rotateToTargetInProgress()){
                            robot.foundationPuller.setPosition(FoundationPuller.PullerPosition.UP);
                        }
                        if (!robot.rotateToTargetInProgress()){
                            subState++;
                        }
                        break;
                    case 5:
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
                        subState = 4;
                        break;
                    case 1:
                        subStateDesc = "Grabbing foundation";
                        robot.foundationPuller.setPosition(FoundationPuller.PullerPosition.DOWN);
                        robot.intake.setMotorPowers(-Intake.AUTO_INTAKE_POWER);
                        if (foundationPullerUpToDownTimer.isExpired()) {
                            subState++;
                        }
                        break;
                    case 2:
                        subStateDesc = "Moving foundation";
                        motorPowers = foundationToWallSwerve.calculateMotorPowers(robot.getCoordinate(), robot.drivetrain.getWheelVelocities(), false);
                        if (!foundationToWallSwerve.inProgress()) {
                            subState++;
                        }
                        break;
                    case 3:
                        subStateDesc = "Rotating like a speddy";
                        robot.rotateToTarget(100,false);
                        if (robot.getCoordinate().getHeading() < 180){
                            robot.foundationPuller.setPosition(FoundationPuller.PullerPosition.UP);
                        }
                        if (!robot.rotateToTargetInProgress()){
                            subState++;
                        }
                    case 4:
                        subStateDesc = "Exit";
                        robot.intake.setMotorPowers(0);
                        advanceState();
                        break;
                }
                break;
            case GRAB_SECOND_SKYSTONE:
                stateDesc = "Intaking second skystone";
                switch (subState) {
                    case 0:
                        subStateDesc = "Initializing new swerves";
                        Position[] movedFoundationToSecondSkystonePosition = {foundationMovedPositionArray[STARTING_ALLIANCE], skybridgeSecondCrossingPositionArray[STARTING_ALLIANCE][SKYBRIDGE_CROSSING_POSITION], skystoneMidpointArray[STARTING_ALLIANCE][skystonePosition + 3], skystonePositionArray[STARTING_ALLIANCE][skystonePosition + 3]};
                        movedFoundationToSecondSkystoneSwerve = new SwerveToTarget(SwerveConstants.SkystoneToUnmovedFoundationSwerveConstants.kP,
                                SwerveConstants.SkystoneToUnmovedFoundationSwerveConstants.kV,
                                SwerveConstants.SkystoneToUnmovedFoundationSwerveConstants.kA,
                                movedFoundationToSecondSkystonePosition,
                                80,
                                0.8,
                                SwerveConstants.SkystoneToUnmovedFoundationSwerveConstants.velocityConstant,
                                SwerveConstants.SkystoneToUnmovedFoundationSwerveConstants.lookaheadDistance,
                                600);
                        Position[] secondSkystoneToMovedFoundationPosition = {skystonePositionArray[STARTING_ALLIANCE][skystonePosition + 3], skybridgeSecondCrossingPositionArray[STARTING_ALLIANCE][SKYBRIDGE_CROSSING_POSITION], foundationMovedPositionArray[STARTING_ALLIANCE]};
                        secondSkystonetoMovedFoundationSwerve = new SwerveToTarget(SwerveConstants.secondSkystoneToMovedFoundationSwerveConstants.kP,
                                SwerveConstants.secondSkystoneToMovedFoundationSwerveConstants.kV,
                                SwerveConstants.secondSkystoneToMovedFoundationSwerveConstants.kA,
                                secondSkystoneToMovedFoundationPosition,
                                80,
                                0.8,
                                SwerveConstants.secondSkystoneToMovedFoundationSwerveConstants.velocityConstant,
                                SwerveConstants.secondSkystoneToMovedFoundationSwerveConstants.lookaheadDistance,
                                600);
                        subState++;
                        break;
                    case 1:
                        subStateDesc = "Driving to second skystone and Intaking Stone";
                        motorPowers = movedFoundationToSecondSkystoneSwerve.calculateMotorPowers(robot.getCoordinate(), robot.drivetrain.getWheelVelocities(), false);
                        if (robot.getCoordinate().getX() < -100) {
                            outtakeState = "hover";
                        }
                        robot.intake.setMotorPowers(Intake.AUTO_INTAKE_POWER);
                        if (!movedFoundationToSecondSkystoneSwerve.inProgress()) {
                            secondStoneTimer.set(GRAB_SKYSTONE_DELAY);
                            subState++;
                        }
                        break;
                    case 2:
                        if(secondStoneTimer.isExpired()){
                            Position[] skystoneToMidpointJerkSwervePositions = {skystonePositionArray[STARTING_ALLIANCE][skystonePosition], skystoneToFoundationPositionArray[STARTING_ALLIANCE][skystonePosition]};
                            secondSkystoneToJerkPositionSwerve = new SwerveToTarget(SwerveConstants.StartToSkystoneSwerveConstants.kP,
                                    SwerveConstants.StartToSkystoneSwerveConstants.kV,
                                    SwerveConstants.StartToSkystoneSwerveConstants.kA,
                                    skystoneToMidpointJerkSwervePositions,
                                    80,
                                    .5,
                                    SwerveConstants.StartToSkystoneSwerveConstants.velocityConstant,
                                    SwerveConstants.StartToSkystoneSwerveConstants.lookaheadDistance,
                                    800);
                            subState++;
                        }
                        break;
                    case 3:
                        subStateDesc = "Jerking backwards again";
                        motorPowers = secondSkystoneToJerkPositionSwerve.calculateMotorPowers(robot.getCoordinate(), robot.drivetrain.getWheelVelocities(), true);
                        if (!skystoneToMidpointJerkSwerve.inProgress()) {
                            subState++;
                        }
                        break;
                    case 4:
                        subStateDesc = "Driving Back To Foundation";
                        motorPowers = secondSkystonetoMovedFoundationSwerve.calculateMotorPowers(robot.getCoordinate(), robot.drivetrain.getWheelVelocities(), true);
                        robot.intake.setMotorPowers(0);
                        outtakeState = "grab";
                        if (!secondSkystonetoMovedFoundationSwerve.inProgress()) {
                            subState++;
                        }
                        break;
                    case 5:
                        subStateDesc = "rotating like a teddy";
                        robot.rotateToTarget(180,false);
                        if(!robot.rotateToTargetInProgress()){
                            subState++;
                        }
                        break;
                    case 6:
                        subStateDesc = "Outtaking Stone";
                        outtakeState = "outtake";
                        if (!robot.outtake.autoOuttakeInProgress()) {
                            subState++;
                        }
                        break;
                    case 7:
                        subStateDesc = "Exit";
                        advanceState();
                        break;
                }
                break;
            case PARK:
                stateDesc = "Parking";
                switch (subState) {
                    case 0:
                        subStateDesc = "Entry";
                        operatingFoundationPullers = false;
                        subState++;
                        break;
                    case 1:
                        /*subStateDesc = "Rotating";
                        robot.rotateToTarget(0, true);
                        if (!robot.rotateToTargetInProgress()) {*/
                        subState++;
//                        }
                        break;
                    case 2:
                        subStateDesc = "Parking";
//                        if (stateEnabled[SECONDARY_MOVE_FOUNDATION]) {
//                            motorPowers = wallToParkSwerve.calculateMotorPowers(robot.getCoordinate(), robot.drivetrain.getWheelVelocities(), false);
//                        } else {
//                            motorPowers = movedFoundationToParkSwerve.calculateMotorPowers(robot.getCoordinate(), robot.drivetrain.getWheelVelocities(),STARTING_ALLIANCE == RED);
//                        }
                        /*if (180 - Math.abs(robot.getCoordinate().getHeading()) < 30) {
                            robot.foundationPuller.setPosition(FoundationPuller.PullerPosition.UP);
                        }
                        motorPowers = startToParkingSwerve.calculateMotorPowers(robot.getCoordinate(), robot.drivetrain.getWheelVelocities(), false);*/
                        motorPowers = movedFoundationToParkSwerve.calculateMotorPowers(robot.getCoordinate(), robot.drivetrain.getWheelVelocities(), false);
                        //if (!startToParkingSwerve.inProgress()) {
                        if (!movedFoundationToParkSwerve.inProgress()) {
                            subState++;
                        }
                        break;
                    case 3:
                        subStateDesc = "Exit";
                        advanceState();
                        break;
                }
                break;
            case END:
                robot.intake.setIntakePusherPosition(Intake.IntakePusherPosition.DOWN);
                break;
            default:
                break;
        }

        telemetry.addData("State: ", stateDesc);
        telemetry.addData("Substate: ", subStateDesc);
        telemetry.addData("IMU", robot.imu.getHeading());
        telemetry.addData("Stone Sensed?", robot.intake.stoneSensed());
        telemetry.addData("X", robot.getCoordinate().getX());
        telemetry.addData("Y", robot.getCoordinate().getY());
        telemetry.addData("Heading", robot.getCoordinate().getHeading());
        telemetry.addData("Stone Position X", robot.skystoneDetector.getScreenPosition().x);
        telemetry.addData("Stone Position Y", robot.skystoneDetector.getScreenPosition().y);
        telemetry.addData("Frame Count", robot.webcam.getFrameCount());
        telemetry.addData("FPS", String.format(Locale.US, "%.2f", robot.webcam.getFps()));
        telemetry.addData("Total frame time ms", robot.webcam.getTotalFrameTimeMs());
        telemetry.addData("Pipeline time ms", robot.webcam.getPipelineTimeMs());
        telemetry.addData("Overhead time ms", robot.webcam.getOverheadTimeMs());
        telemetry.addData("Theoretical max FPS", robot.webcam.getCurrentPipelineMaxFps());
        telemetry.addData("Skystone Position", skystonePosition);
    }

    @Override
    public void stop() {
        robot.intake.setIntakePusherPosition(Intake.IntakePusherPosition.DOWN);
    }
}
