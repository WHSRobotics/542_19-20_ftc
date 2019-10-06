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

    static final int STARTING_POSITION = FOUNDATION;
    static final int STARTING_ALLIANCE = RED;

    static final int SKYBRIDGE_CROSSING_POSITION = OUTSIDE;

    static final int STARTING_COORDINATE_X = 0;

    static final boolean PARTNER_MOVED_FOUNDATION = true;

    static final int LEFT = 0;
    static final int CENTER = 1;
    static final int RIGHT = 2;


    Coordinate[] startingCoordinateArray = new Coordinate[2];
    Position[][] skystonePositionArray = new Position[2][3];
    Position[] /*whitney*/foundationMovedPositionArray = new Position[2];
    Position[] /*whitney*/foundationMovedToPositionArray = new Position[2];
    Position[] /*whitney*/foundationStartingPositionArray = new Position[2];
    Position[][] skybridgePositionArray = new Position[2][2];

    Position[] startToFoundationSwervePositions;
    Position[] foundationToWallSwervePositions;
    Position[] wallToSkystoneSwervePositions;
    Position[] startToSkystoneSwervePositions;
    Position[] skystoneToMovedFoundationSwervePositions;
    Position[] skystoneToUnmovedSwervePositions;
    Position[] movedFoundationToParkingSwervePositions;
    Position[] wallToParkingSwervePositions;

    /*
    * Swerve to target instantiations
    */
        SwerveToTarget startToFoundationSwerve = new SwerveToTarget(SwerveConstants.StartToFoundationSwerveConstants.kP,
                SwerveConstants.StartToFoundationSwerveConstants.kV,
                SwerveConstants.StartToFoundationSwerveConstants.kA,
                startToFoundationSwervePositions,
                1,
                1,
                SwerveConstants.StartToFoundationSwerveConstants.velocityConstant,
                SwerveConstants.StartToFoundationSwerveConstants.lookaheadDistance);

        SwerveToTarget foundationToWallSwerve = new SwerveToTarget(SwerveConstants.FoundationToWallSwerveConstants.kP,
                SwerveConstants.FoundationToWallSwerveConstants.kV,
                SwerveConstants.FoundationToWallSwerveConstants.kA,
                foundationToWallSwervePositions,
                1,
                1,
                SwerveConstants.FoundationToWallSwerveConstants.velocityConstant,
                SwerveConstants.FoundationToWallSwerveConstants.lookaheadDistance);

        SwerveToTarget wallToSkystoneSwerve = new SwerveToTarget(SwerveConstants.WallToSkystoneSwerveConstants.kP,
                SwerveConstants.WallToSkystoneSwerveConstants.kV,
                SwerveConstants.WallToSkystoneSwerveConstants.kA,
                wallToSkystoneSwervePositions,
                1,
                1,
                SwerveConstants.WallToSkystoneSwerveConstants.velocityConstant,
                SwerveConstants.WallToSkystoneSwerveConstants.lookaheadDistance);

        SwerveToTarget startToSkystoneSwerve = new SwerveToTarget(SwerveConstants.StartToSkystoneSwerveConstants.kP,
                SwerveConstants.StartToSkystoneSwerveConstants.kV,
                SwerveConstants.StartToSkystoneSwerveConstants.kA,
                startToSkystoneSwervePositions,
                1,
                1,
                SwerveConstants.StartToSkystoneSwerveConstants.velocityConstant,
                SwerveConstants.StartToSkystoneSwerveConstants.lookaheadDistance);

        SwerveToTarget skystoneToMovedFoundationSwerve = new SwerveToTarget(SwerveConstants.SkystoneToMovedFoundationSwerveConstants.kP,
                SwerveConstants.SkystoneToMovedFoundationSwerveConstants.kV,
                SwerveConstants.SkystoneToMovedFoundationSwerveConstants.kA,
                skystoneToMovedFoundationSwervePositions,
                1,
                1,
                SwerveConstants.SkystoneToMovedFoundationSwerveConstants.velocityConstant,
                SwerveConstants.SkystoneToMovedFoundationSwerveConstants.lookaheadDistance);

    SwerveToTarget skystoneToUnmovedFoundationSwerve = new SwerveToTarget(SwerveConstants.SkystoneToUnmovedFoundationSwerveConstants.kP,
            SwerveConstants.SkystoneToUnmovedFoundationSwerveConstants.kV,
            SwerveConstants.SkystoneToUnmovedFoundationSwerveConstants.kA,
            skystoneToUnmovedSwervePositions,
            1,
            1,
            SwerveConstants.SkystoneToUnmovedFoundationSwerveConstants.velocityConstant,
            SwerveConstants.SkystoneToUnmovedFoundationSwerveConstants.lookaheadDistance);

    SwerveToTarget movedFoundationToParkSwerve = new SwerveToTarget(SwerveConstants.MovedFoundationToParkingSwerveConstants.kP,
            SwerveConstants.MovedFoundationToParkingSwerveConstants.kV,
            SwerveConstants.MovedFoundationToParkingSwerveConstants.kA,
            movedFoundationToParkingSwervePositions,
            1,
            1,
            SwerveConstants.MovedFoundationToParkingSwerveConstants.velocityConstant,
            SwerveConstants.MovedFoundationToParkingSwerveConstants.lookaheadDistance);

    SwerveToTarget wallToParkSwerve = new SwerveToTarget(SwerveConstants.WallToParkingSwerveConstants.kP,
            SwerveConstants.WallToParkingSwerveConstants.kV,
            SwerveConstants.WallToParkingSwerveConstants.kA,
            wallToParkingSwervePositions,
            1,
            1,
            SwerveConstants.WallToParkingSwerveConstants.velocityConstant,
            SwerveConstants.WallToParkingSwerveConstants.lookaheadDistance);
    /**
     * State Definitions
      */
    static final int INIT = 0;
    static final int INITIAL_MOVE_FOUNDATION = 1;
    static final int SCAN_SKYSTONE = 2;
    static final int INTAKE_SKYSTONE = 3;
    static final int DRIVE_TO_FOUDNATION = 4;
    static final int OUTTAKE_SKYSTONE = 5;
    static final int SECONDARY_MOVE_FOUNDATION = 6;
    static final int PARK = 7;
    static final int END = 8;

    static final int NUM_OF_STATES = 9;

    boolean[] stateEnabled = new boolean[NUM_OF_STATES];

    int state = INIT;
    int subState = 0;
    int skystonePosition = CENTER;
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
        stateEnabled[DRIVE_TO_FOUDNATION] = true;
        stateEnabled[OUTTAKE_SKYSTONE] = true;
        stateEnabled[SECONDARY_MOVE_FOUNDATION] = (STARTING_POSITION == SKYSTONE && !PARTNER_MOVED_FOUNDATION);
        stateEnabled[PARK] = true;
        stateEnabled[END] = true;
    }

    /**
     * Timers
     */
    SimpleTimer foundationPullerUpToDownTimer = new SimpleTimer();
    SimpleTimer foundationPullerDownToUpTimer = new SimpleTimer();
    SimpleTimer intakeSystoneTimer = new SimpleTimer();

    private final double GRAB_FOUNDATION_DELAY = 1;
    private final double  INTAKE_SKYSTONE_DELAY = 4;

    double[] motorPowers;

    /*Skystone Detection Tenserflow Variables*/
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

        startingCoordinateArray[RED] = new Coordinate(STARTING_COORDINATE_X, -1500, 190);
        startingCoordinateArray[BLUE] = new Coordinate(STARTING_COORDINATE_X, 1500,  -90);

        skystonePositionArray[RED][LEFT] = new Position(-1088, -606);
        skystonePositionArray[RED][CENTER] = new Position(-885,-60);
        skystonePositionArray[RED][RIGHT] = new Position(-682,-606);

        skystonePositionArray[BLUE][LEFT] = new Position(-682,606);
        skystonePositionArray[BLUE][CENTER] = new Position(-885,606);
        skystonePositionArray[BLUE][RIGHT] = new Position(-1088,606);

        foundationStartingPositionArray[RED] = new Position(STARTING_COORDINATE_X, -600);
        foundationStartingPositionArray[BLUE] = new Position(STARTING_COORDINATE_X,600);

        foundationMovedToPositionArray[RED] = new Position(STARTING_COORDINATE_X, -1571);
        foundationMovedToPositionArray[BLUE] = new Position(STARTING_COORDINATE_X, 1571);

        foundationMovedPositionArray[RED] = new Position(1024,-1108);
        foundationMovedPositionArray[BLUE] = new Position(1024, 1108);

        skybridgePositionArray[RED][INSIDE] = new Position(0,-900);
        skybridgePositionArray[RED][OUTSIDE] = new Position(0,-1500);

        skybridgePositionArray[BLUE][INSIDE] = new Position(0,900);
        skybridgePositionArray[BLUE][OUTSIDE] = new Position(0,1500);
    }

    @Override
    public void loop() {
        robot.estimateHeading();
        robot.estimatePosition();
        switch (state) {
            case INIT:
                stateDesc = "Starting auto";
                robot.setInitialCoordinate(startingCoordinateArray[STARTING_POSITION]);
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
                        motorPowers = startToFoundationSwerve.calculateMotorPowers(robot.getCoordinate(), robot.drivetrain.getWheelVelocities());
                        robot.drivetrain.operate(motorPowers);
                        if (!startToFoundationSwerve.inProgress()) {
                            foundationPullerUpToDownTimer.set(GRAB_FOUNDATION_DELAY);
                            subState++;
                        }
                        break;
                    case 2:
                        subStateDesc = "Grabbing foundation";
                        robot.foundationPuller.setPosition(FoundationPuller.PullerPosition.DOWN);
                        if (foundationPullerUpToDownTimer.isExpired()) {
                            subState++;
                        }
                        break;
                    case 3:
                        subStateDesc = "Driving to wall";
                        motorPowers = foundationToWallSwerve.calculateMotorPowers(robot.getCoordinate(), robot.drivetrain.getWheelVelocities());
                        robot.drivetrain.operate(motorPowers);
                        if (!foundationToWallSwerve.inProgress()) {
                            subState++;
                            foundationPullerDownToUpTimer.set(GRAB_FOUNDATION_DELAY);
                        }
                        break;
                    case 4:
                        subStateDesc = "Releasing foundation";
                        robot.foundationPuller.setPosition(FoundationPuller.PullerPosition.UP);
                        if (foundationPullerDownToUpTimer.isExpired()) {
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
                        subState++;
                        break;
                    case 1:
                        subStateDesc = "Moving to skystone";
                        if (STARTING_POSITION == SKYSTONE) {
                            motorPowers = startToSkystoneSwerve.calculateMotorPowers(robot.getCoordinate(), robot.drivetrain.getWheelVelocities());
                        } else {
                            motorPowers = wallToSkystoneSwerve.calculateMotorPowers(robot.getCoordinate(), robot.drivetrain.getWheelVelocities());
                        }
                        robot.drivetrain.operate(motorPowers);
                        if (!startToSkystoneSwerve.inProgress() && !wallToSkystoneSwerve.inProgress()) {
                            subState++;
                        }
                        break;
                    case 2:
                        subStateDesc = "Scanning skystone";
                        /*
                        scanning stuffs
                         */
                        subState++;
                        break;
                    case 3:
                        subStateDesc = "Exit";
                        advanceState();
                        break;
                }
                break;
            case INTAKE_SKYSTONE:
                stateDesc = "Intaking skystone";
                switch (subState) {
                    case 0:
                        subStateDesc = "Entry";
                        intakeSystoneTimer.set(INTAKE_SKYSTONE_DELAY);
                        subState++;
                        break;
                    case 1:
                        subStateDesc = "Intaking skystone";
                        robot.intake.setIntakeMotorPowers(Intake.INTAKE_POWER);
                        if (intakeSystoneTimer.isExpired()) {
                            robot.intake.setIntakeMotorPowers(0);
                            subState++;
                        }
                        break;
                    case 2:
                        subStateDesc = "Exit";
                        advanceState();
                        break;
                }
                break;
            case DRIVE_TO_FOUDNATION:
                stateDesc = "Driving to foundation";
                switch (subState) {
                    case 0:
                        subStateDesc = "Entry";
                        subState++;
                        break;
                    case 1:
                        subStateDesc = "Driving to foundation";
                        if (stateEnabled[SECONDARY_MOVE_FOUNDATION]) {
                            motorPowers = skystoneToUnmovedFoundationSwerve.calculateMotorPowers(robot.getCoordinate(), robot.drivetrain.getWheelVelocities());
                        } else {
                            motorPowers = skystoneToMovedFoundationSwerve.calculateMotorPowers(robot.getCoordinate(), robot.drivetrain.getWheelVelocities());
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
                        subState++;
                        break;
                    case 1:
                        subStateDesc = "Outtaking skystone";
                        robot.intake.setIntakeMotorPowers(-Intake.INTAKE_POWER);
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
                        robot.foundationPuller.setPosition(FoundationPuller.PullerPosition.DOWN);
                        if (foundationPullerUpToDownTimer.isExpired()) {
                            subState++;
                        }
                        break;
                    case 2:
                        subStateDesc = "Moving foundation";
                        motorPowers = foundationToWallSwerve.calculateMotorPowers(robot.getCoordinate(), robot.drivetrain.getWheelVelocities());
                        robot.drivetrain.operate(motorPowers);
                        if (!foundationToWallSwerve.inProgress()) {
                            foundationPullerDownToUpTimer.set(GRAB_FOUNDATION_DELAY);
                            subState++;
                        }
                        break;
                    case 3:
                        subStateDesc = "Releasing foundation";
                        robot.foundationPuller.setPosition(FoundationPuller.PullerPosition.UP);
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
                            motorPowers = wallToParkSwerve.calculateMotorPowers(robot.getCoordinate(), robot.drivetrain.getWheelVelocities());
                        } else {
                            motorPowers = movedFoundationToParkSwerve.calculateMotorPowers(robot.getCoordinate(), robot.drivetrain.getWheelVelocities());
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
