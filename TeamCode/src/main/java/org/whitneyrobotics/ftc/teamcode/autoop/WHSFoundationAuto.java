package org.whitneyrobotics.ftc.teamcode.autoop;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.whitneyrobotics.ftc.teamcode.lib.util.Coordinate;
import org.whitneyrobotics.ftc.teamcode.lib.util.Position;
import org.whitneyrobotics.ftc.teamcode.lib.util.SimpleTimer;
import org.whitneyrobotics.ftc.teamcode.lib.util.SwerveConstants;
import org.whitneyrobotics.ftc.teamcode.lib.util.SwerveToTarget;
import org.whitneyrobotics.ftc.teamcode.subsys.FoundationPuller;
import org.whitneyrobotics.ftc.teamcode.subsys.Intake;
import org.whitneyrobotics.ftc.teamcode.subsys.WHSRobotImpl;

@Autonomous(name = "WHS Foundation Auto")
public class WHSFoundationAuto extends OpMode {
    WHSRobotImpl robot;

    static final int RED = 0;
    static final int BLUE = 1;
    static final int INSIDE = 0;
    static final int OUTSIDE = 1;

    public static final int STARTING_ALLIANCE = RED;
    static final int SKYBRIDGE_PARKING_POSITION = INSIDE;
    static final double STARTING_COORDINATE_X = 1200;

    Coordinate[] startingCoordinateArray = new Coordinate[2];
    Position[] foundationUnmovedPositionArray = new Position[2];
    Position[] foundationMovedPositionArray = new Position[2];
    Position[][] parkingPositionArray = new Position[2][2];

    SwerveToTarget startToFoundationSwerve;
    SwerveToTarget foundationToWallSwerve;
    SwerveToTarget wallToParkingSwerve;

    private void instantiateSwerveToTarget(){
        Position[] startToFoundationSwervePositions = {startingCoordinateArray[STARTING_ALLIANCE], foundationUnmovedPositionArray[STARTING_ALLIANCE]};
        startToFoundationSwerve = new SwerveToTarget(SwerveConstants.StartToFoundationSwerveConstants.kP,
                SwerveConstants.StartToFoundationSwerveConstants.kA,
                SwerveConstants.StartToFoundationSwerveConstants.kV,
                startToFoundationSwervePositions,
                80,
                5,
                SwerveConstants.StartToFoundationSwerveConstants.velocityConstant,
                SwerveConstants.StartToFoundationSwerveConstants.lookaheadDistance,
                1000);

        Position[] foundationToWallSwervePositions = {foundationUnmovedPositionArray[STARTING_ALLIANCE], foundationMovedPositionArray[STARTING_ALLIANCE]};
        foundationToWallSwerve = new SwerveToTarget(SwerveConstants.FoundationToWallSwerveConstants.kP,
                SwerveConstants.FoundationToWallSwerveConstants.kA,
                SwerveConstants.FoundationToWallSwerveConstants.kV,
                foundationToWallSwervePositions,
                80,
                5,
                SwerveConstants.FoundationToWallSwerveConstants.velocityConstant,
                SwerveConstants.FoundationToWallSwerveConstants.lookaheadDistance,
                1000);

        Position[] WallToParkingSwervePositions = {foundationMovedPositionArray[STARTING_ALLIANCE], parkingPositionArray[STARTING_ALLIANCE][SKYBRIDGE_PARKING_POSITION]};
        wallToParkingSwerve = new SwerveToTarget(SwerveConstants.WallToParkingSwerveConstants.kP,
                SwerveConstants.WallToParkingSwerveConstants.kA,
                SwerveConstants.WallToParkingSwerveConstants.kV,
                WallToParkingSwervePositions,
                80,
                5,
                SwerveConstants.WallToParkingSwerveConstants.velocityConstant,
                SwerveConstants.WallToParkingSwerveConstants.lookaheadDistance,
                5000);
    }

    //State Definitions
    static final int INIT = 0;
    static final int MOVE_FOUDNATION = 1;
    static final int PARK = 2;
    static final int END = 3;

    int NUM_OF_STATES = 4;
    boolean[] stateEnabled = new boolean[NUM_OF_STATES];

    int state = INIT;
    int subState = 0;
    String stateDesc;
    String substateDesc;

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
         /* Determines which states will be run.
            */

    public void defineStateEnabledStatus() {
        stateEnabled[INIT] = true;
        stateEnabled[MOVE_FOUDNATION] = true;
        stateEnabled[PARK] = true;
        stateEnabled[END] = true;
    }

    SimpleTimer foundationPullerUpToDownTimer = new SimpleTimer();

    private final double foundationPullerUpToDownDelay = 0.5;
    boolean isRotating = false;

    double[] motorPowers = {0.0, 0.0};
    @Override
    public void init() {
        robot = new WHSRobotImpl(hardwareMap);
        robot.drivetrain.resetEncoders();
        defineStateEnabledStatus();

        startingCoordinateArray[RED] = new Coordinate(STARTING_COORDINATE_X, -1571, -90);
        startingCoordinateArray[BLUE] = new Coordinate(STARTING_COORDINATE_X, 1571, 90);

        foundationUnmovedPositionArray[RED] = new Position(STARTING_COORDINATE_X, -800);
        foundationUnmovedPositionArray[BLUE] = new Position(STARTING_COORDINATE_X, 800);

        foundationMovedPositionArray[RED] = new Position(950, -1450);
        foundationMovedPositionArray[BLUE] = new Position(950, 1310);

        parkingPositionArray[RED][INSIDE] = new Position(0, -900);
        parkingPositionArray[RED][OUTSIDE] = new Position(400,-1500);

        parkingPositionArray[BLUE][INSIDE] = new Position(0, 900);
        parkingPositionArray[BLUE][OUTSIDE] = new Position(0,1400);

        instantiateSwerveToTarget();
        robot.setInitialCoordinate(startingCoordinateArray[STARTING_ALLIANCE]);
    }

    @Override
    public void loop() {
        robot.estimatePosition();
        robot.estimateHeading();
        if(!isRotating) {
            robot.drivetrain.operate(motorPowers);
        }

        switch (state){
            case INIT:
                stateDesc = "starting auto";
                advanceState();
                break;
            case MOVE_FOUDNATION:
                stateDesc = "moving foundation";
                switch (subState) {
                    case 0:
                        substateDesc = "init";
                        subState++;
                        break;
                    case 1:
                        substateDesc = "driving to foundation";
                        motorPowers = startToFoundationSwerve.calculateMotorPowers(robot.getCoordinate(), robot.drivetrain.getWheelVelocities(), true);
                        if (!startToFoundationSwerve.inProgress()) {
                            foundationPullerUpToDownTimer.set(foundationPullerUpToDownDelay);
                            subState++;
                        }
                        break;
                    case 2:
                        substateDesc = "moving foundation pullers";
                        robot.foundationPuller.setPosition(FoundationPuller.PullerPosition.DOWN);
                        if (foundationPullerUpToDownTimer.isExpired()) {
                            subState++;
                        }
                        break;
                    case 3:
                        substateDesc = "driving back to wall";
                        motorPowers = foundationToWallSwerve.calculateMotorPowers(robot.getCoordinate(), robot.drivetrain.getWheelVelocities(), false);
                        if (!foundationToWallSwerve.inProgress()) {
                            subState++;
                        }
                        break;
                    case 4:
                        isRotating = true;
                        double rotateAngle;
                        if (STARTING_ALLIANCE == RED){
                            rotateAngle = 0;
                        }else{
                            rotateAngle = 180;
                        }
                        robot.rotateToTarget(rotateAngle, true);
                        if (!robot.rotateToTargetInProgress()){
                            robot.intake.setIntakePusherPosition(Intake.IntakePusherPosition.UP);
                            robot.foundationPuller.setPosition(FoundationPuller.PullerPosition.UP);
                            isRotating = false;
                            subState++;
                        }
                        break;
                    case 5 :
                        substateDesc = "exit";
                        advanceState();
                        subState++;
                        break;
                }
                break;
                case PARK:
                    stateDesc = "parking";
                    switch (subState) {
                        case 0:
                            substateDesc = "entry";
                            subState++;
                            break;
                        case 1:
                            substateDesc = "moving to parking position";
                            motorPowers = wallToParkingSwerve.calculateMotorPowers(robot.getCoordinate(), robot.drivetrain.getWheelVelocities(), false);
                            if (!wallToParkingSwerve.inProgress()) {
                                subState++;
                            }
                            break;
                        case 2:
                            substateDesc = "rotating like a neddy";
                            if (SKYBRIDGE_PARKING_POSITION == INSIDE){
                                if (STARTING_ALLIANCE == RED){
                                    robot.rotateToTarget(0, false);
                                }else{
                                    robot.rotateToTarget(180, false);
                                }
                            }
                            if (!robot.rotateToTargetInProgress()){
                                subState++;
                            }
                            break;
                        case 3:
                            substateDesc = "exit";
                            advanceState();
                    }
                break;
                case END:
                    stateDesc = "end";
                    break;
                }
        telemetry.addData("State: ", stateDesc);
        telemetry.addData("Substate: ", substateDesc);
        telemetry.addData("IMU", robot.imu.getHeading());
        telemetry.addData("Stone Sensed?", robot.intake.stoneSensed());
        telemetry.addData("X", robot.getCoordinate().getX());
        telemetry.addData("Y", robot.getCoordinate().getY());
        telemetry.addData("Heading", robot.getCoordinate().getHeading());
        }

    }

