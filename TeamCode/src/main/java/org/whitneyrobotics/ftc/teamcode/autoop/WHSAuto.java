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
import org.whitneyrobotics.ftc.teamcode.lib.util.SwerveConstants;
import org.whitneyrobotics.ftc.teamcode.lib.util.SwerveToTarget;
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

    SwerveToTarget startToFoundationSwerve = new SwerveToTarget(SwerveConstants.StartToFoundationSwerveConstants.kP,
            SwerveConstants.StartToFoundationSwerveConstants.kV,
            SwerveConstants.StartToFoundationSwerveConstants.kA,
            startToFoundationSwervePositions,
            1,1,1,1,1,1);
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
        stateEnabled[SECONDARY_MOVE_FOUNDATION] = (STARTING_POSITION == SKYSTONE);
        stateEnabled[PARK] = true;
        stateEnabled[END] = true;
    }

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

        startingCoordinateArray[RED] = new Coordinate(STARTING_COORDINATE_X, -1500, 150,90);
        startingCoordinateArray[BLUE] = new Coordinate(STARTING_COORDINATE_X, 1500, 150, -90);

        skystonePositionArray[RED][LEFT] = new Position(-1088, -606, 150);
        skystonePositionArray[RED][CENTER] = new Position(-885,-606,150);
        skystonePositionArray[RED][RIGHT] = new Position(-682,-606,150);

        skystonePositionArray[BLUE][LEFT] = new Position(-682,606,150);
        skystonePositionArray[BLUE][CENTER] = new Position(-885,606,150);
        skystonePositionArray[BLUE][RIGHT] = new Position(-1088,606,150);

        foundationStartingPositionArray[RED] = new Position(STARTING_COORDINATE_X, -600, 150);
        foundationStartingPositionArray[BLUE] = new Position(STARTING_COORDINATE_X,600,150);

        foundationMovedToPositionArray[RED] = new Position(STARTING_COORDINATE_X, -1571, 150);
        foundationMovedToPositionArray[BLUE] = new Position(STARTING_COORDINATE_X, 1571, 150);

        foundationMovedPositionArray[RED] = new Position(1024,-1108,150);
        foundationMovedPositionArray[BLUE] = new Position(1024, 1108, 150);

        skybridgePositionArray[RED][INSIDE] = new Position(0,-900,150);
        skybridgePositionArray[RED][OUTSIDE] = new Position(0,-1500,150);

        skybridgePositionArray[BLUE][INSIDE] = new Position(0,900,150);
        skybridgePositionArray[BLUE][OUTSIDE] = new Position(0,1500,150);


    }

    @Override
    public void loop() {
        robot.estimateHeading();
        robot.estimatePosition();


    }
}
