package org.whitneyrobotics.ftc.teamcode.tests;

import com.disnodeteam.dogecv.detectors.skystone.SkystoneDetector;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvWebcam;
import org.whitneyrobotics.ftc.teamcode.lib.util.Alliance;
import org.whitneyrobotics.ftc.teamcode.subsys.ImprovedSkystoneDetector;

import java.util.Locale;

@TeleOp(name="DogeCV Test")
//@Disabled
public class DogeCVTest extends OpMode {
    int i;
    ImprovedSkystoneDetector detector;
    OpenCvCamera webcam;

    @Override
    public void init() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = new OpenCvWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        //webcam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        webcam.openCameraDevice();
        this.detector = new ImprovedSkystoneDetector(Alliance.RED);
        this.detector.useDefaults();
        webcam.setPipeline(detector);
        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
    }

    @Override
    public void loop() {
        telemetry.addData("Stone Position X", detector.getScreenPosition().x);
        telemetry.addData("Stone Position Y", detector.getScreenPosition().y);
        telemetry.addData("Frame Count", webcam.getFrameCount());
        telemetry.addData("FPS", String.format(Locale.US, "%.2f", webcam.getFps()));
        telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
        telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
        telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
        telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
        //telemetry.addData("Classification", detector.getSkystoneState().toString());
        telemetry.update();

        if (gamepad1.a) {
            webcam.stopStreaming();
        } else if (gamepad1.x) {
            webcam.pauseViewport();
        } else if (gamepad1.y) {
            webcam.resumeViewport();
        }
    }
}
