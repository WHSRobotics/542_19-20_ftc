package org.whitneyrobotics.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.whitneyrobotics.ftc.teamcode.lib.geometry.Coordinate;
import org.whitneyrobotics.ftc.teamcode.lib.util.RobotConstants;
import org.whitneyrobotics.ftc.teamcode.subsys.WHSRobotImpl;

/**
 * Created by Amar2 on 11/15/2017.
 */
@Autonomous(name = "RotateToTargetTest", group = "tests")
public class RotateToTargetTest extends OpMode {
    WHSRobotImpl robot;
//    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    @Override
    public void init() {
  //      TelemetryPacket packet = new TelemetryPacket();
    //    telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // dashboard.sendTelemetryPacket(packet);

        robot = new WHSRobotImpl(hardwareMap);
        robot.setInitialCoordinate(new Coordinate(0, 0, 0));
      //  telemetry.setMsTransmissionInterval(10);
    }

    @Override
    public void start(){
        robot.rotateToTarget(RobotConstants.rotateTestAngle, false);//RobotConstants.rotateOrientation);
    }

    @Override
    public void loop() {
        robot.rotateController.setConstants(RobotConstants.R_KP, RobotConstants.R_KI, RobotConstants.R_KD);

        if(robot.rotateToTargetInProgress()) {
            robot.rotateToTarget(RobotConstants.rotateTestAngle, false);//RobotConstants.rotateOrientation);
        }
        robot.estimatePosition();
        robot.estimateHeading();

        telemetry.addData("DriveToTarget in progress: ", robot.driveToTargetInProgress());
        telemetry.addData("RotateToTarget in progress: ", robot.rotateToTargetInProgress());
        telemetry.addData("IMU", robot.imu.getHeading());
        telemetry.addData("X", robot.getCoordinate().getX());
        telemetry.addData("Y", robot.getCoordinate().getY());
        telemetry.addData("Heading", robot.getCoordinate().getHeading());
        telemetry.addData("FL Power", robot.drivetrain.frontLeft.getPower());
        telemetry.addData("BL Power", robot.drivetrain.backLeft.getPower());
        telemetry.addData("FR Power", robot.drivetrain.frontRight.getPower());
        telemetry.addData("BR Power", robot.drivetrain.backRight.getPower());
        telemetry.addData("Distance to target", robot.distanceToTargetDebug);
        telemetry.addData("Angle to target", robot.angleToTargetDebug);
        telemetry.addData("BLdelta", robot.drivetrain.backLeft.getCurrentPosition());
        telemetry.addData("BRdelta", robot.drivetrain.backRight.getCurrentPosition());
        telemetry.addData("FLdelta", robot.drivetrain.frontLeft.getCurrentPosition());
        telemetry.addData("FRdelta", robot.drivetrain.frontRight.getCurrentPosition());
        telemetry.addData("Rotate Integral", robot.rotateController.getIntegral());
        telemetry.addData("Rotate Derivative", robot.rotateController.getDerivative());
        telemetry.addData("Drive Integral", robot.driveController.getIntegral());
        telemetry.addData("Drive Derivative", robot.driveController.getDerivative());

    }
}
