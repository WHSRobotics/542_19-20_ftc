package org.whitneyrobotics.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.whitneyrobotics.ftc.teamcode.lib.geometry.Coordinate;
import org.whitneyrobotics.ftc.teamcode.lib.geometry.Position;
import org.whitneyrobotics.ftc.teamcode.lib.util.RobotConstants;
import org.whitneyrobotics.ftc.teamcode.lib.util.SimpleTimer;
import org.whitneyrobotics.ftc.teamcode.subsys.WHSRobotImpl;
@Autonomous(name = "driveToTarget test")
public class DriveToTargetTest extends OpMode {

    WHSRobotImpl robot;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    Coordinate startingCoordinate = new Coordinate(600, -1571, -90);
    Position p1 = new Position(600, -600);
    Position p2 = new Position(500,0);
    int state = 0;

    SimpleTimer foundationPullerUpToDownTimer = new SimpleTimer();
    String subStateDesc = "";


    @Override
    public void init() {
        TelemetryPacket packet = new TelemetryPacket();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // dashboard.sendTelemetryPacket(packet);

        robot = new WHSRobotImpl(hardwareMap);
        robot.setInitialCoordinate(new Coordinate(0, 0, 0));
        telemetry.setMsTransmissionInterval(10);
    }

    @Override
    public void loop() {
        robot.driveController.setConstants(RobotConstants.D_KP, RobotConstants.D_KI, RobotConstants.D_KD);
        robot.estimatePosition();
        robot.estimateHeading();
        switch(state){
            case 0:
                robot.setInitialCoordinate(startingCoordinate);
                state++;
                break;
            case 1:
                subStateDesc = "Driving to foundation";
                robot.driveToTarget(p1, true);
                if (!robot.driveToTargetInProgress() && !robot.rotateToTargetInProgress()) {
                    foundationPullerUpToDownTimer.set(4.0);
                    state++;
                }
                break;
            case 2:
                subStateDesc = "Grabbing foundation";
                //robot.foundationPuller.setFoundationPullerPosition(FoundationPuller.PullerPosition.DOWN);
                if (foundationPullerUpToDownTimer.isExpired()) {
                    robot.driveToTarget(startingCoordinate, false);
                    state++;
                }
                break;
            case 3:
                subStateDesc = "Driving to wall";
                robot.driveToTarget(startingCoordinate.getPos(), false);
                if (!robot.driveToTargetInProgress() && !robot.rotateToTargetInProgress()) {
                    state++;
                }
                break;
            case 4:
                subStateDesc = "Releasing foundation";
                //robot.foundationPuller.setFoundationPullerPosition(FoundationPuller.PullerPosition.UP);

                break;
        }

        telemetry.addData("State: ", state);
        telemetry.addData("Substate: ", subStateDesc);
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
        telemetry.addData("Vector to Target x", robot.vectorToTargetDebug.getX());
        telemetry.addData("Vector to Target y", robot.vectorToTargetDebug.getY());

    }
}
