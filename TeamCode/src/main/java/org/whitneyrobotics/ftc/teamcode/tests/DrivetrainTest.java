package org.whitneyrobotics.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.whitneyrobotics.ftc.teamcode.lib.util.Toggler;
import org.whitneyrobotics.ftc.teamcode.subsys.Drivetrain;
import org.whitneyrobotics.ftc.teamcode.subsys.IMU;


/**
 * Created by Amar2 on 10/21/2017.
 */
@TeleOp(name = "drivetrainTest", group = "tests")
public class DrivetrainTest extends OpMode {

    Drivetrain drivetrain;
    IMU imu;
    Toggler stateTog = new Toggler(5);
    String mode = "";
    double[] power = new double[2];
    Toggler powerTog = new Toggler(50);
    double currentVelocity = 0;
    double previousMaxVelocity = 0;
    double previousVelocity = 0;
    double currentAcceleration = 0;
    double previousMaxAcceleration = 0;
    double currentTime = 0;
    double lastTime = 0;

    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap);
        imu = new IMU(hardwareMap);
        telemetry.log().add("mode (scaled/normal) switch : gamepad1-x");
        telemetry.log().add("orientation switch : gamepad1-a");

    }

    @Override
    public void loop() {
        currentVelocity = drivetrain.getAllWheelVelocities()[0];
        currentTime = System.nanoTime() / 1E9;
        if (currentVelocity>previousMaxVelocity){
            previousMaxVelocity = currentVelocity;
        }
        telemetry.addData("Maximum Velocity", previousMaxVelocity);

        currentAcceleration = (currentVelocity - previousVelocity )/(currentTime - lastTime);
        if (currentAcceleration > previousMaxAcceleration){
            previousMaxAcceleration = currentAcceleration;
        }
        lastTime = currentTime;
        previousVelocity = currentVelocity;
        telemetry.addData("Max Acceleration", previousMaxAcceleration);
        telemetry.addData("kV", 1/previousMaxVelocity);

        stateTog.changeState(gamepad1.x);
        if (stateTog.currentState() < 3) {
            drivetrain.switchOrientation(gamepad1.b);
        } else {
            drivetrain.switchFieldCentric(gamepad1.b);
        }
        powerTog.changeState(gamepad1.dpad_up, gamepad1.dpad_down);

        if (stateTog.currentState() == 0){
            drivetrain.operateWithOrientation(gamepad1.left_stick_y, gamepad1.right_stick_y);
            mode = "Tank";
        } else if (stateTog.currentState() == 1){
            drivetrain.operateWithOrientationScaled(gamepad1.left_stick_y, gamepad1.right_stick_y);
            mode = " Tank Scaled";
        } else if (stateTog.currentState() == 2){
            if (gamepad1.a) {
                drivetrain.operateWithOrientation(powerTog.currentState() * 0.02, powerTog.currentState() * 0.02);
            } else {
                drivetrain.operate(0.0, 0.0);
            }
            mode = "Step";
        } else if (stateTog.currentState() == 3){
            drivetrain.operateMecanumDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, imu.getHeading());
            mode = "Mecanum";
        } else {
            drivetrain.operateMecanumDrive(gamepad1.left_stick_x/2.54, gamepad1.left_stick_y/2.54, gamepad1.right_stick_x/2.54, imu.getHeading());
            mode = "Mecanum Scaled";
        }


        telemetry.addData("Mode:", mode);
        telemetry.addData("Orientation:", stateTog.currentState() < 3 ? drivetrain.getOrientation() : drivetrain.getFieldCentric());
        telemetry.addData("LeftStickY:", gamepad1.left_stick_y);
        telemetry.addData("RightStickY:", gamepad1.right_stick_y);
        //telemetry.addData("Scaled L", drivetrain.getScaledPower(gamepad1.left_stick_y));
        //telemetry.addData("Scaled R", drivetrain.getScaledPower(gamepad1.right_stick_y));
        telemetry.addData("Power (step)", powerTog.currentState()*0.02);
    }
}