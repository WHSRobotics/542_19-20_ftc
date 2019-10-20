package org.whitneyrobotics.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.whitneyrobotics.ftc.teamcode.lib.util.Toggler;
import org.whitneyrobotics.ftc.teamcode.subsys.Drivetrain;


/**
 * Created by Amar2 on 10/21/2017.
 */
@TeleOp(name = "drivetrainTest", group = "tests")
public class DrivetrainTest extends OpMode {

    Drivetrain drivetrain;
    Toggler stateTog = new Toggler(4);
    String mode = "";
    double[] power = new double[2];
    Toggler powerTog = new Toggler(50);

    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap);
        telemetry.log().add("mode (scaled/normal) switch : gamepad1-x");
        telemetry.log().add("orientation switch : gamepad1-a");
    }

    @Override
    public void loop() {

        stateTog.changeState(gamepad1.x);
        if (stateTog.currentState() < 3) {
            drivetrain.switchOrientation(gamepad1.b);
        } else {
            drivetrain.switchFieldCentric(gamepad1.b);
        }
        powerTog.changeState(gamepad1.dpad_up, gamepad1.dpad_down);

        if (stateTog.currentState() == 0){
            drivetrain.operate(gamepad1.left_stick_y, gamepad1.right_stick_y);
            mode = "Normal";
        } else if (stateTog.currentState() == 1){
            drivetrain.operateWithOrientationScaled(gamepad1.left_stick_y, gamepad1.right_stick_y);
            mode = "Scaled";
        } else {
            if (gamepad1.b) {
                drivetrain.operate(powerTog.currentState() * 0.02, powerTog.currentState() * 0.02);
            } else {
                drivetrain.operate(0.0, 0.0);
            }
            mode = "Step";
        }


        telemetry.addData("Mode:", mode);
        telemetry.addData("Orientation:", drivetrain.getOrientation());
        telemetry.addData("LeftStickY:", gamepad1.left_stick_y);
        telemetry.addData("RightStickY:", gamepad1.right_stick_y);
        //telemetry.addData("Scaled L", drivetrain.getScaledPower(gamepad1.left_stick_y));
        //telemetry.addData("Scaled R", drivetrain.getScaledPower(gamepad1.right_stick_y));
        telemetry.addData("Power (step)", powerTog.currentState()*0.02);
    }
}