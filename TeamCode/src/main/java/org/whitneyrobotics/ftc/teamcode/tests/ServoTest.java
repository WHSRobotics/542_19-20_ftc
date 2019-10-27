package org.whitneyrobotics.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.whitneyrobotics.ftc.teamcode.lib.util.Toggler;
@TeleOp (name = "Servo Test", group =  "Tests")
public class ServoTest extends OpMode {
    Servo[] servos = new Servo[6];
    Toggler servoSelectionTog = new Toggler(6);
    Toggler servoPositionTog = new Toggler(200);
    int i = 0;
    @Override
    public void init() {
        for (int i = 0; i<servos.length; i++) {
            servos[i] = hardwareMap.servo.get(Integer.toString(i));
        }
    }

    @Override
    public void loop() {
        i++;
        servoPositionTog.changeState(gamepad1.a, gamepad1.b);
        servoSelectionTog.changeState(gamepad1.dpad_right, gamepad1.dpad_left);
        servos[servoSelectionTog.currentState()].setPosition(servoPositionTog.currentState()/200f);
        if (i%10 == 0) {
            if (gamepad1.x) servoPositionTog.setState(servoPositionTog.currentState() + 1);
            if (gamepad1.y) servoPositionTog.setState(servoPositionTog.currentState() - 1);
        }

        for(int i = 0; i<servos.length; i++){
            telemetry.addData("Servo" + i + "Current Pos: ", servos[i].getPosition());
        }
        telemetry.addData("Current Selected Servo: ", servoSelectionTog.currentState());
    }
}
