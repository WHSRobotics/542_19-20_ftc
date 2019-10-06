package org.whitneyrobotics.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.whitneyrobotics.ftc.teamcode.lib.util.Toggler;
@TeleOp (name = "Servo Test", group =  "Tests")
public class ServoTest extends OpMode {
    Servo[] servos = new Servo[6];
    Toggler servoSelectionTog = new Toggler(6);
    Toggler servoPosition = new Toggler(200);
    int i;
    @Override
    public void init() {
        for (int i = 0; i<servos.length; i++) {
            servos[i] = hardwareMap.servo.get(Integer.toString(i));
        }
    }

    @Override
    public void loop() {
        i++;
        servoSelectionTog.changeState(gamepad1.dpad_up, gamepad1.dpad_down);
        servoPosition.changeState(gamepad1.dpad_up, gamepad1.dpad_down);
        if (i%10 == 0) {
            if (gamepad1.b) servoPosition.setState(servoPosition.currentState() - 1);
            if (gamepad1.a) servoPosition.setState(servoPosition.currentState() + 1);
        }
        servos[servoSelectionTog.currentState()].setPosition(servoPosition.currentState()/200);

        for(int i = 0; i<servos.length; i++){
            telemetry.addData("Servo" + i + "Current Pos: ", servos[i].getPosition());
        }
        telemetry.addData("Current Selected Servo: ", servoSelectionTog.currentState());
    }
}
