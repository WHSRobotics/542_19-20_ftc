package org.whitneyrobotics.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.whitneyrobotics.ftc.teamcode.lib.util.Toggler;
import org.whitneyrobotics.ftc.teamcode.subsys.Extension;

@TeleOp(name = "Extension.outtake test", group = "Tests")
public class OuttakeTest extends OpMode {
    Extension extension;
    Toggler extensionTog = new Toggler (4);

    @Override
    public void init() {
        extension = new Extension(hardwareMap);
    }

    @Override
    public void loop() {
        extensionTog.changeState(gamepad1.b);
        if (extensionTog.currentState() == 0){
            extension.setHandServoPosition(Extension.HandPosition.UP);
            extension.setWristServoPosition(Extension.WristPosition.OUTTAKE);
            extension.setElbowServoPosition(Extension.ElbowPosition.INTAKE);
        }else if(extensionTog.currentState() == 1){
            extension.setHandServoPosition(Extension.HandPosition.DOWN);
            extension.setWristServoPosition(Extension.WristPosition.INTAKE);
            extension.setElbowServoPosition(Extension.ElbowPosition.INTAKE);
        }else if (extensionTog.currentState() == 2){
            extension.setHandServoPosition(Extension.HandPosition.DOWN);
            extension.setWristServoPosition(Extension.WristPosition.INTAKE);
            extension.setElbowServoPosition(Extension.ElbowPosition.OUTTAKE);
        }else if (extensionTog.currentState() == 3){
            extension.setHandServoPosition(Extension.HandPosition.OUTTAKE);
            extension.setWristServoPosition(Extension.WristPosition.OUTTAKE);
            extension.setElbowServoPosition(Extension.ElbowPosition.OUTTAKE);
        }
    }
}
