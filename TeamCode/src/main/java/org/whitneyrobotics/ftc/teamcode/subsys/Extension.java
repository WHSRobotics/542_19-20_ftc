package org.whitneyrobotics.ftc.teamcode.subsys;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.whitneyrobotics.ftc.teamcode.lib.util.Toggler;

public class Extension {
    private DcMotor leftExtension;
    private DcMotor rightExtension;

    private int[] extensionMotorPositions = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

    private Toggler operateExtensionToggler;
    public Extension(HardwareMap extensionMap){
        operateExtensionToggler = new Toggler(11);
        leftExtension = extensionMap.dcMotor.get("leftExtension");
        rightExtension = extensionMap.dcMotor.get("rightExtension");
    }

    public void operateExtension(boolean gamepadInputUp, boolean gamepadInputDown){
        operateExtensionToggler.changeState(gamepadInputUp,gamepadInputDown);
        leftExtension.setTargetPosition(extensionMotorPositions[operateExtensionToggler.currentState()]);
        rightExtension.setTargetPosition(extensionMotorPositions[operateExtensionToggler.currentState()]);
    }

    public void setExtensionPosition (int extensionPosition){
        operateExtensionToggler.setState(extensionPosition);
        leftExtension.setTargetPosition(extensionMotorPositions[operateExtensionToggler.currentState()]);
        rightExtension.setTargetPosition(extensionMotorPositions[operateExtensionToggler.currentState()]);

    }
}
