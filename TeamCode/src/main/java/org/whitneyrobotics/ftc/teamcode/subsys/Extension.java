package org.whitneyrobotics.ftc.teamcode.subsys;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.whitneyrobotics.ftc.teamcode.lib.util.Functions;

public class Extension {
    private DcMotor leftExtension;
    private DcMotor rightExtension;

    // index 0 = intook; index 3 = level 3, clearance & hover
    private int[] extensionMotorPositions = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    private int CLEARANCE_POSITION = 0;

    private int currentLevel = 0;

    public Extension(HardwareMap extensionMap){
        leftExtension = extensionMap.dcMotor.get("leftExtension");
        rightExtension = extensionMap.dcMotor.get("rightExtension");

        rightExtension.setDirection(DcMotorSimple.Direction.REVERSE);

        leftExtension.setTargetPosition(extensionMotorPositions[0]);
        rightExtension.setTargetPosition(extensionMotorPositions[0]);

        leftExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setLevel(int extensionLevel){
        leftExtension.setTargetPosition(extensionMotorPositions[extensionLevel]);
        rightExtension.setTargetPosition(extensionMotorPositions[extensionLevel]);
    }

    public void estimateLevel() {
        double[] differences = new double[extensionMotorPositions.length];
        double encoderAvg = (leftExtension.getCurrentPosition() + rightExtension.getCurrentPosition()) / 2;
        for (int i = 0; i < extensionMotorPositions.length; i++) {
            differences[i] = Math.abs(encoderAvg - (double)extensionMotorPositions[i]);
        }
        currentLevel = Functions.calculateIndexOfSmallestValue(differences);
    }

    public int getCurrentLevel() {
        return currentLevel;
    }
}

