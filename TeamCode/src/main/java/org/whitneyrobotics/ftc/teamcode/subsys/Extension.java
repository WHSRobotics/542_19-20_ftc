package org.whitneyrobotics.ftc.teamcode.subsys;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.whitneyrobotics.ftc.teamcode.lib.util.Functions;

public class Extension {
    private DcMotor leftExtension;
    private DcMotor rightExtension;

    // index 0 = intook; index 3 = level 3, clearance & hover
    private int[] extensionMotorPositions = {0, 542, 1460, 2460, 3360, 5000, 5960};
    private static final int UPDATE_LEVEL_DEADBAND = 150;
    private double EXTENSION_MOTOR_POWER = 1.0;

    private int currentLevel = 0;

    public Extension(HardwareMap extensionMap){
        leftExtension = extensionMap.dcMotor.get("leftExtension");
        rightExtension = extensionMap.dcMotor.get("rightExtension");

        leftExtension.setDirection(DcMotorSimple.Direction.REVERSE);

        leftExtension.setTargetPosition(extensionMotorPositions[0]);
        rightExtension.setTargetPosition(extensionMotorPositions[0]);

        leftExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setLevel(int extensionLevel){
        leftExtension.setTargetPosition(extensionMotorPositions[extensionLevel]);
        rightExtension.setTargetPosition(extensionMotorPositions[extensionLevel]);
        leftExtension.setPower(EXTENSION_MOTOR_POWER);
        rightExtension.setPower(EXTENSION_MOTOR_POWER);
    }

    public void estimateLevel() {
        double[] differences = new double[extensionMotorPositions.length];
        double encoderAvg = (leftExtension.getCurrentPosition() + rightExtension.getCurrentPosition()) / 2;
        if (currentLevel > 0 && Math.abs(encoderAvg - extensionMotorPositions[currentLevel - 1]) < UPDATE_LEVEL_DEADBAND) {
            currentLevel--;
        }
        if (currentLevel < (extensionMotorPositions.length - 1) && Math.abs(encoderAvg - extensionMotorPositions[currentLevel + 1]) < UPDATE_LEVEL_DEADBAND) {
            currentLevel++;
        }
    }

    public int getCurrentLevel() {
        return currentLevel;
    }

    public void setTargetEncoderPosition(int targetPosition) {
        leftExtension.setTargetPosition(targetPosition);
        rightExtension.setTargetPosition(targetPosition);
        leftExtension.setPower(EXTENSION_MOTOR_POWER);
        rightExtension.setPower(EXTENSION_MOTOR_POWER);
    }
}

