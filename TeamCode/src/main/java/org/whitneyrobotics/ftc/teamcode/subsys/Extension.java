package org.whitneyrobotics.ftc.teamcode.subsys;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.whitneyrobotics.ftc.teamcode.lib.util.PIDController;
import org.whitneyrobotics.ftc.teamcode.lib.util.RobotConstants;

public class Extension {
    private DcMotor leftExtension;
    private DcMotor rightExtension;

    // index 0 = intook; index 3 = level 3, clearance & hover
    private int[] extensionMotorPositions = {0, 470, 1030, 1470, 1930, 2415, 2915, 3315, 3840, 3840, 3840};
    private int[] extensionHigherMotorPositions = new int[extensionMotorPositions.length];
    private static final int UPDATE_LEVEL_DEADBAND = 200;
    private double EXTENSION_MOTOR_POWER = 1.0;

    private PIDController extensionMotorController;

    private double kP = RobotConstants.E_KP;
    private double kI = RobotConstants.E_KI;
    private double kD = RobotConstants.E_KD;

    private int currentLevel = 0;

    private int error;
    protected int errorBias = 0;
    protected int temporaryErrorBias = 0;

    public Extension(HardwareMap extensionMap){
        leftExtension = extensionMap.dcMotor.get("leftExtension");
        rightExtension = extensionMap.dcMotor.get("rightExtension");

        rightExtension.setDirection(DcMotorSimple.Direction.REVERSE);

//        leftExtension.setTargetPosition(extensionMotorPositions[0]);
//        rightExtension.setTargetPosition(extensionMotorPositions[0]);

        leftExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
/*
        leftExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);*/

        for (int i = 0; i < extensionHigherMotorPositions.length; i++) {
            extensionHigherMotorPositions[i] = extensionMotorPositions[i] + 290;
        }

        extensionMotorController= new PIDController(RobotConstants.E_KP,RobotConstants.E_KI, RobotConstants.E_KD);
        extensionMotorController.setConstants(RobotConstants.E_KP,RobotConstants.E_KI, RobotConstants.E_KD);
    }

    public void setLevel(int extensionLevel){
        error = extensionMotorPositions[extensionLevel] - getAvgEncoderPosition() + errorBias + temporaryErrorBias;

        extensionMotorController.setConstants(RobotConstants.E_KP,RobotConstants.E_KI, RobotConstants.E_KD);
        extensionMotorController.calculate(error);

        double power = extensionMotorController.getOutput();
        setPower(power);
    }

    public void setHigherLevel(int higherExtensionLevel) {
        error = extensionHigherMotorPositions[higherExtensionLevel] - getAvgEncoderPosition() + errorBias + temporaryErrorBias;

        extensionMotorController.setConstants(RobotConstants.E_KP,RobotConstants.E_KI, RobotConstants.E_KD);
        extensionMotorController.calculate(error);

        double power = extensionMotorController.getOutput();
        setPower(power);
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
        leftExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftExtension.setTargetPosition(targetPosition);
        rightExtension.setTargetPosition(targetPosition);
        leftExtension.setPower(EXTENSION_MOTOR_POWER);
        rightExtension.setPower(EXTENSION_MOTOR_POWER);
    }

    public int getAvgEncoderPosition(){
        return (leftExtension.getCurrentPosition() + rightExtension.getCurrentPosition())/2;
    }

    public void setPower(double power){
        leftExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftExtension.setPower(power);
        rightExtension.setPower(power);
    }

    public int getError(){
        return error;
    }

    public double getPIDOutput(){
        return extensionMotorController.getOutput();
    }

    public int[] getEncoderPositions() {
        return new int[] {leftExtension.getCurrentPosition(), rightExtension.getCurrentPosition()};
    }

}

