package org.whitneyrobotics.ftc.teamcode.lib.util;

//import com.acmerobotics.dashboard.config.Config;

//@Config
public class RobotConstants {
    public static double leftMultiplier = 1;
    public static double rightMultiplier = 1;
    public static double totalMultiplier = 1/2.54;
    public static int canDrive = 1;
    public static int canIntake = 1;
    public static int canExtend = 1;
    public static int canStoreArm = 1;
    public static int canLift = 1;

    public static double DEADBAND_DRIVE_TO_TARGET = 24.5;
    public static double DEADBAND_ROTATE_TO_TARGET = 1.0;
    public static double drive_min = .2;//.1245;
    public static double drive_max = 1.0;//.6;
    public static double rotate_min = 0.2;
    public static double rotate_max = 1.0;
    public static double R_KP = 1.542;//1.19;
    public static double R_KI = 0.3;
    public static double R_KD = 0.36;
    public static double D_KP = 1.7;//1.5;
    public static double D_KI = 0.7;
    public static double D_KD = 0.8;
    public static double A_KP = 0.0;
    public static double A_KI = 0.0;
    public static double A_KD = 0.0;
    public static double A_KF = 0.0;

    public static double rotateTestAngle = 45;
    public static boolean rotateOrientation = true;
}
