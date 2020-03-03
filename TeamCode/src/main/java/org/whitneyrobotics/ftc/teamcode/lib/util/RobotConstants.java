package org.whitneyrobotics.ftc.teamcode.lib.util;

//import com.acmerobotics.dashboard.config.Config;

import com.acmerobotics.dashboard.config.Config;
@Config
public class RobotConstants {

    public static double DEADBAND_DRIVE_TO_TARGET = 24.5;
    public static double DEADBAND_ROTATE_TO_TARGET = 1.0;
    public static double drive_min = .2;//.1245;
    public static double drive_max = 1.0;//.6;
    public static double rotate_min = 0.2;
    public static double rotate_max = 1.0;
    public static double R_KP = 2.0;//1.19;
    public static double R_KI = 0.4;//07542;
    public static double R_KD = 0.3;
    public static double D_KP = 0;//1.7;
    public static double D_KI = 0;//.7
    public static double D_KD = 0;//.8
    public static double A_KP = 0.0;
    public static double A_KI = 0.0;
    public static double A_KD = 0.0;
    public static double A_KF = 0.0;
    public static double E_KP = 0.006;
    public static double E_KI = 0.00000542;
    public static double E_KD = 0.0001;

    public static double rotateTestAngle = 180;
    public static boolean rotateOrientation = true;
}
