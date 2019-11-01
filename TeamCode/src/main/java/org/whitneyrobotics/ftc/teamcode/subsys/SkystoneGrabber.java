package org.whitneyrobotics.ftc.teamcode.subsys;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class SkystoneGrabber {

    Servo skystoneGrabber;
    double[] positions = {0, 1};


    public enum SkystoneGrabberPosition{
        UP, DOWN;
    }

    public SkystoneGrabber(HardwareMap skystoneGrabberMap){
        skystoneGrabber = skystoneGrabberMap.servo.get("skystoneGrabber");
    }

    public void setPosition (SkystoneGrabberPosition position){
        skystoneGrabber.setPosition(positions[position.ordinal()]);

    }

}
