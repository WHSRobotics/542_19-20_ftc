package org.whitneyrobotics.ftc.teamcode.subsys;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.whitneyrobotics.ftc.teamcode.lib.util.Toggler;

public class SkystoneGrabber {

    Servo skystoneGrabber;
    double[] positions = {0.1 ,0.235, 0.97};
    private Toggler operateSkystoneGrabberTog;

    public enum SkystoneGrabberPosition{
        REST, UP, DOWN;
    }

    public SkystoneGrabber(HardwareMap skystoneGrabberMap){
        skystoneGrabber = skystoneGrabberMap.servo.get("skystoneGrabber");
        operateSkystoneGrabberTog = new Toggler(3);
    }

    public void setPosition (SkystoneGrabberPosition position){
        skystoneGrabber.setPosition(positions[position.ordinal()]);

    }

    public void operate (boolean gamepadInput){
        operateSkystoneGrabberTog.changeState(gamepadInput);
        if(operateSkystoneGrabberTog.currentState() == 2){
            setPosition(SkystoneGrabberPosition.UP);
        }else if (operateSkystoneGrabberTog.currentState() == 1){
            setPosition(SkystoneGrabberPosition.DOWN);
        }else if (operateSkystoneGrabberTog.currentState() == 0){
            setPosition(SkystoneGrabberPosition.REST);
        }
    }
}
