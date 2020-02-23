package org.whitneyrobotics.ftc.teamcode.subsys;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.whitneyrobotics.ftc.teamcode.lib.util.Toggler;

public class DeadWheelPickup {

    Servo deadWheelPickup;
    Toggler deadWheelPickupTog = new Toggler(2);
    public enum DeadWheelPickupPosition {
        UP, DOWN;
    }

    double[] DEAD_WHEEL_PICKUP_POSITIONS = {0.33, 0.74};

    public DeadWheelPickup(HardwareMap deadWheelMap){
        deadWheelPickup = deadWheelMap.servo.get("deadWheel");
    }

    public void setPosition(DeadWheelPickupPosition deadWheelPickupPosition){
        deadWheelPickup.setPosition(DEAD_WHEEL_PICKUP_POSITIONS[deadWheelPickupPosition.ordinal()]);
    }
    public void operate(boolean gamepadInput){
        deadWheelPickupTog.changeState(gamepadInput);
        if (deadWheelPickupTog.currentState() == 0) {
            setPosition(DeadWheelPickupPosition.UP);
        }else {
            setPosition(DeadWheelPickupPosition.DOWN);
        }
    }


}
