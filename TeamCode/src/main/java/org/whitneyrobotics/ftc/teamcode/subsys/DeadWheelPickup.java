package org.whitneyrobotics.ftc.teamcode.subsys;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class DeadWheelPickup {

    Servo deadWheelPickup;
    public enum DeadWheelPickupPosition {
        UP, DOWN;
    }

    double[] DEAD_WHEEL_PICKUP_POSITIONS = {0.33, 0.74};

    public DeadWheelPickup(HardwareMap deadWheelMap){
        deadWheelPickup = deadWheelMap.servo.get("deadWheel");
    }

    public void setDeadWheelPickupPosition(DeadWheelPickupPosition deadWheelPickupPosition){
        deadWheelPickup.setPosition(DEAD_WHEEL_PICKUP_POSITIONS[deadWheelPickupPosition.ordinal()]);
    }


}
