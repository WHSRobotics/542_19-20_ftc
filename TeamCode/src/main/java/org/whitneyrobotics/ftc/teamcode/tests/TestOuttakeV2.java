package org.whitneyrobotics.ftc.teamcode.tests;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TestOuttakeV2 {
    Servo left;
    Servo right;
    Servo wrist;
    Servo thumb;
    public TestOuttakeV2(HardwareMap elbowMap){
        left=elbowMap.servo.get("left");
        right=elbowMap.servo.get("right");
        wrist=elbowMap.servo.get("wrist");
        thumb=elbowMap.servo.get("thumb");
    }
    public void upAndDown(double leftPos, double rightPos){
        left.setPosition(leftPos);//low
        right.setPosition(rightPos);//high
     //originally up
    }
   /* public void down(double leftPos, double rightPos){
        left.setPosition(leftPos);//high
        right.setPosition(rightPos);//low
    }*/
    /*public void (){

        thumb.setPosition(0.9);

    }*/
    public void testThumb(double thumbPos){
        //wrist.setPosition(0.9);
        thumb.setPosition(thumbPos);
    }
    public void testWrist(double wristPos) {
    wrist.setPosition(wristPos);

    }
    /*public void release(double gamepadInput, int state boolean finish) {
        switch (state) {
            case 0:
                up();
                state++;

            case 1:



        }
    }*/


}
