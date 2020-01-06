package org.whitneyrobotics.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.whitneyrobotics.ftc.teamcode.lib.util.Toggler;

@TeleOp(name="Outtake V2 Test")
public class OuttakeV2Test extends OpMode {
    TestOuttakeV2 grabber;
    double posRight;
    double posLeft;
    double posWrist;
    double posThumb;
    double correctPosition1;
    double correctPosition2;

    Toggler posTog = new Toggler(100);
    Toggler posTog2 = new Toggler(100);
    Toggler posTog3 = new Toggler(100);
    Toggler togBias = new Toggler(100);

    @Override
    public void init() {
        togBias.setState(50);
        grabber =new TestOuttakeV2(hardwareMap);
        posLeft=0.95;
        posRight=0.05;
        posWrist=0.95;
        posThumb=0.05;


    }

    @Override
    public void loop() {
        posTog.changeState(gamepad1.dpad_up,gamepad1.dpad_down );
        correctPosition1= posTog.currentState()/100.0;//lowinit
        correctPosition2=1-correctPosition1;//highiit;

        posTog2.changeState(gamepad1.b, gamepad1.x); // thumb
        posTog3.changeState(gamepad1.y, gamepad1.a); // wrist

        togBias.changeState(gamepad1.left_bumper, gamepad1.left_trigger>0.01);
        double bias = (togBias.currentState()-50.0)/100.0;

        grabber.upAndDown(correctPosition2,correctPosition1+bias);
        grabber.testThumb(posTog2.currentState()/100.0);
        grabber.testWrist(posTog3.currentState()/100.0);

        //grabber.down();
        telemetry.addData("leftPosition", grabber.left.getPosition());
        telemetry.addData("rightPosition", grabber.right.getPosition());
        telemetry.addData("wristPosition", grabber.wrist.getPosition());
        telemetry.addData("thumbPosition", grabber.thumb.getPosition());
    }
}
