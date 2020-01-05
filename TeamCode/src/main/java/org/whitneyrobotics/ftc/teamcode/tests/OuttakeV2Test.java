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
    @Override
    public void init() {
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
        grabber.upAndDown(correctPosition2,correctPosition1);
        grabber.testThumb(correctPosition1);
        grabber.testWrist(correctPosition2);

        //grabber.down();
        telemetry.addData("leftPosition", grabber.left.getPosition());
        telemetry.addData("rightPosition", grabber.right.getPosition());
        telemetry.addData("wristPosition", grabber.wrist.getPosition());
        telemetry.addData("thumbPosition", grabber.thumb.getPosition());
    }
}
