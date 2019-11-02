package org.whitneyrobotics.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(name = "aditya test")
public class adityaTest extends OpMode {
   DcMotor testRun;
   //created test motor object
    @Override
    public void init() {
     testRun=hardwareMap.dcMotor.get("testMotor");
     testRun.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    @Override
    public void loop() {
      testRun.setPower(-1.0);
        testRun.setPower(-0.5);
        testRun.setPower(0.0);
        testRun.setPower(0.5);
        testRun.setPower(1.0);
        //goes through a range
       /* testRun.setTargetPosition(1200);
        testRun.setMode(DcMotor.RunMode.RUN_TO_POSITION);*/


        telemetry.addData("motor position", testRun.getCurrentPosition());


    }
}
