package org.whitneyrobotics.ftc.teamcode.lib.subsys;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Amar2 on 12/15/2017.
 */

public interface MotorSubsystem {

    /**
     * Sets the RunMode of all of the motors the subsystem uses
     * @param runMode The {@link DcMotor.RunMode} to set all of the motors to
     */
    void setRunMode(DcMotor.RunMode runMode);

    /**
     * Sets the ZeroPowerBehavior of all of the motors the subsystem uses
     * @param zeroPowerBehavior The {@link DcMotor.ZeroPowerBehavior} to set all of the motors to
     */
    void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior);



}
