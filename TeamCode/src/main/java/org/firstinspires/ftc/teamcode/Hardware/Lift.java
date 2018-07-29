package org.firstinspires.ftc.teamcode.Hardware;

/**
 * Created by KusakabeMirai on 6/19/2018.
 */
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Hallib.HalDashboard;
import org.firstinspires.ftc.teamcode.Hallib.HalUtil;
import org.firstinspires.ftc.teamcode.Robot.RobotMap;

public class Lift {

    private DcMotor lift;
    private LinearOpMode opMode;

    /**
     * Lift is a constructor
     *
     * @param opMode is the opMode in the main program
     */
    public Lift(LinearOpMode opMode){
        this.opMode = opMode;
        lift = opMode.hardwareMap.dcMotor.get(RobotMap.DcMotor.lift);
    }

    /**
     * up sets the lift to a certain position
     * @param liftPosition is the desired position
     */

    public void up (int liftPosition){
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (liftPosition == 0) {
            lift.setTargetPosition(1);
        }
        else if (liftPosition == 1) {
            lift.setTargetPosition(2);
        }
        else if (liftPosition == 2) {
            lift.setTargetPosition(3);
        }
        else if (liftPosition == 3) {
            lift.setTargetPosition(4);
        }
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // note the below line will cause a hang because of synchronous execution. best to use a state machine
        if (lift.isBusy())
            lift.setPower(.75);
        else {
            lift.setPower(0);
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void liftglyph(){
        HalUtil.sleep(400);
        lift.setPower(1);
        HalUtil.sleep(300);
        lift.setPower(0);
    }

    public void control(double power){
        lift.setPower(power);
    }

}