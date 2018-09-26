package org.firstinspires.ftc.teamcode.Test;

/**
 * Created by KusakabeMirai on 9/7/2018.
 */

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by PantherBots on 9/7/2018.
 */

@TeleOp(name = "MyPushBot", group = "preseasonRobot")
public class PushBot6219 extends OpMode {

    public DcMotor leftMotor;
    public DcMotor rightMotor;
    public DcMotor intake;
    private final static double intakespeed = 0.7;

    @Override
    public void init(){
        leftMotor = hardwareMap.dcMotor.get("motorLeft");
        rightMotor = hardwareMap.dcMotor.get("motorRight");
        intake = hardwareMap.dcMotor.get("intake");
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        intake.setPower(0);
    }

    @Override
    public void loop(){
        leftMotor.setPower(Range.clip(gamepad1.left_stick_y,1,-1));
        rightMotor.setPower(Range.clip(gamepad1.right_stick_y,1,-1));
        if (gamepad1.left_bumper) intake.setPower(intakespeed);
        else intake.setPower(0);
    }

    @Override
    public void stop(){

    }

}