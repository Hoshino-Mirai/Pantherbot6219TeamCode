package org.firstinspires.ftc.teamcode.Hardware;

/**
 * Created by KusakabeMirai on 6/19/2018.
 */
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot.RobotMap;

/**
 * Created by margaretli on 12/16/17.
 */

public class TeamMarkerArm {
    Servo armgripper;
    Servo claw;
    LinearOpMode opMode;

    public TeamMarkerArm(LinearOpMode opMode){
        this.opMode = opMode;
        claw = opMode.hardwareMap.servo.get(RobotMap.Servo.claw);
    }

    public void open(){
        claw.setPosition(.7);
    }

    public void close(){
        claw.setPosition(0);
    }


}