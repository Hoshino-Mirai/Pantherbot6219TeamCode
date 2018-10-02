package org.firstinspires.ftc.teamcode.Hardware;

/**
 * Created by KusakabeMirai on 6/19/2018.
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot.RobotMap;

public class Intake {
    DcMotor linear;
    DcMotor rotation;
    DcMotor Intake

    LinearOpMode opMode;
    private double linearPower;
    private double rotationPower;
    private double intakePower;

    public Intake (LinearOpMode opMode) {
        this.opMode = opMode;
        linear = opMode.hardwareMap.dcMotor.get(RobotMap.DcMotor.linearIntake);
        rotation = opMode.hardwareMap.dcMotor.get(RobotMap.DcMotor.rotationIntake);
        Intake = opMode.hardwareMap.dcMotor.get(RobotMap.DcMotor.intakeMotor);

    }

    public void in(double power){
        if (linearPower < .8){
            linearPower = (scalePower(power));
            rotationPower = (scalePower(power));
        }
        else{
            leftPower = 0.8;
            rightPower = 0.8;
        }
        hugleft.setPower(leftPower);
        hugright.setPower(-rightPower);
    }

    private double scalePower(double dVal) {
        return (Math.signum(dVal) * ((Math.pow(dVal, 2) * (.9)) + .1));
    }

    public void out(){
        hugleft.setPower(-.8);
        hugright.setPower(.8);
    }

    public void stop(){
        hugleft.setPower(0);
        hugright.setPower(0);
    }

}