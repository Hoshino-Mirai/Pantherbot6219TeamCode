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
    DcMotor intake;

    LinearOpMode opMode;
    private double linearPower;
    private double rotationPower;
    private double intakePower;

    public Intake (LinearOpMode opMode) {
        this.opMode = opMode;
        linear = opMode.hardwareMap.dcMotor.get(RobotMap.DcMotor.linearIntake);
        rotation = opMode.hardwareMap.dcMotor.get(RobotMap.DcMotor.rotationIntake);
        intake = opMode.hardwareMap.dcMotor.get(RobotMap.DcMotor.intakeMotor);

    }

    public void extend(double power){
        if (linearPower < .8){
            linearPower = (scalePower(power));
        }
        else{
            linearPower = 0.8;
        }
        linear.setPower(linearPower);
    }
    public void in(double power){
        if (intakePower< .8){
            intakePower = (scalePower(power));
        }
        else{
            intakePower = 0.8;
        }
        intake.setPower(intakePower);
    }
    }

    public void rotate(double power){
        if (rotationPower < .8){
            rotationPower = (scalePower(power));
        }
        else{
            rotationPower = 0.8;
        }
        rotation.setPower(rotationPower);
    }

    private double scalePower(double dVal) {
        return (Math.signum(dVal) * ((Math.pow(dVal, 2) * (.9)) + .1));
    }

    public void out(){
        intake.setPower(-.8);
    }

    public void stop(){
        intake.setPower(0);
        rotation.setPower(0);
        linear.setPower(0);
    }

}