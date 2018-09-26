package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robot.RobotMap;
import com.qualcomm.robotcore.hardware.DcMotor;


/**
 *  This class is for controlling two motor at the same time
 *  With Motor1 being the main driving motor
 *  All of the Sensor readings will be received from Motor1
 *  This class also assumes that you will be using encoder for the motor group.
 */
public class MotorGroup {

    private DcMotor motor1;
    private DcMotor motor2;

    private LinearOpMode opMode;
    private HardwareMap hwMap;

    public MotorGroup(LinearOpMode opMode, DcMotor motor1, DcMotor motor2){
        this.opMode = opMode;
        this.motor1 = motor1;
        this.motor2 = motor2;
        this.motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public MotorGroup(LinearOpMode opMode, String name1, String name2){
        this.opMode = opMode;
        this.motor1 = opMode.hardwareMap.dcMotor.get(name1);
        this.motor2 = opMode.hardwareMap.dcMotor.get(name2);
        this.motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * I dare you to set this group to RUN_WITHOUT_ENCODER
     *
     * In all seriousness though, DO NOT do that, it is a terrible idea
     * This class is specifically built for a dual motor system that can be used for a drive train
     * Therefore it is crucial for this class to be compatible with PID controller
     * And in order to harness the most from PID, you want the encoder to be functioning
     * @param mode
     */
    public void setMode(DcMotor.RunMode mode){
        this.motor1.setMode(mode);
        this.motor2.setMode(mode);
    }

    public void setPower(double power){
        this.motor1.setPower(power);
        this.motor2.setPower(power);
    }

    public void setDirection(boolean isForward){
        this.motor1.setDirection(isForward ? DcMotor.Direction.FORWARD : DcMotor.Direction.REVERSE);
        this.motor2.setDirection(isForward ? DcMotor.Direction.FORWARD : DcMotor.Direction.REVERSE);
    }

    public int getCurrentPosition(){
        return motor1.getCurrentPosition();
    }

    public DcMotor.Direction getDirection(){
        return motor1.getDirection();
    }

    public double getPower(){
        return motor1.getPower();
    }

    public void setPowerFloat(){
        motor1.setPowerFloat();
        motor2.setPowerFloat();
    }

    public boolean getPowerFloat(){
        return motor1.getPowerFloat();
    }

    public void setTargetPosition(int position){
        motor1.setTargetPosition(position);
        motor2.setTargetPosition(position);
    }

    public int getTargetPosition(){
        return motor1.getTargetPosition();
    }

    public void reset(){
        this.motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public boolean isBusy(){
        return (motor1.isBusy() || motor2.isBusy());
    }
}
