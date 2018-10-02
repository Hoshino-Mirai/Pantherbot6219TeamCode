package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Hallib.HalUtil;
import org.firstinspires.ftc.teamcode.Robot.RobotMap;

public class TeamMarkerDeployer {
    private LinearOpMode opMode;

    public Servo joint;
    public Servo pivot;
    public Servo claw;

    private final double jointZero = 0;
    private final double pivotZero = 0;
    private final double clawZero = 0.3;

    private final double jointDeploy = 0.7;
    private final double pivotDeploy = 0.7;
    private final double clawDeploy = 0.9;

    public TeamMarkerDeployer(LinearOpMode opMode){
        this.opMode = opMode;
        joint = opMode.hardwareMap.servo.get(RobotMap.Servo.joint);
        pivot = opMode.hardwareMap.servo.get(RobotMap.Servo.pivot);
        claw = opMode.hardwareMap.servo.get(RobotMap.Servo.claw);
        resetAll();
    }

    public void resetAll(){
        joint.setPosition(jointZero);
        pivot.setPosition(pivotZero);
        claw.setPosition(clawZero);
    }

    public void deployMarker(){
        joint.setPosition(jointDeploy);
        HalUtil.sleep(500);
        pivot.setPosition(pivotDeploy);
        HalUtil.sleep(500);
        claw.setPosition(clawDeploy);
        HalUtil.sleep(100);
    }

}
