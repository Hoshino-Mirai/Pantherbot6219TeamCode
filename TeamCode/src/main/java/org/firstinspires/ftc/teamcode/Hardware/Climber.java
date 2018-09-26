package org.firstinspires.ftc.teamcode.Hardware;

/**
 * Created by KusakabeMirai on 6/19/2018.
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hallib.HalDashboard;
import org.firstinspires.ftc.teamcode.Hallib.HalUtil;
import org.firstinspires.ftc.teamcode.Pantherbot6219Lib.PIDController;
import org.firstinspires.ftc.teamcode.Robot.RobotMap;
import org.firstinspires.ftc.teamcode.Robot.TankDriveRobot;

public class Climber implements PIDController.PidInput{
    public DcMotor climber;

    private LinearOpMode opMode;
    public PIDController pidControl;

    //private final static double SCALE = (144.5/12556.5);    // INCHES_PER_COUNT
    private final static double SCALE = (.0112142857);
    private double stallStartTime = 0.0;
    private double prevTime = 0.0;
    private int prevPos = 0;
    private boolean slowSpeed;
    private double minTarget, maxTarget;

    private HalDashboard dashboard;
    private ElapsedTime mRunTime;
    private boolean slow = true;

    double prevAngle = 0;

    public Climber (LinearOpMode opMode) {
        this.opMode = opMode;
        climber = opMode.hardwareMap.dcMotor.get(RobotMap.DcMotor.climber);
        climber.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        climber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mRunTime = new ElapsedTime();
        mRunTime.reset();
        dashboard = TankDriveRobot.getDashboard();
        pidControl = new PIDController(0.03,0,0.01,0,0.8,0.2,this);
        dashboard.clearDisplay();
    }

    public void controlPID(double distance, boolean slow) throws InterruptedException {
        //Slow Mode check-----------------------------------------------------------------------
        if(slow) {
            pidControl.setOutputRange(-0.5, 0.5);
        } else {
            pidControl.setOutputRange(-0.8,0.8);
        }

        //Distance Check. Checked every tick.
        if (Math.abs(distance) <= 5) {
            pidControl.setPID(0.081,0,0,0);
        }
        else if(Math.abs(distance) <= 10) {
            pidControl.setPID(0.0485,0,0,0);
        } else {
            pidControl.setPID(0.0345,0,.0005,0);
        }
        //-------------------------------------------------------------------------

        pidControl.setTarget(distance);//ref drivePID (dist, T/F slow, AbortTrigger)
        stallStartTime = HalUtil.getCurrentTime();//using HalUtil time.

        //While loop -----------------------------------------------------------------------------
        //while: pidControl is not on target or pidControlTurn is not on target, and OpMode is active
        while ((!pidControl.isOnTarget()) && opMode.opModeIsActive()) {

            //safety: abort when needed. Please uncomment when in actual competition.

            /*if(abortTrigger != null && abortTrigger.shouldAbort())
            {
                break;
            }*/

            //Variable Definition and Assignment -----------------------------------------------
            int currPos = climber.getCurrentPosition();//this is an encoder output
            //-----------------------------------------------------------------------------------
            double drivePower = pidControl.getPowerOutput();//technically a curve, treat as variable (0-1)
            //----------------------------------------------------------------------------
            climber.setPower(drivePower);
            //--------------------------------------------------------------------------------
            double currTime = HalUtil.getCurrentTime();
            if (currPos != prevPos) {
                stallStartTime = currTime;
                prevPos = currPos;
            }
            else if (currTime > stallStartTime + 0.15) {
                // The motors are stalled for more than 0.15 seconds.
                break;
            }
            //---------------------------------------------------------------------------
            pidControl.displayPidInfo(5);
            opMode.idle();
        }
        //------------------------------------------------------------------------------------
        //When it is done, or aborted, or anything, it will auto reset motors to 0
        climber.setPower(0);
        resetPIDDrive();
    }

    public void up(double p, int time){
        climber.setPower(p);
        opMode.sleep(time);
    }

    public void up(double p){
        climber.setPower(p);
    }

    public void down(double p, int time){
        climber.setPower(-p);
        opMode.sleep(time);
    }

    public void down(double p){
        climber.setPower(-p);
    }

    public void resetPIDDrive() {
        pidControl.reset();
    }

    public void controlManual(double power){
        climber.setPower(scalePower(power));
    }

    private double scalePower(double dVal) {
        return (Math.signum(dVal) * ((Math.pow(dVal, 2) * (.9)) + .1));
    }


    public void stop(){
        climber.setPower(0);
    }

    public double getInput(PIDController pidCtrl)
    {
        double input = 0.0;
        input = (climber.getCurrentPosition())*SCALE;
        return input;
    }

}