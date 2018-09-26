package org.firstinspires.ftc.teamcode.Hardware;

/**
 * Created by KusakabeMirai on 6/19/2018.
 */


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.Hallib.HalDashboard;
import org.firstinspires.ftc.teamcode.Hallib.HalUtil;
import org.firstinspires.ftc.teamcode.Pantherbot6219Lib.PIDController;
import org.firstinspires.ftc.teamcode.Robot.RobotMap;
import org.firstinspires.ftc.teamcode.Robot.TankDriveRobot;

//import ftclib.FtcAnalogGyro;

public class WCDriveBase implements PIDController.PidInput {

// Variable Declaration Section --------------------------------------------------------------------

    private LinearOpMode opMode;
    public PIDController pidControl, pidControlTurn;
    private Orientation angles;
    private Acceleration gravity;

    //private final static double SCALE = (144.5/12556.5);    // INCHES_PER_COUNT
    private final static double SCALE = (.0112142857);
    private double degrees = 0.0;
    private double stallStartTime = 0.0;
    private double prevTime = 0.0;
    private int prevLeftPos = 0;
    private int prevRightPos = 0;
    private boolean slowSpeed;
    private double minTarget, maxTarget;

    private final boolean forward = true;
    private final boolean reverse = false;

    public DcMotor leftFrontMotor, leftBackMotor,
            rightFrontMotor, rightBackMotor;

    public MotorGroup leftGroup, rightGroup;
    private BNO055IMU imu;
    //private ModernRoboticsI2cGyro gyroSensor;
    private HalDashboard dashboard;
    private ElapsedTime mRunTime;
    private boolean slow = true;

    double prevAngle = 0;

    public double intZ(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return (prevAngle - angles.firstAngle);
    }
    public void resetIntZ(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        prevAngle = angles.firstAngle;
    }
    public void reset(){
        leftGroup.setPower (0);
        rightGroup.setPower(0);
    }

    /*public interface AbortTrigger
    {
        boolean shouldAbort();
    }*/
    //--------------------------------------------------------------------------------------------

//Self Declaration: ---------------------------------------------------------------------------

    public WCDriveBase(LinearOpMode opMode) throws InterruptedException {

        //Variable Declaration section --------------------------------------------------------

        this.opMode = opMode;
        leftFrontMotor = opMode.hardwareMap.dcMotor.get(RobotMap.DcMotor.leftFrontMotor);
        leftBackMotor = opMode.hardwareMap.dcMotor.get(RobotMap.DcMotor.leftBackMotor);
        leftGroup = new MotorGroup(this.opMode, leftFrontMotor, leftBackMotor);
        rightFrontMotor = opMode.hardwareMap.dcMotor.get(RobotMap.DcMotor.rightFrontMotor);
        rightBackMotor = opMode.hardwareMap.dcMotor.get(RobotMap.DcMotor.rightBackMotor);
        rightGroup = new MotorGroup(this.opMode, rightFrontMotor, rightBackMotor);
        rightGroup.setDirection(reverse);
        // No need to setup encoders on the controller group, it is handled by the constructor itself.
        /*
        leftGroup.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightGroup.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftGroup.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightGroup.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        */

        // IMU init
        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        //gyro = new FtcAnalogGyro(opMode, "gyro1", 0.00067);
        //gyroSensor = (ModernRoboticsI2cGyro)opMode.hardwareMap.gyroSensor.get("gyro");
        mRunTime = new ElapsedTime();
        mRunTime.reset();
        dashboard = TankDriveRobot.getDashboard();
        //---------------------------------------------------------------------------------------

        //auto Gyro Calibration every init (not required)
        /*if(auto)
        {
            //gyro.calibrate();
            gyroSensor.calibrate();
            dashboard.displayPrintf(0, "Gyro : Calibrating");
            while(gyroSensor.isCalibrating()) {
                opMode.idle();
            }
            dashboard.displayPrintf(1, "Gyro : Done Calibrating");
        } */

        //Extended: variable Declaration -------------------------------------------------------
        //Sets up PID Drive: kP, kI, kD, kF, Tolerance, Settling Time
        pidControl = new PIDController(0.03,0,0,0,0.8,0.2,this);
        pidControlTurn = new PIDController(0.02,0,0,0,0.5,0.2,this);
        pidControlTurn.setAbsoluteSetPoint(true);
        dashboard.clearDisplay();
        //--------------------------------------------------------------------------------------
    }


    /**
     * void: drivePID(dist, "slow"(T/F), AbortTrigger (object) )
     *
     * This is a command block that gets called when driving.
     *
     * PID implemented -> see PIDController for details of how PID works.
     *
     * Assuming this is running in a loop
     */

    //Tip: Input distance in inches and power with decimal to hundredth place

    public void drivePID(double distance, boolean slow) throws InterruptedException {
        this.degrees = 0;
        resetIntZ();
        //Slow Mode check-----------------------------------------------------------------------
        if(slow) pidControl.setOutputRange(-0.5, 0.5);
        else pidControl.setOutputRange(-0.8,0.8);

        //Distance Check. Checked every tick.
        if (Math.abs(distance) <= 5) pidControl.setPID(0.081,0,0,0);
        else if(Math.abs(distance) <= 10) pidControl.setPID(0.0485,0,0,0);
        else pidControl.setPID(0.0345,0,.0005,0);

        //-------------------------------------------------------------------------

        pidControl.setTarget(distance);//ref drivePID (dist, T/F slow, AbortTrigger)
        pidControlTurn.setTarget(this.degrees);//WARNING: Must be defined if needed. Not provided.
        stallStartTime = HalUtil.getCurrentTime();//using HalUtil time.

        //While loop -----------------------------------------------------------------------------
        //while: pidControl is not on target or pidControlTurn is not on target, and OpMode is active
        while ((!pidControl.isOnTarget() || !pidControlTurn.isOnTarget()) && opMode.opModeIsActive()) {

            //safety: abort when needed. Please uncomment when in actual competition.

            /*if(abortTrigger != null && abortTrigger.shouldAbort())
            {
                break;
            }*/

            //Variable Definition and Assignment -----------------------------------------------
            int currLeftPos = leftGroup.getCurrentPosition();//this is an encoder output
            int currRightPos = rightGroup.getCurrentPosition();//another encoder output
            //-----------------------------------------------------------------------------------
            double drivePower = pidControl.getPowerOutput();//technically a curve, treat as variable (0-1)
            double turnPower = pidControlTurn.getPowerOutput();//same here (-1 - 1) (1 means left)(-1 means right)
            //----------------------------------------------------------------------------
            leftGroup.setPower(drivePower + turnPower);
            rightGroup.setPower(drivePower - turnPower);
            //--------------------------------------------------------------------------------
            double currTime = HalUtil.getCurrentTime();
            if (currLeftPos != prevLeftPos || currRightPos != prevRightPos)
            {
                stallStartTime = currTime;
                prevLeftPos = currLeftPos;
                prevRightPos = currRightPos;
            }
            else if (currTime > stallStartTime + 0.15)
            {
                // The motors are stalled for more than 0.15 seconds.
                break;
            }
            //---------------------------------------------------------------------------
            pidControlTurn.displayPidInfo(0);
            pidControl.displayPidInfo(5);
            opMode.idle();
        }
        //------------------------------------------------------------------------------------
        //When it is done, or aborted, or anything, it will auto reset motors to 0
        leftGroup.setPower(0);
        rightGroup.setPower(0);
        resetPIDDrive();
    }
//---------------------------------------------------------------------------------------------

    /**
     * Void: spinPID (deg)
     * Uses PIDControlTurn and the IMU module (gyro sensor)
     *
     * Using orientation (first angle, second angle, third angle
     *
     * Spin the robot to a certain degree, instead of making it going forward or curve
     *
     * Gyro: first angle is Z (heading) second angle is Y (pitch) third angle is X (roll)
     * Imagining flying an airplane, heading is if ur going N/S, Pitch is if ur going up/down,\m
     * Roll would be how tilted you are.
     *
     * */
    public void spinPID(double degrees) throws InterruptedException {
        resetIntZ();
        //calling in the angle measurements from the gyro
        //usage: angles.firstAngle......etc
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //resetIntZ();

        //lowering output range for 10 degs angles. so that it turns more accurately.
        if(Math.abs(degrees) < 10.0)pidControlTurn.setOutputRange(-0.45,0.45);
        else pidControlTurn.setOutputRange(-0.75,0.75);

        //------------------------------------------------------------------------------------
        //Setting PID variable based on target size.
        //TODO: Turning custom PID vairable for angles between 45 and 180
        this.degrees = degrees;
        if(Math.abs(degrees - intZ()) < 10.0){ //<10 deg PID dial
            pidControlTurn.setPID(0.15,0,0.0005,0);
        }
        else if(Math.abs(degrees - intZ()) < 20.0){ //<20deg PID dial
            //pidControlTurn.setPID(0.03,0,0.002,0);
            pidControlTurn.setPID(0.13,0,0.002,0);
        }
        else if(Math.abs(degrees - intZ()) < 45.0){ //<40deg PID dial
            //pidControlTurn.setPID(0.022,0,0.0011,0);
            pidControlTurn.setPID(0.122,0,0.0011,0);
        }
        else if(Math.abs(degrees - intZ()) < 90.0){//<90deg PID dial
            pidControlTurn.setPID(0.123,0,0.0005,0);
        }
        else {                                       //More than 90deg PID dial
            pidControlTurn.setPID(0.123,0,0,0);
        }
        //-----------------------------------------------------------------------------------
        pidControlTurn.setTarget(degrees);//sets target degrees.
        stallStartTime = HalUtil.getCurrentTime();//check time
        //----------------------------------------------------------------------------------
        //While: pidControlTurn not on target and OpMode is Active
        while (!pidControlTurn.isOnTarget() && opMode.opModeIsActive()) {

            //-----------------------------------------------------------------------------
            //Encoder Position Check
            int currLeftPos = leftGroup.getCurrentPosition();
            int currRightPos = rightGroup.getCurrentPosition();
            //Output Check generated from PID (results)
            double outputPower = pidControlTurn.getPowerOutput();
            //set Power
            leftGroup.setPower(outputPower);
            rightGroup.setPower(-outputPower);
            //------------------------------------------------------------------------------

            //Stall Check--------------------------------------------------------------------
            double currTime = HalUtil.getCurrentTime();//time check
            if (currLeftPos != prevLeftPos || currRightPos != prevRightPos)
            {
                stallStartTime = currTime;
                prevLeftPos = currLeftPos;
                prevRightPos = currRightPos;
            }
            else if (currTime > stallStartTime + 1)
            {
                // The motors are stalled for more than 1 seconds.
                break;
            }
            //---------------------------------------------------------------------------------

            pidControlTurn.displayPidInfo(0);
            opMode.idle();
        }
        //sets it to 0 when done.
        leftGroup.setPower(0);
        rightGroup.setPower(0);
        resetPIDDrive();//reset
        resetIntZ();
    }

    /**
     * This method drives the motors at "magnitude" and "curve". Both magnitude and curve are -1.0 to +1.0 values,
     * where 0.0 represents stopped and not turning. curve < 0 will turn left and curve > 0 will turn right. The
     * algorithm for steering provides a constant turn radius for any normal speed range, both forward and backward.
     * Increasing sensitivity causes sharper turns for fixed values of curve.
     *
     * @param magnitude specifies the speed setting for the outside wheel in a turn, forward or backwards, +1 to -1.
     * @param curve specifies the rate of turn, constant for different forward speeds. Set curve < 0 for left turn or
     *              curve > 0 for right turn. Set curve = e^(-r/w) to get a turn radius r for wheelbase w of your
     *              robot. Conversely, turn radius r = -ln(curve)*w for a given value of curve and wheelbase w.
     * @param inverted specifies true to invert control (i.e. robot front becomes robot back).
     */
    public void curve(double magnitude, double curve, boolean inverted, boolean gyroAssist)
    {
        double leftOutput;
        double rightOutput;
        double sensitivity = 0.5;

        if (curve < 0.0)
        {
            double value = Math.log(-curve);
            double ratio = (value - sensitivity)/(value + sensitivity);
            if (ratio == 0.0)
            {
                ratio = 0.0000000001;
            }
            leftOutput = magnitude/ratio;
            rightOutput = magnitude;
        }
        else if (curve > 0.0)
        {
            double value = Math.log(curve);
            double ratio = (value - sensitivity)/(value + sensitivity);
            if (ratio == 0.0)
            {
                ratio = 0.0000000001;
            }
            leftOutput = magnitude;
            rightOutput = magnitude/ratio;
        }
        else
        {
            leftOutput = magnitude;
            rightOutput = magnitude;
        }

        curveDrive(leftOutput, rightOutput, inverted, gyroAssist);
    }

    /**
     * This method implements tank drive where leftPower controls the left motors and right power controls the right
     * motors.
     *
     * @param leftPower specifies left power value.
     * @param rightPower specifies right power value.
     * @param inverted specifies true to invert control (i.e. robot front becomes robot back).
     */
    public void curveDrive(double leftPower, double rightPower, boolean inverted, boolean gyroAssist)
    {

        double currTime = HalUtil.getCurrentTime();

        leftPower = HalUtil.clipRange(leftPower);
        rightPower = HalUtil.clipRange(rightPower);

        double gyroRateScale = 0.0;
        double gyroAssistKp = 1.0;

        if (inverted)
        {
            double swap = leftPower;
            leftPower = -rightPower;
            rightPower = -swap;
        }

        if(gyroAssist)
        {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double diffPower = (leftPower - rightPower)/2.0;
            double assistPower = HalUtil.clipRange(gyroAssistKp*(diffPower - gyroRateScale*(intZ()/(currTime-prevTime))));
            leftPower += assistPower;
            rightPower -= assistPower;
            double maxMag = Math.max(Math.abs(leftPower), Math.abs(rightPower));
            if (maxMag > 1.0)
            {
                leftPower /= maxMag;
                rightPower /= maxMag;
            }
        }

        leftPower = HalUtil.clipRange(leftPower, -1.0, 1.0);
        rightPower = HalUtil.clipRange(rightPower, -1.0, 1.0);

        leftGroup.setPower(leftPower);
        rightGroup.setPower(rightPower);

        prevTime = currTime;

    }

    /**
     * sleep stops the program for number of seconds
     *
     * @param seconds is the time in seconds to wait
     */
    public void sleep(double seconds)
    {
        double startTime = HalUtil.getCurrentTime();
        while(startTime + seconds > HalUtil.getCurrentTime()) {}
    }

    /**
     * slowSpeed sets if the robot should go slowly
     *
     * @param slow is whether or not the robot should go slow
     */
    public void slowSpeed(boolean slow)
    {
        slowSpeed = slow;
    }


    /**
     * tankDrive controls the robot's drive train motors and gets input from the joysticks
     *
     * @param leftPower is the power for the left motor
     * @param rightPower is the power for the right motor
     */
    public void tankDrive(float leftPower, float rightPower) throws InterruptedException {
        leftPower = Range.clip(leftPower, -1, 1);
        rightPower = Range.clip(rightPower, -1, 1);
        if(slowSpeed) {
            leftPower = (float) scalePowerSlow(leftPower);
            rightPower = (float) scalePowerSlow(rightPower);
        } else {
            leftPower = (float) scalePower(leftPower);
            rightPower = (float) scalePower(rightPower);
        }
        leftGroup.setPower(leftPower);
        rightGroup.setPower(rightPower);
    }

    public void resetMotors() throws InterruptedException {
        leftGroup.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightGroup.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void resetPIDDrive() {
        pidControl.reset();
        pidControlTurn.reset();
    }

    public void noEncoders() {
        leftGroup.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightGroup.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private double scalePower(double dVal) {
        return -(Math.signum(dVal) * ((Math.pow(dVal, 2) * (.9)) + .1));
    }

    private double scalePowerSlow(double dVal) {
        return -(Math.signum(dVal) * ((Math.pow(dVal, 2) * (.55)) + .1));
    }

    public void resetHeading() {
        //gyro.resetIntegrator();
        // gyroSensor.resetZAxisIntegrator();
    }

    @Override
    public double getInput(PIDController pidCtrl) {
        double input = 0.0;
        if (pidCtrl == pidControl) {
            input = (leftGroup.getCurrentPosition() + rightGroup.getCurrentPosition())*SCALE/2.0;
        }
        else if (pidCtrl == pidControlTurn) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            //input = gyro.getHeading();
            input = intZ();
        }
        return input;
    }
}