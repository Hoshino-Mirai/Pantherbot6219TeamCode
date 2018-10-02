package org.firstinspires.ftc.teamcode.Robot;

/**
 * Created by KusakabeMirai on 6/19/2018.
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Hallib.HalDashboard;
import org.firstinspires.ftc.teamcode.Hardware.Intake;
import org.firstinspires.ftc.teamcode.Hardware.JewelArm;
import org.firstinspires.ftc.teamcode.Hardware.TeamMarkerArm;
import org.firstinspires.ftc.teamcode.Hardware.TankDriveBase;
import org.firstinspires.ftc.teamcode.Hardware.TeamMarkerDeployer;
import org.firstinspires.ftc.teamcode.Hardware.VuMark;

public class TankDriveRobot {

    private static HalDashboard dashboard = null;
    public TankDriveBase driveBase = null;
    public TeamMarkerDeployer deployer = null;
    public org.firstinspires.ftc.teamcode.Hardware.VuMark VuMark = null;
    public Intake intake = null;

    public TankDriveRobot(LinearOpMode opMode, boolean auto) throws InterruptedException {
        dashboard = new HalDashboard(opMode.telemetry);
        deployer = new TeamMarkerDeployer(opMode);
        driveBase = new TankDriveBase(opMode);
        VuMark = new VuMark(opMode);
        intake = new Intake(opMode);
    }

    public static HalDashboard getDashboard() {
        return dashboard;
    }

}