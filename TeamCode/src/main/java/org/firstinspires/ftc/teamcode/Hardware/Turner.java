package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by KusakabeMirai on 7/27/2018.
 */

public class Turner {

    public Servo turner1;
    public Servo turner2;
    public LinearOpMode opMode;

    public Turner(LinearOpMode opMode){
        this.opMode = opMode;
    }


}
