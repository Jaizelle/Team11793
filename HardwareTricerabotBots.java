package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class HardwareTricerabotBots
{
    /* Public OpMode Members */
    public  DcMotor  leftDrive    = null ;
    public  DcMotor  rightDrive   = null ;

    /*
    public  Servo   GrabbingOn    = null ;
    public  Servo   Hammer        = null ;
    public  Servo   Gate          = null ;


    public final static double GRAB_ON = 0.2;
    public final static double HAMMER_HOME = 0.2;
    public final static double MARK_GATE    = 0.2;
    public final static double GRAB_MIN_RANGE  = 0.20;
    public final static double GRAB_MAX_RANGE  = 0.90;
    public final static double HAMMER_MIN_RANGE  = 0.20;
    public final static double HAMMER_MAX_RANGE  = 0.7;
    public final static double MARK_MIN_RANGE    = 0.2;
    public final static double MARK_MAX_RANGE    = 0.9;
    */


    /* Local OpMode members. */
    HardwareMap hwMap  = null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareTricerabotBots () {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // save reference to HW Map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftDrive  = hwMap.get(DcMotor.class, "left_drive");
        rightDrive = hwMap.get(DcMotor.class, "right_drive");
        leftDrive.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // Define and initialize ALL installed servos.
        /*
        GrabbingOn  = hwMap.get(Servo.class, "GrabbingOn");
        Hammer      = hwMap.get(Servo.class, "Hammer");
        Gate      = hwMap.get(Servo.class, "Gate");
        GrabbingOn.setPosition(GRAB_ON);
        Hammer.setPosition(HAMMER_HOME);
        Gate.setPosition(MARK_GATE);
        */





    }
}
}
