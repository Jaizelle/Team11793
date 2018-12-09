/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a K9 robot.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Servo channel:  Servo to raise/lower arm: "arm"
 * Servo channel:  Servo to open/close claw: "claw"
 *
 * Note: the configuration of the servos is such that:
 *   As the arm servo approaches 0, the arm position moves up (away from the floor).
 *   As the claw servo approaches 0, the claw opens up (drops the game element).
 */
public class HardwareTricerabots
{
    /* Public OpMode Members */
    public  DcMotor  leftDrive    = null ;
    public  DcMotor  rightDrive   = null ;
    public  DcMotor  elevator = null;
    public DcMotor claw = null;
    public Servo servo = null;
    public DistanceSensor distSensor = null;
    
    
    public ColorSensor colorSensor = null;
    
    public BNO055IMU imu;
    
    public int elevatorpos;
    public int extendedpos;
    public int elevatordpos = 465;

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
    public HardwareTricerabots() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        //all elevator related initialization will be done by the init method in the "EleHW" object.
        // save reference to HW Map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftDrive  = hwMap.get(DcMotor.class, "left_drive");
        rightDrive = hwMap.get(DcMotor.class, "right_drive");
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        
        elevator = hwMap.get(DcMotor.class, "elevator");
        claw = hwMap.get(DcMotor.class, "claw");
        colorSensor = hwMap.get(ColorSensor.class, "color_sensor");
        servo = hwMap.get(Servo.class, "servo");
        distSensor = hwMap.get(DistanceSensor.class, "dist_sensor");
        servo.setDirection(Servo.Direction.REVERSE);
        

        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        elevator.setPower(1);
        claw.setPower(0);
        
      

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        claw.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        elevatorpos = elevator.getCurrentPosition();
        extendedpos = elevatorpos + elevatordpos;//initially this constant was 710. at the time, we were using a gear ratio for torque of 60/45 = 4/3. 710 * 3 / 4 = total distance = x * 1. x = 710*3/4 = 532.5 ~ 533. This may need to be adjusted but it is probably smarter to remove the stopper.
        //this number is just an estimate and may need some adjusting.

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
