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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

/**
 * This OpMode uses the common HardwareK9bot class to define the devices on the robot.
 * All device access is managed through the HardwareK9bot class. (See this class for device names)
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a basic Tank Drive Teleop for the K9 bot
 * It raises and lowers the arm using the Gampad Y and A buttons respectively.
 * It also opens and closes the claw slowly using the X and B buttons.
 *
 * Note: the configuration of the servos is such that
 * as the arm servo approaches 0, the arm position moves up (away from the floor).
 * Also, as the claw servo approaches 0, the claw opens up (drops the game element).
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Autonomous(name="Tricerabots: Autonomous Tank", group="Tricerabots")

public class AutonTricerabots extends LinearOpMode {
    
    private final double CUBE_HUE_UPPER_BOUND = Math.PI/6;
    private final double CUBE_HUE_LOWER_BOUND = 0;
    private final double SATURATION_LOWER_BOUND = .2;
    private int ambientr;
    private int ambientb;
    private int ambientg;
    //we can probably get rid of these as they can be replaced with the new ColorSensorStuff.
    
    private int lpos;
    private int rpos;
    private int elevatorpos;
    private int extendedpos;
    private int clawpos;
    
    private int i = 0;
    private boolean go = false;
    
    private ColorSensor colorSensor;
    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private DcMotor elevator;
    private DcMotor claw;
    
    //epic name
    private ColorSensorStuff collor = null;
    
    //movements represents a list of actions that the robot takes.
    //The first column represents the distance that the left motor must move and the second column for the right motor.
    private int[][] movements = new int[3][2];
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    /* Declare OpMode members. */
    HardwareTricerabots   robot           = new HardwareTricerabots();              // Use a K9'shardware
    
    
    private boolean detectCube(double saturation, double hue) { //dont need
        if (
            saturation > SATURATION_LOWER_BOUND && 
            hue > CUBE_HUE_LOWER_BOUND && 
            hue < CUBE_HUE_UPPER_BOUND
            ) {
            return true;
        } else {
            return false;
        }
    }
    
    private void calibrate() { //dont need
        ambientr = colorSensor.red();
        ambientb = colorSensor.blue();
        ambientg = colorSensor.green();
    }
    //these two methods create a quick way to add a movement to the movements array.
    //move(3000) = 26 in. 1 in = move(115.3)
    private int[] move(int dist) {
        int[] m = new int[2];
        m[0] = dist;
        m[1] = dist;
        return m;
    }
    //wheels have a lever arm of approx. r = 11.25 in /2 = 5.625 in.
    // 2pi * r = displacement = 35.34 in = turn(4075) to turn 360 degrees.
    //somthing went wrong... The wheels are slipping.
    private int[] turn(int dist) {
        int[] m = new int[2];
        m[0] = -dist;
        m[1] = dist;
        return m;
    }
    
    private void updateMotorPos() {
        leftDrive.setTargetPosition(lpos);
        rightDrive.setTargetPosition(rpos);
    }
    
    @Override
    public void runOpMode() {
        
        /*
        movements[0] = turn(900000);
        movements[1] = turn(500);
        movements[2] = move (1000);
        */
        
        
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        
        colorSensor = robot.colorSensor;
        leftDrive = robot.leftDrive;
        rightDrive = robot.rightDrive;
        elevator = robot.elevator;
        claw = robot.claw;
        
        leftDrive.setPower(1/2);
        rightDrive.setPower(1/2);
        
        //set up  motors for run to position mode and set target position to current position
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        claw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lpos = leftDrive.getCurrentPosition();
        rpos = rightDrive.getCurrentPosition();
        clawpos = claw.getCurrentPosition();
        updateMotorPos();
        
        //elevatorpos is the position at initiation which is when the motor is lowered.
        elevatorpos = robot.elevatorpos;
        //extendedpos is the position at which the elevator is extended which should be elevatorpos shifted by a certain amount.
        extendedpos = elevatorpos + 500;
        elevator.setTargetPosition(elevatorpos);
        
        collor = new ColorSensorStuff(colorSensor);
        
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            
            boolean busy = leftDrive.isBusy() || rightDrive.isBusy();
            if (!busy && i < movements.length && go) { //if the motors aren't busy and there is a movement avaliable and we are clear to move,
                lpos += movements[i][0];
                rpos += movements[i][1]; //set the target positions of the motor to that of the next movement so that the robot reaches the next position.
                i++; //and increment i
                updateMotorPos();
            }
            
            
            
            elevator.setTargetPosition(extendedpos);
            
            if (!elevator.isBusy()) {
                claw.setTargetPosition(clawpos - 270); //this will only fire when the elevator has fully extended so it is critical that we get this right.
                go = true;
            }
            
            // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
            
            //dont need this stuff
            int r = colorSensor.red() - ambientr;
            int g = colorSensor.green() - ambientg;
            int b = colorSensor.blue() - ambientb;
            
            // Send telemetry message to signify robot running;
            double[] color = collor.getColor();
            double h = color[0];
            double sat = color[1];
            
            telemetry.addData("hue", "%.2f", h);
            telemetry.addData("saturation", "%.2f", sat);
            
            telemetry.addData("isCube", detectCube(sat, h));
            telemetry.addData("elevator displacement", elevator.getCurrentPosition() - elevatorpos);
            
            /*
            telemetry.addData("lpos", leftDrive.getCurrentPosition());
            telemetry.addData("rpos", rightDrive.getCurrentPosition());
            */
            
            telemetry.update();
            
            // Pause for 40 mS each cycle = update 25 times a second.
            sleep(40); //we can change this number if we need somthing more precise.
        }
    }
}
