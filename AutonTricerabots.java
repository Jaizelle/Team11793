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
    private int lpos;
    private int rpos;
    
    private int i = 0;
    
    private ColorSensor colorSensor;
    private DcMotor leftDrive;
    private DcMotor rightDrive;
    
    private int[][] movements = new int[2][2];
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    /* Declare OpMode members. */
    HardwareTricerabots   robot           = new HardwareTricerabots();              // Use a K9'shardware
    
    
    private boolean detectCube(double saturation, double hue) {
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
    
    private void calibrate() {
        ambientr = colorSensor.red();
        ambientb = colorSensor.blue();
        ambientg = colorSensor.green();
    }
    
    private int[] move(int dist) {
        int[] m = new int[2];
        m[0] = dist;
        m[1] = dist;
        return m;
    }
    
    private void updateMotorPos() {
        leftDrive.setTargetPosition(lpos);
        rightDrive.setTargetPosition(rpos);
    }
    
    @Override
    public void runOpMode() {
        
        movements[0] = move(1000);
        
        
        
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        
        colorSensor = robot.colorSensor;
        leftDrive = robot.leftDrive;
        rightDrive = robot.rightDrive;
        
        leftDrive.setPower(1);
        rightDrive.setPower(1);
        
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lpos = leftDrive.getCurrentPosition();
        rpos = -rightDrive.getCurrentPosition();
        
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            boolean busy = leftDrive.isBusy() || rightDrive.isBusy();
            if (!busy && i < movements.length) {
                lpos += movements[i][0];
                rpos += movements[i][1];
                i++;
                
                updateMotorPos();
            }
            
            
            
            // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
            
            int r = colorSensor.red() - ambientr;
            int g = colorSensor.green() - ambientg;
            int b = colorSensor.blue() - ambientb;
            
            double root = Math.sqrt(3);
            
            double h = Math.atan2(root * (g - b), 2 * r - g - b);
            double sat = Math.hypot(root * (g - b)/2 ,r - g/2 - b/2)/(r+g+b);
            
            // Send telemetry message to signify robot running;
            
             telemetry.addData("hue", "%.2f", h);
            telemetry.addData("saturation", "%.2f", sat);
            
            telemetry.addData("isCube", detectCube(sat, h));
            telemetry.addData("lpos", leftDrive.getCurrentPosition());
            telemetry.addData("rpos", rightDrive.getCurrentPosition());
            
            telemetry.update();

            // Pause for 40 mS each cycle = update 25 times a second.
            sleep(40);
        }
    }
}
