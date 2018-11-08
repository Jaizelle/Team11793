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
 I*
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
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.Hardware;
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

@TeleOp(name="Tricerabots: Telop Tank", group="Tricerabots")//this is no longer tank mode i guess

public class TeleopTricerabots extends LinearOpMode {
    
    private final double CUBE_HUE_UPPER_BOUND = Math.PI/6;
    private final double CUBE_HUE_LOWER_BOUND = 0;
    private final double SATURATION_LOWER_BOUND = .2;
    
    private int elevatorpos;
    private int extendedpos;
    

    /* Declare OpMode members. */
    HardwareTricerabots   robot = new HardwareTricerabots();  // Use a K9'shardware
        
               // Define and initialize ALL installed servos.
        /*
        GrabbingOn  = hwMap.get(Servo.class, "GrabbingOn");
        Hammer      = hwMap.get(Servo.class, "Hammer");
        Gate      = hwMap.get(Servo.class, "Gate");
        GrabbingOn.setPosition(GRAB_ON);
        Hammer.setPosition(HAMMER_HOME);
        Gate.setPosition(MARK_GATE);
        */
    /*
    double          armPosition     = robot.ARM_HOME;                   // Servo safe position
    double          clawPosition    = robot.CLAW_HOME;                  // Servo safe position
    final double    CLAW_SPEED      = 0.01 ;                            // sets rate to move servo
    final double    ARM_SPEED       = 0.01 ;                            // sets rate to move servo
*/
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
    

    @Override
    public void runOpMode() {
        double left;
        double right;
        
        

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        
        elevatorpos = robot.elevatorpos;
        extendedpos = elevatorpos + 1000;

        ColorSensor colorSensor = robot.colorSensor;
        DcMotor elevator = robot.elevator;
        
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();
        
        


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
            left = gamepad1.left_stick_y;
            right = gamepad1.right_stick_y;
            
            
            
            
            
            if (gamepad1.right_bumper) {
                elevator.setTargetPosition(extendedpos);
            } else if (gamepad1.right_trigger > .5) {
                elevator.setTargetPosition(elevatorpos);
            }
            
            robot.leftDrive.setPower(left);
            robot.rightDrive.setPower(right);
            
            int r = colorSensor.red();
            int g = colorSensor.green();
            int b = colorSensor.blue();
            
            double root = Math.sqrt(3);
            
            double h = Math.atan2(root * (g - b), 2 * r - g - b);
            double sat = Math.hypot(root * (g - b)/2 ,r - g/2 - b/2)/(r+g+b);
/*
            // Use gamepad Y & A raise and lower the arm
            if (gamepad1.a)
                armPosition += ARM_SPEED;
            else if (gamepad1.y)
                armPosition -= ARM_SPEED;
            // Use gamepad X & B to open and close the claw
            if (gamepad1.x)
                clawPosition += CLAW_SPEED;
            else if (gamepad1.b)
                clawPosition -= CLAW_SPEED;
            // Move both servos to new position.
            armPosition  = Range.clip(armPosition, robot.ARM_MIN_RANGE, robot.ARM_MAX_RANGE);
            robot.arm.setPosition(armPosition);
            clawPosition = Range.clip(clawPosition, robot.CLAW_MIN_RANGE, robot.CLAW_MAX_RANGE);
            robot.claw.setPosition(clawPosition);
            // Send telemetry message to signify robot running;
            telemetry.addData("arm",   "%.2f", armPosition);
            telemetry.addData("claw",  "%.2f", clawPosition);
            */
            
            telemetry.addData("left_drive",  "%.2f", left);
            telemetry.addData("right_drive", "%.2f", right);

            telemetry.addData("hue", "%.2f", h);
            telemetry.addData("saturation", "%.2f", sat);
            
            telemetry.addData("isCube", detectCube(sat, h));
            
            telemetry.update();
            

            // Pause for 40 mS each cycle = update 25 times a second.
            sleep(40);
        }
    }
}
