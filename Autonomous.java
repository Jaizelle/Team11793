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

package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous
public class Autonomous extends OpMode {
    private class Vector {
        public double x;
        public double y;

        public double angle() {
            return Math.atan2(x, y);
        }

        public double distance() {
            return Math.sqrt(Math.pow(this.x, 2) + Math.pow(this.y, 2));
        }

        public Vector(double newx, double newy) {
            x = newx;
            y = newy;
        }

        public static Vector polar(double r, double angle) {
            Vector v = new Vector(Math.cos(angle), Math.sin(angle));
            v = Scale(v, r);
            return v;
        }

        public static Vector Add(Vector a, Vector b) {
            Vector sum = new Vector(a.x + b.x, a.y + b.y);
            return sum;
        }

        public static Vector Scale(Vector a, double b) {
            Vector product = new Vector(a.x * b, a.y * b);
            return product;
        }

    }
    private class Movement {
        private double zeroTime;
        private double finalTime;
        private int leftK;
        private int rightK;
        private double displacement;
        private double turnAngle;
        private boolean linear;


        public Movement(boolean isLinear, double d) {
            if (isLinear) {
                displacement = d;
                leftK = 1;
                rightK = 1;
            }
            else {
                turnAngle = d;
                leftK = -1;
                rightK = 1;
                displacement = turnAngle * wheelLeverArm;
            }
            linear = isLinear;
        }
        public void Execute() {
            power = motorDisplacememt(displacement, runtime.seconds());
            leftDrive.setPower(leftK * power);
            rightDrive.setPower(rightK * power);
        }
        public void UpdateOrientation(){
            if (linear) {
                v = Vector.polar(displacement, angle);
                position += v;
            }
            else {
                angle += turnAngle;
            }
        }
        public void ChangeOrientation() { }

        public boolean isRunning(double t) {
            if (t > finalTime) return false;
            else return true;
        }
    }
    private class MoveForward extends Movement {
        private double displacement;

        public moveForward(double distance){
            displacement = distance * ratio / radius;
            finalTime = maneuverTime(displacement);
        }

        public void execute() {
            power = motorDisplacememt(displacement, runtime.seconds());
        }

        public void changeOrientation() {

        }

    }
    private class RotateRobot extends Movement{
        private double displacement;
        private double turnAngle;


        public rotateRobot(double angle) {
            displacement = angle * wheelLeverArm * ratio / radius;
            finalTime = maneuverTime(displacement);
        }

        public void execute() {
            power = motorDisplacememt(displacement, runtime.seconds());
            rightDrive.setPower(power);
            leftDrive.setPower(-power);
        }

        public void changeOrientation() {
            angle += turnAngle;
        }
    }
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private double radius = .09; //radius of the wheel measured in meters. the wheel is 90 mm
    private double ratio = 5/4; //the gear ratio is 5/4. this is multiplied by any angle that we want the wheel to rotate in order to calculate the angle that the motor must rotate.
    private double wheelLeverArm = 0; //the distance each wheel is from the center of mass or the distance between both wheels divided by two probably measured in meters because SI units master race.
                    //need measurement
    private double a = 0;
    private double vMax = 0;
    private Vector position = new Vector(0, 0);
    private double angle = 0;
    private Movement[] movements;
    private double constantPower;

    private double maneuverTime(double displacement) {
        if (a * displacement > vMax * vMax) {
            return displacement / vMax + vMax / a;
        } else {
            return 2 * Math.sqrt(displacement / a);
        }
    }

    private double motorDisplacememt(double displacement, double t) {
        if (a * displacement > vMax * vMax) {
            double tFinal = displacement / vMax + vMax / a;
            //accelerate until maxVelocity then maintain maxVelocity until stopping distance
            if (a * t < vMax) return a * t / vMax;
            if (vMax < t * a && t * a < tFinal * a - vMax) return 1.0d;
            if (tFinal - vMax / a < t && t < tFinal) return a * (tFinal - t) / vMax;
            if (t > tFinal) return 0;
        } else {
            double tFinal = 2 * Math.sqrt(displacement / a);
            if (t < tFinal / 2) return a * t / vMax;
            if (t > tFinal/2 && t < tFinal) return a * (tFinal - t) / vMax;
            if (t > tFinal) return 0;
        }
    }

    private void turnByRadians(double angle, double t) {
        //rotate the robot by this many radians ccw (negative for cw).
        dAngle = angle * wheelLeverArm * ratio / radius;
        power = motorDisplacememt(dAngle , t);
        rightDrive.setPower(power);
        leftDrive.setPower(-power);
    }//i dont think i need this
/*
    private void moveForward(double displacement, double t) { //probably dont need this

    }
*/
    /*
     * Code to run ONCE when the driver hits INIT
     */

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");


        leftDrive.RunMode = RUN_TO_POSITION;
        rightDrive.RunMode = RUN_TO_POSITION;

        leftDrive.setTargetPosition(100);
        leftDrive.setPower(0);

        rightDrive.setTargetPosition(100);
        rightDrive.setPower(0);



        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        /*
        power = motorDisplacememt(10, runtime.seconds());
        leftDrive.setPower(power);
        rightDrive.setPower(power);
        */


        leftDrive.setPower(constantPower);
        rightDrive.setPower(constantPower);

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("x", position.x);
        telemetry.addData("y", position.y);
        telemetry.addData("orientation", angle);
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    private void LowerTheArm(){

    }




}
