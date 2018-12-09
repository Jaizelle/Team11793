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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
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
    
    private int lpos;
    private int rpos;
    private int elevatorpos;
    private int extendedpos;
    private int clawpos;
    private int clawdisp = 270;
    
    private double[][] position = new double[1][2];
    private double angle = 0;
    private double[][] displacement = new double[1][2];
    private double nextAngle = 0;
    
    private int i = 0;
    
    private ColorSensor colorSensor;
    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private DcMotor elevator;
    private DcMotor claw;
    private Servo servo;
    private DistanceSensor distSensor;
    
    private final double RATIO = 4743/(2*Math.PI*14.3);//linear ratio between encoder readings and distance.
    private final double TURNRATIO = 4743/(Math.PI * 2);//The ratio between encoder readings and angle turned when the motors are running in opposite directions.
    
    
    //epic name
    private ColorSensorStuff collor = null;
    
    //movements represents a list of actions that the robot takes.
    //The first column represents the distance that the left motor must move and the second column for the right motor.
    private int[][] movements = new int[3][2];
    private Matricies matricies = Matricies.getInstance();
    
    
    
    
    
    
    //here is what detecting the cube would look like:
    private void findCube(){
        int disptotal = 9001; //= some total displacement or the maximum distance the robot is willing to go while looking for the cube. Check this value The robot might go way too far.
        leftDrive.setPower(.1);
        rightDrive.setPower(.1);
        setTarget(move(disptotal));
        updateMotorPos();
        boolean cube = false;
        while (!cube && opModeIsActive()) { // the robot will look for a cube every frame while the motors move at a constant speed.
            cube = collor.detectCuble();
            sleep(10); //some small number
        }
        if (cube) {
            //we have found a cube. We now want to knock it away and then roll through the remaining distance and continue our route.
            telemetry.addData("found the cube", " ");
            telemetry.update();
            lpos = leftDrive.getCurrentPosition();
            rpos = rightDrive.getCurrentPosition();
            leftDrive.setPower(.2);
            rightDrive.setPower(.2); //reset the speed
            setTarget(turnAngle(Math.PI/2));
            updateMotorPos();
        }
    }
    
    private double[][] scan(double angle) { //the goal is to return a table of all the readings as the robot rotates of the distance from the place that the robot's sensor is pointing to the robot's position (the midpoint of the two wheels) as a functon of the current position of the robot which is calculated by the initial angle plus the reading on the encoders divided by the turn ratio (in radians) as the robot rotates accross a certain angle.
        //I dont know why im not setting this up the same way i set up findCube() but IDC
        setTarget(turnAngle(angle));
        updateMotorPos();
        int lposi = leftDrive.getCurrentPosition();
        double[][] table = new double[50][2]; //has to be a large number
        double anglei = matricies.angle;
        int pointer = 0;
        while(opModeIsActive() && (leftDrive.isBusy() || rightDrive.isBusy())) {
            double dAngle = (leftDrive.getCurrentPosition()-lposi)/TURNRATIO;
            angle = anglei + dAngle;
            double reading = distSensor.getDistance(DistanceUnit.CM);
            double[][] r = matricies.add(
                matricies.scale(reading, matricies.getDistSensorForward(angle)),
                matricies.getDistSensorPos(angle)
            ); //calculates the vector from the midpoint of the wheels to the pointed position.
            table[pointer][0] = Math.atan2(r[0][0], r[1][0]);//set the table elements to the angle of the r vector and the magnitude of the r vector respectively
            table[pointer][1] = Math.hypot(r[0][0], r[1][0]);
            pointer++;
        }
        return table;
    }
    
    private double[] changeTable(double[][] table) { //calculates the difference between each table element's distance. We know we've hit a cube if the difference is a large positive number
        int pointer = 0;
        double[] change = new double[table.length - 1];
        while (table[pointer][1] != 0) {
            change[pointer] = table[pointer + 1][1] - table[pointer][1]; //calculate the change. if change[i] is a large positive number then table[i+1][1] > table[i][1]
            pointer++;
        }
        return change;
    }
    
    private int[] getPoints(double[] changeTable) { //finds the locations of the gameobjects of the cube assuming that the minimum distance from nearby objects is 2.5cm
        int pointer = 0;
        int i = 0;
        int[] points = new int[10]; //more than we need.
        while (changeTable[pointer] != 0) {
            if (changeTable[pointer] > 2.5d) { //if the change in values is greater than 2.5cm then asume that we've found a game object
                points[i] = pointer;
                i++;
            }
            pointer++;
        }
        return points;
    }
    /*
    private double[][][] getGameObjects(int[] points, double[][] table){ //returns the locations of the game objects as vectors
        for (int i = 0; i < points.length; i++) {
            double r = table[points[i]][1];
            
            double[][] vector = table[points[i]];
            gameObjects[i] = matricies.multiply(
                matricies.rotateTransform(vector[0])
            );
        }
    } //WIP
    */
    
    private void setTarget(int[] pos) { //sets the next target position of both motors
        lpos += pos[0];
        rpos += pos[1];
    }
    
    
    
    
    
    
    
    
    
    //these two methods create a quick way to add movements to the movements array.
    private int[] move(int dist) {
        int[] m = new int[2];
        m[0] = dist;
        m[1] = dist;
        return m;
    }
    //wheels have a lever arm of approx. r = 11.25 in /2 = 5.625 in.
    private int[] turn(int dist) {
        int[] m = new int[2];
        m[0] = -dist;
        m[1] = dist;
        return m;
    }
    
    private int[] moveCM (double cm) { //moves the robot forward and updates the position of the robot according to the nextAngle
        Long m = new Long(Math.round(cm * RATIO)); //technically, i should be rounding first then taking the intValue()
        int measure = m.intValue();
        double[][] displacement = matricies.scale(measure / RATIO, matricies.getForward(nextAngle)); //why isnt there operator overloading in java
        this.displacement = matricies.add(this.displacement, displacement);
        return move(measure);
    }
    
    private int[] turnAngle(double angle) { //adds a turn and changes nextAngle
        Double m = new Double(angle * TURNRATIO);
        int measure = Math.round(m.floatValue());
        double dAngle = measure / TURNRATIO;
        nextAngle += dAngle;
        return turn(measure);
    }
    
    private void updateMotorPos() { //sets target position of both motors
        leftDrive.setTargetPosition(lpos);
        rightDrive.setTargetPosition(rpos);
    }
    
    private void updateRobotPos() { //sets the new position and angle to the angle and position that was calculated as a result of moving
        angle = nextAngle;
        position = matricies.add(position, displacement);
        displacement = matricies.vector(0.0d, 0.0d);
    }
    
        HardwareTricerabots   robot           = new HardwareTricerabots();     // Use a Tricerabots'shardware
    
    @Override
    public void runOpMode() {
        
        //movements[0] = move(5000); //if we are going to claim, the easiest way to do so is by driving straight for the depot and droping our object.
        movements[0] = move(3378); //lets make this number more precise.
        movements[1] = turn(2378);
        movements[2] = move(3378);
        //turn(10000) = 3*180-30 = 540 - 30 = 510 degrees.
        //1 degree = turn(10000/510) = turn(19.6)
        //360 degrees = turn(19.6 * 360) = turn(7059)
        //10 * 360 degrees = turn (16.6 * 360 * 10) = turn(70588)
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        
        colorSensor = robot.colorSensor;
        leftDrive = robot.leftDrive;
        rightDrive = robot.rightDrive;
        elevator = robot.elevator;
        claw = robot.claw;
        servo = robot.servo;
        distSensor = robot.distSensor;
        
        leftDrive.setPower(.2);
        rightDrive.setPower(.2);
        claw.setPower(1);
        
        //set up  motors for run to position mode and set target position to current position
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        claw.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lpos = leftDrive.getCurrentPosition();
        rpos = rightDrive.getCurrentPosition();
        clawpos = claw.getCurrentPosition();
        updateMotorPos();
        
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //elevatorpos is the position at initiation which is when the motor is lowered.
        elevatorpos = robot.elevatorpos;
        //extendedpos is the position at which the elevator is extended which should be elevatorpos shifted by a certain amount.
        extendedpos = robot.extendedpos;
        elevator.setTargetPosition(elevatorpos);
        
        collor = new ColorSensorStuff(colorSensor);
        
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();
        
        // Wait for the game to start (driver presses PLAY)
        /*
        int[] turn = turn(70588);
        leftDrive.setTargetPosition(turn[0]);
        rightDrive.setTargetPosition(turn[1]);
        updateMotorPos();
        */
        waitForStart();
        
        findCube();
        
        
        elevator.setTargetPosition(extendedpos);
        sleep(3000);
        claw.setTargetPosition(clawpos + clawdisp);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sleep(100);
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            boolean busy = leftDrive.isBusy() && rightDrive.isBusy();
            if (!busy && (i < movements.length)) { //if the motors aren't busy and there is a movement avaliable,
                setTarget(movements[i]);//set the target positions of the motor to that of the next movement so that the robot reaches the next position.
                i++; //and increment i
                updateMotorPos();
            }
            
            if (!busy && i == movements.length) {
                //move our servo
                servo.setPosition(-90);
            }
            
            // Send telemetry message to signify robot running;
            double[] color = collor.getColor();
            double h = color[0];
            double sat = color[1];
            
            telemetry.addData("hue", "%.2f", h);
            telemetry.addData("saturation", "%.2f", sat);
            telemetry.addData("i", i);
            telemetry.addData("busy", busy);
           
            telemetry.addData("elevator displacement", elevator.getCurrentPosition() - elevatorpos);
            telemetry.update();
            
            // Pause for 40 mS each cycle = update 25 times a second.
            //1/25 = .2/5 = .04
            //10 ms = .01s
            //1s = 10 * 100 = 1000ms
            sleep(40); //we can change this number if we need somthing more precise.
        }
    }
}
