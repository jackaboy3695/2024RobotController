/* Copyright (c) 2021 FIRST. All rights reserved.
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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;
import java.util.ArrayList;
#include <PRIZM.h>  // Include all of the instructions for Tetrix stuff
PRIZM robot;

@TeleOp(name="OminWheels 0.1", group="Linear OpMode")
@Disabled
public class OminWheelsJack extends LinearOpMode {

    // Declare OpMode members for each of the 4 wheel motors, the two shoulder motors, the extender motors, and the hand servos.
    
    //Time
    private ElapsedTime runtime = new ElapsedTime();
    
    //Drive
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    
    //Arms
    private DcMotor leftShoulder = null;
    private DcMotor leftExtender = null;
    private DcMotor rightShoulder = null;
    private DcMotor rightExtender = null;
    
    //Hands
    private Servo leftHand = null;
    private Servo rightHand = null;
    
    //Lists
    List<DcMotor> allMotors = new ArrayList<>();
    List<Servo> allServos = new ArrayList<>();
    
    @Override
    
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        
        //Drive
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "right_back_drive");
        
        //Arms
        leftShoulder  = hardwareMap.get(DcMotor.class, "left_shoulder");
        leftExtender  = hardwareMap.get(DcMotor.class, "left_extender");
        rightShoulder = hardwareMap.get(DcMotor.class, "right_shoulder");
        rightExtender = hardwareMap.get(DcMotor.class, "right_extender");

        allMotors.add(leftFrontDrive);
        allMotors.add(leftBackDrive);
        allMotors.add(rightFrontDrive);
        allMotors.add(rightBackDrive);
        allMotors.add(leftShoulder);
        allMotors.add(leftExtender);
        allMotors.add(rightShoulder);
        allMotors.add(rightExtender);
        
        //Servos
        leftHand  = hardwareMap.get(Servo.class, "left_hand");
        rightHand = hardwareMap.get(Servo.class, "right_hand");

        allServos.add(leftHand);
        allServos.add(rightHand);
        
        //Drive
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        
        //Arms
        leftShoulder.setDirection(DcMotor.Direction.REVERSE);
        leftExtender.setDirection(DcMotor.Direction.REVERSE);
        rightShoulder.setDirection(DcMotor.Direction.FORWARD);
        rightExtender.setDirection(DcMotor.Direction.FORWARD);
        
        //Servos
        leftHand.setDirection(Servo.Direction.REVERSE);
        rightHand.setDirection(Servo.Direction.FORWARD);
      
        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "space", "space");
        telemetry.update();

        waitForStart();
        runtime.reset();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // Servo variables
            final double INCREMENT = 0.0;
            final int CYCLEMS = 50;
            final double MAX_POS = 1.0;
            final double MIN_POS = 0.0;
            
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial           = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value - Forward/Backward
            double lateral         =  gamepad1.right_stick_x; // Note: Strafe
            double yaw             =  gamepad1.left_stick_x; //Note: Turn
            
            // Arms
            boolean rightGrab      =  gamepad2.right_bumper;
            boolean leftGrab       =  gamepad2.left_bumper;
            float rightExtension   =  gamepad2.right_trigger;
            float leftExtension    =  gamepad2.left_trigger;
            float leftArmAngle     = -gamepad2.left_stick_y;
            float rightArmAngle    = -gamepad2.right_stick_y;
            
            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100% to ensure that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            } 
            
            // Hand controls
            if (leftGrab){
                leftHand.setPosition(0);
            } else {
                leftHand.setPosition(1);
            }
            if (rightGrab){
                rightHand.setPosition(0);
            } else {
                rightHand.setPosition(1);
            }
            
            // Arm controls
            robot.setMotorTarget (leftShoulder,100,leftArmAngle);
            robot.setMotorTarget (rightShoulder,100,rightArmAngle);
            robot.setMotorTarget (leftExtender,100,leftExtension);
            robot.setMotorTarget (rightExtender,100,rightExtension);
            
            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            for (DcMotor thatMotor:allMotors){
                telemetry.addData("MotorSpeed", thatMotor.getSpeed());
            }
            
            for (Servo thatServo:allServos){
                telemetry.addData("ServoPosition", thatServo.getSpeed());
            }
            telemetry.update();
        }
    }}

/*
OmniWheelsJack.java
@TeleOp(name="OminWheels 0.1", group="Linear OpMode")


Build started at Thu Oct 17 2024 14:19:35 GMT-0700 (Pacific Daylight Time)
org/firstinspires/ftc/teamcode/OmniWheelsJack.java line 40, column 8: ERROR: class OminWheelsJack is public, should be declared in a file named OminWheelsJack.java
org/firstinspires/ftc/teamcode/OmniWheelsJack.java line 100, column 23: ERROR: cannot find symbol
  symbol:   variable righttHand
  location: class org.firstinspires.ftc.robotcontroller.external.samples.OminWheelsJack
org/firstinspires/ftc/teamcode/OmniWheelsJack.java line 179, column 13: ERROR: cannot find symbol
  symbol:   method setMotorTarget(com.qualcomm.robotcore.hardware.DcMotor,int,float)
  location: class org.firstinspires.ftc.robotcontroller.external.samples.OminWheelsJack
org/firstinspires/ftc/teamcode/OmniWheelsJack.java line 180, column 13: ERROR: cannot find symbol
  symbol:   method setMotorTarget(com.qualcomm.robotcore.hardware.DcMotor,int,float)
  location: class org.firstinspires.ftc.robotcontroller.external.samples.OminWheelsJack
org/firstinspires/ftc/teamcode/OmniWheelsJack.java line 181, column 13: ERROR: cannot find symbol
  symbol:   method setMotorTarget(com.qualcomm.robotcore.hardware.DcMotor,int,float)
  location: class org.firstinspires.ftc.robotcontroller.external.samples.OminWheelsJack
org/firstinspires/ftc/teamcode/OmniWheelsJack.java line 182, column 13: ERROR: cannot find symbol
  symbol:   method setMotorTarget(com.qualcomm.robotcore.hardware.DcMotor,int,float)
  location: class org.firstinspires.ftc.robotcontroller.external.samples.OminWheelsJack
org/firstinspires/ftc/teamcode/OmniWheelsJack.java line 191, column 58: ERROR: cannot find symbol
  symbol:   method getSpeed()
  location: variable thatMotor of type com.qualcomm.robotcore.hardware.DcMotor
org/firstinspires/ftc/teamcode/OmniWheelsJack.java line 195, column 61: ERROR: cannot find symbol
  symbol:   method getSpeed()
  location: variable thatServo of type com.qualcomm.robotcore.hardware.Servo

Build FAILED!

Build finished in 0.8 seconds
    */
