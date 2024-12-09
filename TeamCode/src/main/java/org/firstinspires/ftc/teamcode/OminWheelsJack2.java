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

@TeleOp(name="OminWheels 0.1", group="Linear OpMode")
    
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

        public class AutoWithHardware extends LinearOpMode {

        RobotHardware   robot      =    new RobotHardware(this);
    
public class OminWheelsJack extends LinearOpMode {

    double drive  = 0.0;
    double strafe = 0.0;
    double turn   = 0.0;

    double INCH_TO_TICK = 20.0; // Untested
    double TICK_TO_INCH = 0.05; // Also untested
   
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
         public class AutoWithHardware extends LinearOpMode {

        RobotHardware   robot      =    new RobotHardware(this);
    
public class OminWheelsJack extends LinearOpMode {
                telemetry.addData("MotorSpeed", thatMotor.getSpeed());
            }
            
            for (Servo thatServo:allServos){
                telemetry.addData("ServoPosition", thatServo.getSpeed());
            }
            telemetry.update();
        }
    }}
