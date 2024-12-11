package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

var INCH = 500;
    
public class RobotHardware {
    
    public LinearOpMode myOpMode = null;
    
    // Declare OpMode members
    public ElapsedTime runtime = new ElapsedTime();
    
    public DcMotor leftFrontWheel = null; //Motors to control all wheels
    public DcMotor leftBackWheel = null;
    public DcMotor rightFrontWheel = null;
    public DcMotor rightBackWheel = null;

    public DcMotor leftLeg = null;
    public DcMotor rightLeg = null;
    
    public DcMotor spiralLift = null;
    public DcMotor spiralBrush = null;


    
    public RobotHardware (LinearOpMode opmode) {
        myOpMode = opmode;
    }


    
    public void init()    {
        
        leftFrontWheel  = myOpMode.hardwareMap.get(DcMotor.class, "leftFront");
        leftBackWheel   = myOpMode.hardwareMap.get(DcMotor.class, "leftBack");
        rightFrontWheel = myOpMode.hardwareMap.get(DcMotor.class, "rightFront");
        rightBackWheel  = myOpMode.hardwareMap.get(DcMotor.class, "rightBack");
       
        leftFrontWheel.setDirection(DcMotor.Direction.REVERSE);
        leftBackWheel.setDirection(DcMotor.Direction.REVERSE);
        rightFrontWheel.setDirection(DcMotor.Direction.FORWARD);
        rightBackWheel.setDirection(DcMotor.Direction.FORWARD);

        leftFoot        = myOpMode.hardwareMap.get(DcMotor.class, "myLeftFoot");
        rightFoot       = myOpMode.hardwareMap.get(DcMotor.class, "myRightFoot");

        spiralLift      = myOpMode.hardwareMap.get(DcMotor.class, "archimedes");
        spiralBrush     = myOpMode.hardwareMap.get(DcMotor.class, "brush");
        
        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();

        robot.driveRobot (1, 0, 0);
        Sleep (INCH*60);
        //grab sample code goes here
        robot.driveRobot (0, 0, 1);
        Sleep (INCH*60);    
        robot.driveRobot (0, 0, 0);
        robot.liftScrew (16);
        Sleep (2500);
        robot.toggleDepositDoor();
        Sleep (500);
        robot.liftScrew (-16);
        Sleep (2500);
        robot.driveRobot (0, 0, 1);
        Sleep (INCH*30); 
        
    }
}
