package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

var INCH = 82;
    
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

        //Drive to sample
        robot.driveRobot (1, 0, 0);
        Sleep (INCH*30);
        robot.driveRobot (0, 0, 0);
        Sleep (100);

        //Pick up sample
        robot.toggleSweeper();
        Sleep (500);
        robot.toggleSweeper();
        Sleep (100);

        //Turn towards net
        robot.driveRobot (0, 0, 1);
        Sleep (INCH*30);    
        robot.driveRobot (0, 0, 0);
        Sleep (100);

        //Go to net
        robot.driveRobot (1, 0, 0);
        Sleep (INCH*60);    
        robot.driveRobot (0, 0, 0);
        Sleep (100);
        
        //Deposit sample
        robot.liftScrew (7.5);
        robot.standUp(7.5);
        robot.toggleDepositDoor();
        robot.setScrewPower(100);
        Sleep (1000);
        robot.setScrewPower(0);
        robot.toggleDepositDoor();
        robot.standUp(-7.5);
        robot.liftScrew (-7.5);
        Sleep (1000);
        
        //Turn towards other sample
        robot.driveRobot (0, 0, 1);
        Sleep (INCH*30); 
        robot.driveRobot (0, 0, 0);
        Sleep (100);

        //Go to other sample
        robot.driveRobot (1, 0, 0);
        Sleep (INCH*17); 
        robot.driveRobot (0, 0, 0);
        Sleep (100);

        //Pick up sample
        robot.toggleSweeper();
        Sleep (500);
        robot.toggleSweeper();
        Sleep (100);

        //Turn back towards net
        robot.driveRobot (0, 0, 1);
        Sleep (INCH*30); 
        robot.driveRobot (0, 0, 0);
        Sleep (100); 

        //Move back to net
        robot.driveRobot (1, 0, 0);
        Sleep (INCH*48);
        robot.driveRobot (0, 0, 0);
        Sleep (100); 

        //Deposit other sample
        robot.liftScrew (7.5);
        robot.standUp(7.5);
        robot.toggleDepositDoor();
        robot.setScrewPower(.75);
        Sleep (1000);
        robot.setScrewPower(0);
        robot.toggleDepositDoor();
        robot.standUp(-7.5);
        robot.liftScrew (-7.5);
        Sleep (1000);

        //Turn towards observatory
        robot.driveRobot (0, 0, 1);
        Sleep (INCH*30); 
        robot.driveRobot (0, 0, 0);
        Sleep (100);  

        //Go to observatory
        robot.driveRobot (1, 0, 0);
        Sleep (INCH*5.1); 
        robot.driveRobot (0, 0, 0);
        Sleep (100); 

        //Speen
        robot.driveRobot (0, 0, 1);

        //Prevent an AI uprising
            // if (sentience==true){
            //     delete robot;
            // }
    }
}
