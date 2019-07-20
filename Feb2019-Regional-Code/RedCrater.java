package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@Autonomous(name = "RedCrater", group = "Auto")

public class RedCrater extends AutoCommon {
    
    @Override
    public void runOpMode() throws InterruptedException { 
        
        //initialize motors and sensors
        initializeRobot();
        
        initializeIMU();
        
        waitForStart();

        while (opModeIsActive()) {
            double turn = 0.25;
            
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            float angleOne = angles.firstAngle;
            telemetry.addData("Angle", angles.firstAngle);
            telemetry.update();
            
            //1. come down
            actuatorLift(9000, true);
            
            //2. auto correct
            //while coming down the robot rotates, so auto correction corrects it
            float angleTwo = performCorrection(angleOne);
            sleep(1000);
            
            initializeEncoders();

            //3. drive backward
            moveUsingEncoder(-400, 0.5);
            

            //4. strafe forward initial
            encoderStrafe(0.5, 1000, angleOne);
            
            //4.1 drive backward
            moveUsingEncoder(400, 0.5);

            //4.2 strafe forward initial
            encoderStrafe(0.5, 2500, angleOne);
        
            //5 turn so it touches the crater
            disableEncoders();
            turn(0.5, 2000);
            mechanumDrive(0.5,0.5,0.5,0.5,1000);
        
            break;
        }
    }
}
