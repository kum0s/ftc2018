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

@Autonomous(name = "RedDepot", group = "Auto")

public class RedDepot extends AutoCommon {
    
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
            
            //2. auto correct - 1
            //while coming down the robot rotates, so auto correction corrects it
            float angleTwo = performCorrection(angleOne);
            sleep(100);
            
            //3. auto correct - 2
            //go back to original direction which is at 45
            angleTwo = performCorrection(45);
            sleep(100);
            
            initializeEncoders();
            
            //4 strafe forward initial
            encoderStrafe(0.8, 1000, angleOne);
            
            //4.1 go backward
            moveUsingEncoder(-400, 0.5);
            
            //4.2 strafe final
            encoderStrafe(0.8, 5000, angleOne);
        
            disableEncoders();
            
            //5. turn by 0
            angleTwo = performCorrection(0);

            
            //6. go slightly towards the wall
            initializeEncoders();
            encoderStrafe(0.8, 1000, angleOne);
            
            disableEncoders();
            
            //7. do adjustments to make sure we are still at 0 angle
            angleTwo = performCorrection(angleOne);

            
            //8. drop the team marker
            turnIntake(0.25, 500);
            sleep(200);
            turnIntake(-0.25, 500);
            
            disableEncoders();
            
            //9. do adjustments to make sure we are still at 45 angle
            angleTwo = performCorrection(angleOne);
            
            
            //10. move back towards crater
            sleep(1000);
            initializeEncoders();
            moveUsingEncoder(-6500, 1, angleOne);

            break;
        }
    }
}
