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

@Autonomous(name = "Crater", group = "Auto")
//@Disabled
public class Crater extends AutoCommon {
    
    @Override
    public void runOpMode() throws InterruptedException { 
        
        //initialize motors and sensors
        initializeRobot();
        
        initializeIMU();
        
        initialiseVuForiaAndTensorFlow();
        
        telemetry.addData("", "Ready for Start");
        telemetry.update();
        
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
            //+20 degrees is to make sure that notch can come out cleanly
            //Without adding 20 degrees, it jams up
            float angleTwo = performCorrection(angleOne + 20);
            sleep(100);

            //3. Go back and unhook
            strafeDiagonal(0.5, 250);
            
            //4.. get Gold position
            GoldPosition goldPosition = getGoldPosition();
            if(goldPosition == GoldPosition.LEFT) {
                telemetry.addData("Gold Mineral Position", "LEFT");    
            } else if (goldPosition == GoldPosition.RIGHT){
                telemetry.addData("Gold Mineral Position", "RIGHT");    
            } else if (goldPosition == GoldPosition.CENTER){
                telemetry.addData("Gold Mineral Position", "CENTER");    
            } else if (goldPosition == GoldPosition.NONE){
                telemetry.addData("Gold Mineral Position", "NONE");    
                goldPosition = GoldPosition.LEFT;
            }
            telemetry.update();
            
            
            //5. auto correct - 2
            //go back to original direction which is at 45
            angleTwo = performCorrection(45);
            sleep(100);
            
            //6. strafe forward 
            initializeEncoders();
            encoderStrafe(0.8, 2000, 45);
            sleep(100);
            
            if (goldPosition == GoldPosition.LEFT) {
                //1. go back
                moveUsingEncoder(-1000, 0.5, 45);
                
                //2. align with lander
                disableEncoders();
                angleTwo = performCorrection(45);
                sleep(100);

                //3. strafe to displace gold
                initializeEncoders();
                encoderStrafe(0.6, 1500, 45);
                
                //4. turn so it touches the crater
                disableEncoders();
                turn(0.5, 2000);
                mechanumDrive(0.5,0.5,0.5,0.5,1000);
            } else if (goldPosition == GoldPosition.CENTER) {
                //1. go forward
                moveUsingEncoder(500, 0.5, 45);
                sleep(100);
                
                //2. align with lander
                disableEncoders();
                angleTwo = performCorrection(45);
                sleep(100);

                //3. strafe to displace gold
                initializeEncoders();
                encoderStrafe(0.6, 1500, 45);
                
                //4. turn so it touches the crater
                disableEncoders();
                turn(0.5, 2000);
                mechanumDrive(0.5,0.5,0.5,0.5,1000);
            } else if (goldPosition == GoldPosition.RIGHT) {
                //0. strafe forward a bit more
                initializeEncoders();
                encoderStrafe(0.8, 500, 45);
                sleep(50);
                
                //1. go forward
                moveUsingEncoder(2000, 0.8);
                sleep(50);
                
                //2. align with lander
                disableEncoders();
                angleTwo = performCorrection(45);
                sleep(50);

                //3. strafe to displace gold
                initializeEncoders();
                encoderStrafe(0.8, 1500, 45);
                
                //4. turn so it touches the crater
                disableEncoders();
                turn(0.5, 2000);
                mechanumDrive(0.5,0.5,0.5,0.5,1000);
            }
            break;
        }
    }
}
