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

@Autonomous(name = "DepotTensorFlowTest", group = "Auto")
@Disabled
public class DepotTensorFlowTest extends AutoCommon {
    
    @Override
    public void runOpMode() throws InterruptedException { 
        
        //initialize motors and sensors
        initializeRobot();
        
        initializeIMU();
        
        initialiseVuForiaAndTensorFlow();
        
        
        
        telemetry.addData("","ready");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
        //   double turn = 0.25;
            
        //     angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //     float angleOne = angles.firstAngle;
        //     telemetry.addData("Angle", angles.firstAngle);
        //     telemetry.update();
            
            //strafeDiagonal(0.5, 500);
            
            sleep(3000);
            
            GoldPosition goldPosition = getGoldPosition();
            if (goldPosition == GoldPosition.LEFT) {
                telemetry.addData("Gold Mineral Position", "LEFT");
                sleep(5000);
            } else if (goldPosition == GoldPosition.CENTER) {
                telemetry.addData("Gold Mineral Position", "CENTER");
                sleep(5000);
            } else if (goldPosition == GoldPosition.RIGHT) {
                telemetry.addData("Gold Mineral Position", "RIGHT");
                sleep(5000);
            }else{
                telemetry.addData("Gold Mineral Position", "NOT DETECTED");
                sleep(5000);
            }
            sleep(10000);
            break;
        }
    }
}
