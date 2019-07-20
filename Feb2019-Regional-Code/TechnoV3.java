package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

//@Disabled
@TeleOp(name = "TechnoV3", group = "Teleop")

public class TechnoV3 extends LinearOpMode{
    
    DcMotor leftFrontMotor;
    DcMotor leftBackMotor;
    DcMotor rightFrontMotor;
    DcMotor rightBackMotor;
    
    DcMotor intake;
    DcMotor actuator;
    DcMotor lift;
    
    DcMotor tilt;
    
    boolean liftPosition;
    boolean usingAutoLift;
    boolean tiltPosition;
    boolean usingAutoTilt;
    
    int[] positions = {-50, 0, 570};
    
    @Override
    public void runOpMode()throws InterruptedException {
        
        double fwdBackPower, strafePower, turnPower, maxPower;
        double leftFrontPower, rightFrontPower;
        double leftBackPower, rightBackPower;
        
        leftFrontMotor = hardwareMap.dcMotor.get("driveFL");
        leftBackMotor = hardwareMap.dcMotor.get("driveBL");
        rightFrontMotor = hardwareMap.dcMotor.get("driveFR");
        rightBackMotor = hardwareMap.dcMotor.get("driveBR");
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotor.Direction.REVERSE);
        
        intake = hardwareMap.dcMotor.get("intake");
        actuator = hardwareMap.dcMotor.get("actuator");
        lift = hardwareMap.dcMotor.get("lift");
        tilt = hardwareMap.dcMotor.get("tilt");
        
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        tilt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tilt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();
        
        while (opModeIsActive()){
            
            fwdBackPower= -gamepad1.left_stick_y;
            
            if (-0.25 < gamepad1.left_stick_x && gamepad1.left_stick_x < 0.25) {
                strafePower = 0;
            } else {
                strafePower= gamepad1.left_stick_x;
            }
            
            turnPower= gamepad1.right_stick_x;
            
            leftFrontPower= fwdBackPower + turnPower + strafePower;
            rightFrontPower= fwdBackPower - turnPower - strafePower;
            leftBackPower= fwdBackPower + turnPower - strafePower;
            rightBackPower= fwdBackPower - turnPower + strafePower;
            
            maxPower = Math.abs(leftFrontPower);
            if(Math.abs(rightFrontPower) > maxPower) {maxPower = Math.abs(rightFrontPower);} 
            if(Math.abs(leftBackPower) > maxPower) {maxPower = Math.abs(leftBackPower);} 
            if(Math.abs(rightBackPower) > maxPower) {maxPower = Math.abs(rightBackPower);} 
            
            if (maxPower > 1) {
                leftFrontPower = leftFrontPower/maxPower;
                rightFrontPower = rightFrontPower/maxPower;
                leftBackPower = leftBackPower/maxPower;
                rightBackPower = rightBackPower/maxPower;
            }
            
            if (gamepad1.right_bumper || gamepad1.left_bumper) {
                leftFrontPower = leftFrontPower * 0.3;
                rightFrontPower = rightFrontPower * 0.3;
                leftBackPower = leftBackPower * 0.3;
                rightBackPower = rightBackPower * 0.3;
            }
            leftFrontMotor.setPower(leftFrontPower);
            rightFrontMotor.setPower(rightFrontPower);
            leftBackMotor.setPower(leftBackPower);
            rightBackMotor.setPower(rightBackPower);
            
            if (gamepad2.left_trigger > 0) {
                
                if (gamepad2.left_bumper || gamepad2.right_bumper) {
                    intake.setPower(gamepad2.left_trigger * 0.25);
                } else{
                    intake.setPower(gamepad2.left_trigger);
                }
                
            } else if (gamepad2.right_trigger > 0) {
                
                if (gamepad2.left_bumper || gamepad2.right_bumper) {
                    intake.setPower(-gamepad2.right_trigger * 0.25);
                } else{
                   intake.setPower(-gamepad2.right_trigger); 
                }
                
            } else {
                intake.setPower(0);
            }
            
            if (gamepad2.a) {
                if (gamepad2.left_bumper || gamepad2.right_bumper){
                    actuator.setPower(.5);   
                } else{
                    actuator.setPower(1); 
                }

            } else if (gamepad2.b) {
                if (gamepad2.left_bumper || gamepad2.right_bumper){
                    actuator.setPower(-.5);
                } else {
                    actuator.setPower(-1);
                }
                
            } else {
                actuator.setPower(0);
            }
            
            // manual section for lift
            if (gamepad2.left_stick_y != 0 && gamepad2.right_stick_y != 0) {
                usingAutoLift = false;
                lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                
                if (gamepad2.left_bumper || gamepad2.right_bumper){
                    lift.setPower((gamepad2.left_stick_y * (-.3)));
                } else{
                    lift.setPower((gamepad2.left_stick_y * (-.85)));
                }
                
            } else {
                if(usingAutoLift == false){
                  lift.setPower(0.00);  
                }
                
            }
            
            //manual section for tilt
            if (gamepad2.dpad_down) {
                tilt.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                
                if (gamepad2.left_bumper || gamepad2.right_bumper){
                    tilt.setPower(-0.1);
                } else {
                    tilt.setPower(-0.3);
                }
                
                usingAutoTilt = false;
            } else if (gamepad2.dpad_up) {
                tilt.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                
                if (gamepad2.left_bumper || gamepad2.right_bumper){
                    tilt.setPower(0.2); 
                } else{
                    tilt.setPower(0.3);
                }
                
                usingAutoTilt = false;
            } else {
                if(usingAutoTilt == false){
                    tilt.setPower(0);
                }
            }
            
            //telemetry.update();
            
            if (gamepad2.y) {
                lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftPosition = true;
                usingAutoLift = true;
                
                
            }
            
            if (gamepad2.x) {
                lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftPosition = false;
                usingAutoLift = true;
                
            }
            if (usingAutoLift == true) {
                
                if (liftPosition == true && lift.getCurrentPosition() < 7850) {
                    lift.setTargetPosition(7850);
                    lift.setPower(1);

                } else if (lift.getCurrentPosition() > 7850) {
                    lift.setTargetPosition(7850);
                    lift.setPower(-1);
                } else if (liftPosition == false && lift.getCurrentPosition() > 0){
                    lift.setTargetPosition(0);
                    lift.setPower(-1);
                
                } else if (liftPosition == false && lift.getCurrentPosition() < 0){
                    lift.setTargetPosition(0);
                    lift.setPower(1);
                
                } else {
                    lift.setPower(0.00);  
            
                }   
            }
            
            
            if (gamepad2.dpad_right) {
                tilt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                tiltPosition = true;
                usingAutoTilt = true;
                
                
            }
            
            if (gamepad2.dpad_left) {
                tilt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                tiltPosition = false;
                usingAutoTilt = true;
                
            }
            
            if (usingAutoTilt == true) {
                
                if (tiltPosition == true && tilt.getCurrentPosition() < 570) {
                    tilt.setTargetPosition(570);
                    tilt.setPower(0.25);
                    
                // } else if (tilt.getCurrentPosition() > 570) {
                //     tilt.setTargetPosition(570);
                //     tilt.setPower(-0.25);
                } else if (tiltPosition == false && tilt.getCurrentPosition() > 0){
                    tilt.setTargetPosition(0);
                    tilt.setPower(-0.25);
                
                // } else if (tiltPosition == false && tilt.getCurrentPosition() < 0){
                //     tilt.setTargetPosition(0);
                //     tilt.setPower(0.25);
                
                } else {
                    tilt.setPower(0.00);  
            
                }   
            }
            
            
            if (gamepad2.left_stick_button && gamepad2.right_stick_button) {
                lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            } 
            
            if (gamepad2.right_stick_button){
                
                tilt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            
            telemetry.addData("", tilt.getCurrentPosition());
            telemetry.addData("", lift.getCurrentPosition());
            telemetry.update();
        }
    }

}
