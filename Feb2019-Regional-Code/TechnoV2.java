package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "TechnoV2", group = "Teleop")

public class TechnoV2 extends LinearOpMode{
    
    DcMotor leftFrontMotor;
    DcMotor leftBackMotor;
    DcMotor rightFrontMotor;
    DcMotor rightBackMotor;
    
    DcMotor intake;
    DcMotor actuator;
    DigitalChannel touch;
    
    @Override
    public void runOpMode()throws InterruptedException {
        
        double fwdBackPower, strafePower, turnPower, maxPower;
        double leftFrontPower, rightFrontPower;
        double leftBackPower, rightBackPower;
        
        leftFrontMotor= hardwareMap.dcMotor.get("driveFL");
        leftBackMotor= hardwareMap.dcMotor.get("driveBL");
        rightFrontMotor= hardwareMap.dcMotor.get("driveFR");
        rightBackMotor= hardwareMap.dcMotor.get("driveBR");
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotor.Direction.REVERSE);
        
        intake = hardwareMap.dcMotor.get("intake");
        actuator = hardwareMap.dcMotor.get("actuator");
        touch = hardwareMap.get(DigitalChannel.class, "touch");
        
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
            
            telemetry.addData("powers", "|%.3f|%.3f|%.3f|%.3f|",
                        leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
            telemetry.update();
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
                intake.setPower(gamepad2.left_trigger);
            } else if (gamepad2.right_trigger > 0) {
                intake.setPower(-gamepad2.right_trigger);
            } else {
                intake.setPower(0);
            }
            
            if (gamepad2.a) {
                actuator.setPower(1);
            } else if (gamepad2.b) {
                actuator.setPower(-1);
            } else {
                actuator.setPower(0);
            }
            
            telemetry.update();
            
        }
    }

}
