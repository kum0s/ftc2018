/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;

@Autonomous(name = "", group = "Auto")
@Disabled
public abstract class AutoCommon extends LinearOpMode {

    DcMotor leftFrontMotor;
    DcMotor leftBackMotor;
    DcMotor rightFrontMotor;
    DcMotor rightBackMotor;

    DcMotor slide;
    DcMotor intake;
    DcMotor actuator;

    BNO055IMU imu;
    Orientation angles;

    enum GoldPosition{
        LEFT,
        RIGHT,
        CENTER,
        NONE
    };

    int motorPosition;
    double turn = 0.25;
    void initializeRobot () {

        double fwdBackPower, strafePower, turnPower, maxPower;
        double leftFrontPower, rightFrontPower;
        double leftBackPower, rightBackPower;

        leftFrontMotor= hardwareMap.dcMotor.get("driveFL");
        leftBackMotor= hardwareMap.dcMotor.get("driveBL");
        rightFrontMotor= hardwareMap.dcMotor.get("driveFR");
        rightBackMotor= hardwareMap.dcMotor.get("driveBR");
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotor.Direction.REVERSE);

        actuator = hardwareMap.dcMotor.get("actuator");
        intake = hardwareMap.dcMotor.get("intake");

    }

    void mechanumDrive(double leftFront, double rightFront, double leftBack, double rightBack, long time) {
        leftFrontMotor.setPower(leftFront);
        rightFrontMotor.setPower(rightFront);
        leftBackMotor.setPower(leftBack);
        rightBackMotor.setPower(rightBack);
        sleep(time);
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
    }

    void encoderStrafe(double strafe, int motorPos) {
        leftFrontMotor.setTargetPosition(leftFrontMotor.getCurrentPosition() + motorPos);
        rightFrontMotor.setTargetPosition(rightFrontMotor.getCurrentPosition() - motorPos);
        leftBackMotor.setTargetPosition(leftBackMotor.getCurrentPosition() - motorPos);
        rightBackMotor.setTargetPosition(rightBackMotor.getCurrentPosition() + motorPos);

        leftFrontMotor.setPower(strafe);
        rightFrontMotor.setPower(-strafe);
        leftBackMotor.setPower(-strafe);
        rightBackMotor.setPower(strafe);
        //sleep(time);

        while(leftFrontMotor.isBusy() && rightFrontMotor.isBusy() && leftBackMotor.isBusy() && rightBackMotor.isBusy() && opModeIsActive()) {
            //Loop body can be empty
            idle();
        }
    }
    
    void encoderStrafeDiagnal(double strafe, int time) {

    }
    
    void strafeDiagonal(double strafe, long time) {
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(-strafe);
        leftBackMotor.setPower(-strafe);
        rightBackMotor.setPower(0);

        sleep(time);
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
    }


    void strafe(double strafe, long time) {
        leftFrontMotor.setPower(strafe);
        rightFrontMotor.setPower(-strafe);
        leftBackMotor.setPower(-strafe);
        rightBackMotor.setPower(strafe);
        sleep(time);

        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
    }

    void turn(double turn, long time) {
        leftFrontMotor.setPower(turn);
        rightFrontMotor.setPower(-turn);
        leftBackMotor.setPower(turn);
        rightBackMotor.setPower(-turn);
        sleep(time);
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
    }

    void actuatorLift(int liftTime, boolean direction) {
        if (direction == true) {
            actuator.setPower(1);
            sleep(liftTime);
        } else if (direction == false) {
            actuator.setPower(-1);
            sleep(liftTime);
        }
        actuator.setPower(0);
    }

    void initializeIMU () {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        // angleOne = angles.firstAngle;
        telemetry.update();
    }

    void initializeEncoders(){
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    void moveUsingEncoder(int motorPos, double motorPower){
        leftFrontMotor.setTargetPosition(leftFrontMotor.getCurrentPosition() + motorPos);
        rightFrontMotor.setTargetPosition(rightFrontMotor.getCurrentPosition() + motorPos);
        leftBackMotor.setTargetPosition(leftBackMotor.getCurrentPosition() + motorPos);
        rightBackMotor.setTargetPosition(rightBackMotor.getCurrentPosition() + motorPos);

        leftFrontMotor.setPower(motorPower);
        rightFrontMotor.setPower(motorPower);
        leftBackMotor.setPower(motorPower);
        rightBackMotor.setPower(motorPower);
        while(leftFrontMotor.isBusy() && rightFrontMotor.isBusy() && leftBackMotor.isBusy() && rightBackMotor.isBusy() && opModeIsActive()) {
            //Loop body can be empty
            idle();
        }
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
    }

    void turnIntake (double intakeSpeed, int intakeTime) {
        intake.setPower(intakeSpeed);
        sleep(intakeTime);
        intake.setPower(0);
    }

    void encoderStrafe(double strafe, int motorPos, float angleOne) {

        leftFrontMotor.setTargetPosition(leftFrontMotor.getCurrentPosition() + motorPos);
        rightFrontMotor.setTargetPosition(rightFrontMotor.getCurrentPosition() - motorPos);
        leftBackMotor.setTargetPosition(leftBackMotor.getCurrentPosition() - motorPos);
        rightBackMotor.setTargetPosition(rightBackMotor.getCurrentPosition() + motorPos);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        float angleTwo = angles.firstAngle;
        double correction = angleTwo - angleOne;

        leftFrontMotor.setPower(strafe);
        rightFrontMotor.setPower(-strafe);
        leftBackMotor.setPower(-strafe);
        rightBackMotor.setPower(strafe);


        while(leftFrontMotor.isBusy() && rightFrontMotor.isBusy() && leftBackMotor.isBusy() && rightBackMotor.isBusy() && opModeIsActive()) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            angleTwo = angles.firstAngle;
            correction = angleTwo - angleOne;
            correction *= 0.1;
            telemetry.addData("Correction", correction);

            leftFrontMotor.setPower(strafe + correction);
            rightFrontMotor.setPower(-strafe);
            leftBackMotor.setPower(-strafe + correction);
            rightBackMotor.setPower(strafe);

            telemetry.addData("Angle", angleTwo);
            telemetry.update();
        }
    }

    void moveUsingEncoder(int motorPos, double motorPower, float angleOne){
        leftFrontMotor.setTargetPosition(leftFrontMotor.getCurrentPosition() + motorPos);
        rightFrontMotor.setTargetPosition(rightFrontMotor.getCurrentPosition() + motorPos);
        leftBackMotor.setTargetPosition(leftBackMotor.getCurrentPosition() + motorPos);
        rightBackMotor.setTargetPosition(rightBackMotor.getCurrentPosition() + motorPos);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        float angleTwo = angles.firstAngle;
        double correction = angleTwo - angleOne;

        leftFrontMotor.setPower(motorPower);
        rightFrontMotor.setPower(motorPower);
        leftBackMotor.setPower(motorPower);
        rightBackMotor.setPower(motorPower);
        while(leftFrontMotor.isBusy() && rightFrontMotor.isBusy() && leftBackMotor.isBusy() && rightBackMotor.isBusy() && opModeIsActive()) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            angleTwo = angles.firstAngle;
            correction = angleTwo - angleOne;
            correction *= 0.1;
            telemetry.addData("Correction", correction);

            leftFrontMotor.setPower(motorPower - correction);
            rightFrontMotor.setPower(motorPower + correction);
            leftBackMotor.setPower(motorPower - correction);
            rightBackMotor.setPower(motorPower + correction);

            telemetry.addData("Angle", angleTwo);
            telemetry.update();
        }
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
    }

    void disableEncoders() {
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
    }

    float performCorrection(float angleOne) {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        float angleTwo = angles.firstAngle;
        telemetry.addData("Angle", angleTwo);
        telemetry.update();

        if (angleOne > angleTwo) {
            leftFrontMotor.setPower(-turn);
            rightFrontMotor.setPower(turn);
            leftBackMotor.setPower(-turn);
            rightBackMotor.setPower(turn);
            while (angleOne > angleTwo && opModeIsActive()){
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                angleTwo = angles.firstAngle;
                telemetry.addData("Angle", angleTwo);
                telemetry.update();
            }
        } else if (angleOne < angleTwo ) {
            leftFrontMotor.setPower(turn);
            rightFrontMotor.setPower(-turn);
            leftBackMotor.setPower(turn);
            rightBackMotor.setPower(-turn);
            while (angleOne < angleTwo && opModeIsActive()){
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                angleTwo = angles.firstAngle;
                telemetry.addData("Angle", angleTwo);
                telemetry.update();
            }
        }

        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);

        return angleTwo;
    }

    GoldPosition getMineralPosition() {
        return GoldPosition.LEFT;
    }

    //----------------------------------------------------------------------------------------------
    //
    //////////////////////////////VuForia and TensorFlow code///////////////////////////////////////
    //
    //----------------------------------------------------------------------------------------------
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "Afcs+zn/////AAAAGR09crC9yUhjlzB3l5aWqoqFpQF6d25RQchdU+kLgbg3ogYJxJ9Meutxrqr/w8m+WnE3PFjXoKhl0/lB4fhAOH6KYRKEj1L8e7BLDsgLVfhAXBibh3140QCKVgK+XF9tZeJWtVPj5svRdXimk5zsE5Dacs4YLOrH+coqabK/YUCpH2AUxu5lWwiS5MPWjTYAsCkKRt+a1wOQxMA3M/J23O970JhNxcqQup96nAk9IJlqybM2dHbW4/TZf1+TvmMZnnR3WZu9XdXz3c7zPEGVyELEz5VbXMffkLXAqC4p7y3+8FCvEnZtWvV7lAr/l7gW+qPNYebnD1sSomPzUoRpHMJlOkxT7uuLgPRj8dDgNCC7";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    public GoldPosition getGoldPosition() {
        GoldPosition goldPosition = GoldPosition.NONE;
        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }
            boolean isDetected = false;
            ElapsedTime timer = new ElapsedTime();
            while (opModeIsActive() && timer.milliseconds() < 3000) {
                if (tfod != null && !isDetected) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() == 2) {
                            int goldMineralX = -1;
                            int silverMineral1X = -1;
                            int silverMineral2X = -1;
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) recognition.getLeft();
                                } else if (silverMineral1X == -1 && recognition.getLabel().equals(LABEL_SILVER_MINERAL)) {
                                    silverMineral1X = (int) recognition.getLeft();
                                } else if( recognition.getLabel().equals(LABEL_SILVER_MINERAL)){
                                    silverMineral2X = (int) recognition.getLeft();
                                }
                            }

                            if(goldMineralX == -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                                telemetry.addData("Gold Mineral Position", "Right");
                                goldPosition = GoldPosition.RIGHT;
                                isDetected = true;
                            }

                            if (silverMineral1X != -1 && goldMineralX != -1 && silverMineral2X == -1) {
                                if (goldMineralX < silverMineral1X) {
                                    telemetry.addData("Gold Mineral Position", "Center");
                                    goldPosition = GoldPosition.CENTER;
                                    isDetected = true;
                                } else if (goldMineralX > silverMineral1X) {
                                    telemetry.addData("Gold Mineral Position", "Left");
                                    goldPosition = GoldPosition.LEFT;
                                    isDetected = true;
                                }
                            }
                        }
                        telemetry.update();
                    }
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
        
        return goldPosition;
    }

    public void initialiseVuForiaAndTensorFlow(){
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
    }
    
    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.40;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
}
