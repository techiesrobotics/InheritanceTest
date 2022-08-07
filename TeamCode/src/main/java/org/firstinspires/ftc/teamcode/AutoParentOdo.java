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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;


//@Disabled
public abstract class AutoParentOdo extends LinearOpMode {

    /* Declare OpMode members. */
    SampleMecanumDrive odoDriveTrain;
    TechiesHardwareWithoutDriveTrain robot;
    ElapsedTime runtime = new ElapsedTime();
    double currentVelocity;
    double maxVelocity = 0.0;
    double currentPos;
    double repetitions = 0;
    SlideMovementPIDController pidController;
    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    public static final int LEVEL_ZONE_1 = 1;
    public static final int LEVEL_ZONE_2 = 2;
    public static final int LEVEL_ZONE_3 = 3;
    public static final int LEVEL_ZONE_DEFAULT = LEVEL_ZONE_1;

    static final String VUFORIA_KEY =
            " AbfemMf/////AAABmVQ5LwVH3Umfv+Oiv7oNSvcdOBa+ogwEc69mgH/qFNgbn1NXBJsX4J5R6N4SYojjxFB6eHjdTHaT3i9ZymELzgaFrPziL5B/TX2/dkxnIK5dcOjLHOZu2K3jVPYciJnwj20ZmRXSN46Y4uMdzWSJ3X1wgKovNQzZvx+7dljIonRJfLjSF5aSuoTDqEkdcsGTJ92J7jgc5jN53Vml3rAI+qPUTT8qpI8T1enV6NYublcUMpofpovmHsH9kvI+U1h9Rc8cXwbGPlr3PVoKQOwuZA0Y98Jywey6URYTswzpbMw8cWu4PMisB2ujpf0VEjiV6jjofr3OVRj6r5lEJZsnElY2mOhXdgVqJndvXYvCSpyI";
    static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    static final String LABEL_FIRST_ELEMENT = "Duck";
    VuforiaLocalizer vuforia;
    TFObjectDetector tfod;
    String Level;

    OpenCvCamera webcam;
    TechiesPipeline pipeline;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables and camera.
         * The init() method of the hardware class does all the work here
         */

        robot = new TechiesHardwareWithoutDriveTrain(hardwareMap);
        odoDriveTrain = new SampleMecanumDrive(hardwareMap);
        pidController = new SlideMovementPIDController(telemetry);
        pipeline = new TechiesPipeline();
       // initVuforia();
        //initTfod();
        //activateCamera();
       // int targetZone = determineZoneLevel();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.

        webcam.openCameraDeviceAsync(new OpenCvWebcam.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        telemetry.addData("before detect target zone", "targetZone");
        telemetry.update();
        int targetZone = pipeline.detemineFreightLevel();

        telemetry.addData("after determine target Zone Level", targetZone);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("before wait for start", "before");
        telemetry.update();
        waitForStart();

        doMissions(targetZone);

        //shutDownCamera();

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }


    protected void doMissions(int targetZone) {

        goToAllianceHubFromStart();
        dropPreloadFreight(targetZone);
        doAdditionalMissions(targetZone);
        park();


    }

    protected abstract void doAdditionalMissions(int targetZone) ;
    protected abstract void park();
    protected abstract void goToAllianceHubFromStart() ;
    protected void dropPreloadFreight(int targetZone) {
        telemetry.addData("dropPreloadFreight", "dropPreloadFreight");

        if (LEVEL_ZONE_1 == targetZone) {
            robot.leftBucket.setPower(-.2);
            robot.rightBucket.setPower(-.2);
            sleep(350);
            robot.leftBucket.setPower(0);
            robot.rightBucket.setPower(0);
            SlideMovementPID(100);
            robot.horizontalSlide.setPosition(.8);
            sleep(1000);
            robot.leftBucket.setPower(-.2);
            robot.rightBucket.setPower(-.2);
            sleep(800);
            robot.leftBucket.setPower(.2);
            robot.rightBucket.setPower(.2);
            sleep(600);
            robot.horizontalSlide.setPosition(.3);
            robot.slides.rightriser.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            robot.slides.leftriser.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            robot.slides.rightriser.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.slides.leftriser.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.slides.leftriser.setPower(0);
            robot.slides.rightriser.setPower(0);

        } else if (LEVEL_ZONE_2 == targetZone) {
            robot.leftBucket.setPower(-.2);
            robot.rightBucket.setPower(-.2);
            sleep(350);
            robot.leftBucket.setPower(0);
            robot.rightBucket.setPower(0);
            SlideMovementPID(300);
            robot.horizontalSlide.setPosition(.8);
            sleep(1000);
            robot.leftBucket.setPower(-.2);
            robot.rightBucket.setPower(-.2);
            sleep(1000);
            robot.leftBucket.setPower(.2);
            robot.rightBucket.setPower(.2);
            sleep(150);
            robot.horizontalSlide.setPosition(.3);
            sleep(100);
            robot.slides.rightriser.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            robot.slides.leftriser.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            robot.slides.rightriser.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.slides.leftriser.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.slides.leftriser.setPower(0);
            robot.slides.rightriser.setPower(0);


        } else if (LEVEL_ZONE_3 == targetZone) {
            robot.leftBucket.setPower(-.2);
            robot.rightBucket.setPower(-.2);
            sleep(350);
            robot.leftBucket.setPower(0);
            robot.rightBucket.setPower(0);
            SlideMovementPID(470);
            robot.horizontalSlide.setPosition(.8);
            sleep(1000);
            robot.leftBucket.setPower(-.2);
            robot.rightBucket.setPower(-.2);
            sleep(700);
            robot.leftBucket.setPower(.1);
            robot.rightBucket.setPower(.1);
            sleep(175);
            robot.horizontalSlide.setPosition(.3);
            sleep(100);
            robot.slides.rightriser.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            robot.slides.leftriser.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            robot.slides.rightriser.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.slides.leftriser.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.slides.leftriser.setPower(0);
            robot.slides.rightriser.setPower(0);


        }


        /*
        robot.setBucketPower(-.2);
        sleep(350);
        robot.setBucketPower(0);


        if (LEVEL_ZONE_1 == targetZone) {
          // TODO KL TEST
              SlideMovementPID(200);  // different
            //pidController.slideMovementPID(robot.slides, 200);
        } else if (LEVEL_ZONE_2 == targetZone) {
            SlideMovementPID(300);
            // TODO KL TEST
         //   pidController.slideMovementPID(robot.slides, 300);
        } else if (LEVEL_ZONE_3 == targetZone) {
             SlideMovementPID(450);
            // TODO KL TEST
            //pidController.slideMovementPID(robot.slides, 450);
        }
        robot.horizontalSlide.setPosition(.8);
        sleep(1000);
        robot.setBucketPower(-.2); // TODO is this the same across 3? .2 or -.2?
        sleep(1000);
        robot.setBucketPower(.2);

        if (LEVEL_ZONE_1 == targetZone) {
            sleep(600);
        } else if (LEVEL_ZONE_2 == targetZone) {
            sleep(150);
        } else if (LEVEL_ZONE_3 == targetZone) {
            sleep(150);
        }
        robot.horizontalSlide.setPosition(.3);
        sleep(100);
        robot.slides.setRiserPower(0);

         */

    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.7f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT);
    }
    protected void activateCamera() {
        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            telemetry.addData(">", "activate camera");
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.1, 16.0 / 5.0);
        }
    }

    protected int determineZoneLevel() {
        int targetZone = LEVEL_ZONE_DEFAULT;
        if (opModeIsActive()) {
            if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        // step through the list of recognitions and display boundary info.
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)"), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)"), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)"), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());
                            Double angle = Double.valueOf( recognition.estimateAngleToObject(AngleUnit.DEGREES));
                            telemetry.addData(String.format("  Angle: "), angle);

                            if (angle.doubleValue() <= -7){
                                Level = "One";
                                targetZone = LEVEL_ZONE_1;

                            }
                            else if (angle.doubleValue() >= 7){
                                Level = "Three";
                                targetZone = LEVEL_ZONE_3;
                            }
                            else {
                                Level = "Two";
                                targetZone = LEVEL_ZONE_2;

                            }
                            telemetry.addData(String.format("  Level: "), Level);

                        }
                        telemetry.update();
                    }
                }
            }


        return targetZone;
    }
    protected void shutDownCamera() {
        if (tfod != null) {
            tfod.shutdown();
        }
    }
    private void SlideMovementPID (int targetPosition) {
        telemetry.addData("SlideMovementPID", "start SlideMovementPID");
        robot.slides.rightriser.setTargetPosition(targetPosition);
        robot.slides.leftriser.setTargetPosition(-targetPosition);
        robot.slides.rightriser.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.slides.leftriser.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if(robot.slides.rightriser.isBusy() && repetitions < 800) {

            robot.slides.rightriser.setPower(0.5);
            robot.slides.leftriser.setPower(0.5);
            telemetry.addData("inside if", "inside if");
        }
        else{
            telemetry.addData("SlideMovementPID", "inside  else");
            robot.slides.rightriser.setPower(0);
            robot.slides.leftriser.setPower(0);
            repetitions = 0;
        }
        currentVelocity = robot.slides.rightriser.getVelocity();
        currentPos = robot.slides.leftriser.getCurrentPosition();
        if (currentVelocity > maxVelocity)
            maxVelocity = currentVelocity;

        telemetry.addData("current velocity", currentVelocity);
        telemetry.addData("current position", currentPos);
        telemetry.addData("position delta", currentPos- robot.slides.rightriser.getTargetPosition());
        telemetry.addData("power", robot.slides.rightriser.getPower());
        telemetry.addData("repetitions", repetitions);
        telemetry.update();
        repetitions++;
    }

}
