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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;


@Autonomous(name="Techies Autonomous", group="Pushbot")
//@Disabled
public class TechiesAutonomous extends LinearOpMode {

    /* Declare OpMode members. */
    TechiesHardware robot = new TechiesHardware();
    private ElapsedTime runtime = new ElapsedTime();

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

    private static final String VUFORIA_KEY =
            " AbfemMf/////AAABmVQ5LwVH3Umfv+Oiv7oNSvcdOBa+ogwEc69mgH/qFNgbn1NXBJsX4J5R6N4SYojjxFB6eHjdTHaT3i9ZymELzgaFrPziL5B/TX2/dkxnIK5dcOjLHOZu2K3jVPYciJnwj20ZmRXSN46Y4uMdzWSJ3X1wgKovNQzZvx+7dljIonRJfLjSF5aSuoTDqEkdcsGTJ92J7jgc5jN53Vml3rAI+qPUTT8qpI8T1enV6NYublcUMpofpovmHsH9kvI+U1h9Rc8cXwbGPlr3PVoKQOwuZA0Y98Jywey6URYTswzpbMw8cWu4PMisB2ujpf0VEjiV6jjofr3OVRj6r5lEJZsnElY2mOhXdgVqJndvXYvCSpyI";
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Duck";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private String Level;



    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables and camera.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        initiateRobot();
        initVuforia();
        initTfod();
        activateCamera();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        int targetZone = determineZoneLevel();
        telemetry.addData("Zone Level", targetZone);

        Drive_forward(targetZone);
        shutDownCamera();


        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    private void Drive_forward(int targetZone) {
        if (LEVEL_ZONE_1 == targetZone) {
            encoderDrive(DRIVE_SPEED, -10, -10, -10, -10, 5.0);

        } else if (LEVEL_ZONE_2 == targetZone) {
            encoderDrive(DRIVE_SPEED, -51, -51, -51, -51, 5.0);


        } else if (LEVEL_ZONE_3 == targetZone) {
            encoderDrive(DRIVE_SPEED, 10, 10, 10, 10, 5.0);


        }

    }


    public void encoderDrive(double speed,
                             double leftInches, double rightInches, double leftBackInches, double rightBackInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftDrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightDrive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            newLeftBackTarget = robot.leftBack.getCurrentPosition() + (int) (leftBackInches * COUNTS_PER_INCH);
            newRightBackTarget = robot.rightBack.getCurrentPosition() + (int) (rightBackInches * COUNTS_PER_INCH);

            robot.leftDrive.setTargetPosition(newLeftTarget);
            robot.rightDrive.setTargetPosition(newRightTarget);
            robot.leftBack.setTargetPosition(newLeftBackTarget);
            robot.rightBack.setTargetPosition(newRightBackTarget);
            // Turn On RUN_TO_POSITION
            setEncoder(DcMotor.RunMode.RUN_TO_POSITION);
            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftDrive.setPower(Math.abs(speed));
            robot.rightDrive.setPower(Math.abs(speed));
            robot.leftBack.setPower(Math.abs(speed));
            robot.rightBack.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftDrive.isBusy() && robot.rightDrive.isBusy() &&
                            robot.leftBack.isBusy() && robot.rightBack.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget,
                        newRightTarget, newLeftBackTarget, newRightBackTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.leftDrive.getCurrentPosition(),
                        robot.rightDrive.getCurrentPosition(),
                        robot.leftBack.getCurrentPosition(),
                        robot.rightBack.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);
            robot.leftBack.setPower(0);
            robot.rightBack.setPower(0);

            // Turn off RUN_TO_POSITION
            setEncoder(DcMotor.RunMode.RUN_USING_ENCODER);
            //  sleep(250);   // optional pause after each move
        }
    }

    private void setEncoder(DcMotor.RunMode stopAndResetEncoder) {
        robot.leftDrive.setMode(stopAndResetEncoder);
        robot.rightDrive.setMode(stopAndResetEncoder);
        robot.leftBack.setMode(stopAndResetEncoder);
        robot.rightBack.setMode(stopAndResetEncoder);
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
    protected void initiateRobot() {
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                robot.leftDrive.getCurrentPosition(),
                robot.rightDrive.getCurrentPosition());
        telemetry.update();
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
                                telemetry.addData(String.format("  Level: "), Level);
                                targetZone = LEVEL_ZONE_1;

                            }
                            else if (angle.doubleValue() >= 7){
                                Level = "Three";
                                telemetry.addData(String.format("  Level: "), Level);
                                targetZone = LEVEL_ZONE_3;

                            }
                            else {
                                Level = "Two";
                                telemetry.addData(String.format("  Level: "), Level);
                                targetZone = LEVEL_ZONE_2;

                            }

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

}
