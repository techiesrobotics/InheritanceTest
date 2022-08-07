package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="ChrisPIDPositionTest",group="Linear Opmode Test")
public class XTestPIDChris extends LinearOpMode {
    DcMotorEx motor;
    DcMotorEx motor2;

    double currentVelocity;
    double maxVelocity = 0.0;
    double currentPos;
    double repetitions = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotorEx.class, "PID");
        motor2 = hardwareMap.get(DcMotorEx.class, "PID2");

        setUpMotor(motor);
        motor.setDirection(DcMotorEx.Direction.FORWARD);
        setUpMotor(motor2);
        motor2.setDirection(DcMotorEx.Direction.FORWARD);
        waitForStart();
        // Pre-run
        while (opModeIsActive()) {
            moveTestMotor();
            

        }
    }

    private void setUpMotor(DcMotorEx aMotor) {

        aMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        aMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        aMotor.setVelocityPIDFCoefficients(1.20,.220, 0,10.996); //Change these
        aMotor.setPositionPIDFCoefficients(5.0);
        aMotor.setTargetPositionTolerance(50); //Maybe change this


    }


    private void moveTestMotor() {
        // if(gamepad1.) motor.setTargetPosition(0); // Change these
        motor2.setTargetPosition(-550);
        motor.setTargetPosition(550);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if(motor.isBusy() && repetitions < 800) {
            motor.setPower(0.6);
            motor2.setPower(0.6);

        }
        else{
            motor.setPower(0);
            motor2.setPower(0);
            repetitions = 0;
        }
        currentVelocity = motor.getVelocity();
        currentPos = motor.getCurrentPosition();
        if (currentVelocity > maxVelocity)
            maxVelocity = currentVelocity;

        telemetry.addData("current velocity", currentVelocity);
        telemetry.addData("current position", currentPos);
        telemetry.addData("position delta", currentPos-motor.getTargetPosition());
        telemetry.addData("power", motor.getPower());
        telemetry.addData("repetitions", repetitions);
        telemetry.update();
        repetitions++;
    }

}