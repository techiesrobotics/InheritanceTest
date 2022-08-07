package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SlideMovementPIDController  {
    Telemetry telemetry = null;
    private ElapsedTime runtime = new ElapsedTime();
    double currentVelocity;
    double maxVelocity = 0.0;
    double currentPos;
    double repetitions = 0;

    public SlideMovementPIDController (Telemetry aTelemetry){
        Telemetry telemetry =aTelemetry;
    }

    public void slideMovementPID (TechiesSlideHardware slides, int targetPosition) {
        slides.rightriser.setTargetPosition(targetPosition);
        slides.leftriser.setTargetPosition(-targetPosition);
        slides.rightriser.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slides.leftriser.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if(slides.rightriser.isBusy() && repetitions < 800) {
            slides.rightriser.setPower(0.5);
            slides.leftriser.setPower(0.5);

        }
        else{
            slides.rightriser.setPower(0);
            slides.leftriser.setPower(0);
            repetitions = 0;
        }
        currentVelocity = slides.rightriser.getVelocity();
        currentPos = slides.leftriser.getCurrentPosition();
        if (currentVelocity > maxVelocity)
            maxVelocity = currentVelocity;

        telemetry.addData("current velocity", currentVelocity);
        telemetry.addData("current position", currentPos);
        telemetry.addData("position delta", currentPos- slides.rightriser.getTargetPosition());
        telemetry.addData("power", slides.rightriser.getPower());
        telemetry.addData("repetitions", repetitions);
        telemetry.update();
        repetitions++;
    }
}
