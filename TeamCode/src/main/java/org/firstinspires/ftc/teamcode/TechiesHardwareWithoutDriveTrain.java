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

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class TechiesHardwareWithoutDriveTrain
{
    /* Public OpMode members. */

    public DcMotor  intake      = null;

    public CRServo  leftBucket  = null;
    public CRServo  rightBucket = null;


    public Servo   duckMech   = null;

    public Servo   horizontalSlide   = null;


    public TechiesSlideHardware slides= null;

    /* local OpMode members. */
    HardwareMap hwMap     =  null;
    //private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public TechiesHardwareWithoutDriveTrain(HardwareMap aHWMap){
        hwMap = aHWMap;

        slides = new TechiesSlideHardware(hwMap);
        // Define and Initialize Motors


        intake    = hwMap.get(DcMotor.class, "intake");
/*
        leftriser    = hwMap.get(DcMotorEx.class, "leftriser");
         rightriser    = hwMap.get(DcMotorEx.class, "rightriser");
*/

        leftBucket    = hwMap.get(CRServo.class, "leftBucket");
        rightBucket   = hwMap.get(CRServo.class, "rightBucket");

        duckMech   = hwMap.get(Servo.class, "DuckMech");

        horizontalSlide = hwMap.get(Servo.class, "horizontalSlide");

        intake.setDirection(DcMotor.Direction.FORWARD);

        //duckMech.setDirection(CRServo.Direction.FORWARD);

        leftBucket.setDirection(CRServo.Direction.FORWARD);
        rightBucket.setDirection(CRServo.Direction.FORWARD);


        intake.setPower(0.0);

       // duckMech.setPower(0.0);

        leftBucket.setPower(0.0);
        rightBucket.setPower(0.0);
        horizontalSlide.setPosition(0.25);

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setBucketPower(double power){
        leftBucket.setPower(power);
        rightBucket.setPower(power);
    }
 }

