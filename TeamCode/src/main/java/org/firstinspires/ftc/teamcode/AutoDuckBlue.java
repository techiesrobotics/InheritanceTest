package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


//@Autonomous(name="AutoDuckBlue", group="Pushbot")
//@Disabled
public class AutoDuckBlue extends AutoDuck {


    @Override
    protected void spinCarousel() {
        // TODO add code
    }

    @Override
    protected void goToCarousel() {
        // TODO add code
    }

    @Override
    protected void park() {
        // TODO add code
    }


    @Override
    protected void goToAllianceHubFromStart(){
        Trajectory goToAllianceHubFromStartDuckBlue = drive.trajectoryBuilder(new Pose2d())
                .forward(5)
                .build();
        drive.followTrajectory(goToAllianceHubFromStartDuckBlue);
    }


}

