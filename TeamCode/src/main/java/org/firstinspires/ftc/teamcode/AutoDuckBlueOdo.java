package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="AutoDuckBlueOdo", group="Pushbot")
//@Disabled
public class AutoDuckBlueOdo extends AutoDuckOdo {


    @Override
    protected void spinCarousel() {
        robot.duckMech.setPosition(1);
        sleep(3000);
        robot.duckMech.setPosition(.5);
    }

    @Override
    protected void goToCarousel() {
        Pose2d endPoseAllianceHub = new Pose2d(-27,-50, Math.toRadians(180));
        Trajectory goToCarouselDuckBlue = odoDriveTrain.trajectoryBuilder(endPoseAllianceHub)
                .lineToLinearHeading(new Pose2d(-55, -130, Math.toRadians(185)))
                .build();

        odoDriveTrain.followTrajectory(goToCarouselDuckBlue);

    }

    @Override
    protected void park() {
        Trajectory parkDuckBlue = odoDriveTrain.trajectoryBuilder(new Pose2d(-55,-130,Math.toRadians(185)))
                .lineToLinearHeading(new Pose2d(-68, -30, Math.toRadians(75)))
                .build();
        Trajectory parkDuckBlue2 = odoDriveTrain.trajectoryBuilder(new Pose2d(-50,-30,Math.toRadians(75)))
                .forward(40)
                .build();
        odoDriveTrain.followTrajectory(parkDuckBlue);
        odoDriveTrain.followTrajectory(parkDuckBlue2);


    }


    @Override
    protected void goToAllianceHubFromStart(){
        Pose2d startPose = new Pose2d(-48,-74, Math.toRadians(180));
        odoDriveTrain.setPoseEstimate(startPose);
        Trajectory goToAllianceHubFromStartDuckBlue = odoDriveTrain.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-27, -50, Math.toRadians(180)))
                .build();
        odoDriveTrain.followTrajectory(goToAllianceHubFromStartDuckBlue);
    }


}

