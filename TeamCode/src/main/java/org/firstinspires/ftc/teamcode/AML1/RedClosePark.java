package org.firstinspires.ftc.teamcode.AML1;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Manipulators;
import org.firstinspires.ftc.teamcode.VuforiaBM;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "RedClosePark", group = "testTest")
public class RedClosePark extends LinearOpMode {

    enum State {
        TRAJECTORY_1,   // Go to carousel
        CAROUSEL,       // Spin carousel
        TRAJECTORY_2,   // Go to depot
        IDLE            // Our bot will enter the IDLE state when done
    }

    State currentState = State.IDLE;
    Pose2d startPose = new Pose2d(-24, -70, Math.toRadians(90));

    @Override
    public void runOpMode() throws InterruptedException{

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Manipulators manip = new Manipulators(hardwareMap);
        VuforiaBM vuforia = new VuforiaBM(this);

        drive.setPoseEstimate(startPose);

        //First trajectory to carousel
        Trajectory trajectory1 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-61.5, -63.5, Math.toRadians(245)))
                .build();

        //Wait during carousel
        double waitTime1 = 5;
        double waitTime2 = 2;
        ElapsedTime waitTimer = new ElapsedTime();

        // Third trajectory into the warehouse
        TrajectorySequence trajectory2 = drive.trajectorySequenceBuilder(trajectory1.end())
                .lineToLinearHeading(new Pose2d(-24, -71.2, Math.toRadians(173)))
                .lineToLinearHeading(new Pose2d(10.3, -73.2, Math.toRadians(173)))
                .lineToLinearHeading(new Pose2d(50, -73.2, Math.toRadians(173)))
                .build();

        // Start doing vision
        //int pos = vuforia.capPositionReturn();

        waitForStart();

        if (isStopRequested()) return;

        currentState = State.TRAJECTORY_1;
        drive.followTrajectoryAsync(trajectory1);

        while (opModeIsActive() && !isStopRequested()) {


            switch (currentState) {

                case TRAJECTORY_1:
                    // Check if the drive class isn't busy
                    // `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
                    // We move on to the next state
                    // Make sure we use the async follow function
                    telemetry.addData("Trajectory 1", currentState);
                    telemetry.update();
                    if (!drive.isBusy()) {
                        currentState = State.CAROUSEL;
                        manip.redCarousel();
                        waitTimer.reset();
                    }
                    break;
                case CAROUSEL:
                    // Use wait time to spin duck off carousel
                    telemetry.addData("Carousel", currentState);
                    telemetry.update();
                    if (waitTimer.seconds() >= waitTime1) {
                        currentState = State.TRAJECTORY_2;
                        drive.followTrajectorySequenceAsync(trajectory2);
                        manip.carouselStop();
                    }
                    break;
                case TRAJECTORY_2:
                    // Check if the drive class is busy turning
                    // If not, move onto the next state, DROP, once finished
                    if (!drive.isBusy()) {
                        currentState = State.IDLE;

                        break;
                    }
                    break;
                case IDLE:
                    telemetry.addData("IDLE", "here");
                    telemetry.update();
                    // Do nothing in IDLE
                    // currentState does not change once in IDLE
                    // This concludes the autonomous program
                    break;
            }
            drive.update();

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();


            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }


    }
}
