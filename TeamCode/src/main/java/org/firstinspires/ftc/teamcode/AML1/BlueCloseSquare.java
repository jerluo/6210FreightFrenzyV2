package org.firstinspires.ftc.teamcode.AML1;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Manipulators;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "BlueCloseSquare", group = "testTest")
public class BlueCloseSquare extends LinearOpMode {

    enum State {
        TRAJECTORY_1,   // Go to carousel
        CAROUSEL,       // Spin carousel
        TRAJECTORY_2,   // Go to dump
        LIFT,
        OUTTAKE,        // Outtake cargo
        TRAJECTORY_3,   // Go to warehouse
        IDLE            // Our bot will enter the IDLE state when done
    }

    State currentState = State.IDLE;
    Pose2d startPose = new Pose2d(-24, 70, Math.toRadians(270));

    @Override
    public void runOpMode() throws InterruptedException{

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Manipulators manip = new Manipulators(hardwareMap);
        //VuforiaBM vuforia = new VuforiaBM(this);

        drive.setPoseEstimate(startPose);

        //First trajectory to carousel
        Trajectory trajectory1 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-58, 62.5, Math.toRadians(131)))
                .build();

        //Wait during carousel
        double waitTime1 = 5;
        //Wait during outtake
        double waitTime2 = 1.5;
        //Wait during lift
        double waitTime3 = 0.5;
        ElapsedTime waitTimer = new ElapsedTime();

        // Second trajectory to depot
        // Ensure that we call trajectory1.end() as the start for this one
        Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end())
                .lineToLinearHeading(new Pose2d(-50, 45, Math.toRadians(180)))
                .build();


        // Third trajectory into the warehouse
        TrajectorySequence trajectory3 = drive.trajectorySequenceBuilder(trajectory1.end())
                .lineToLinearHeading(new Pose2d(-24, 74.2, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(10.3, 75.2, Math.toRadians(190)))
                .lineToLinearHeading(new Pose2d(50, 75.2, Math.toRadians(190)))
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
                        manip.blueCarousel();
                        waitTimer.reset();
                    }
                    break;
                case CAROUSEL:
                    // Use wait time to spin duck off carousel
                    telemetry.addData("Carousel", currentState);
                    telemetry.update();
                    if (waitTimer.seconds() >= waitTime1) {
                        currentState = State.TRAJECTORY_2;
                        drive.followTrajectoryAsync(trajectory2);
                        manip.carouselStop();
                    }
                    break;
                case TRAJECTORY_2:
                    // Check if the drive class is busy turning
                    // If not, move onto the next state, DROP, once finished
                    if (!drive.isBusy()) {
                        currentState = State.LIFT;
                        waitTimer.reset();

                        manip.manualLift(0.5);
                    }
                    break;
                case LIFT:
                    if (waitTimer.seconds() >= waitTime3) {
                        currentState = State.OUTTAKE;
                        waitTimer.reset();

                        manip.manualLift(0);
                        manip.intake(true);
                    }
                    break;
                case OUTTAKE:
                    // Make sure the lift has reached target position
                    // by checking if it's still busy
                    // When reached, outtake block
                    if (waitTimer.seconds() >= waitTime2) {
                        currentState = State.TRAJECTORY_3;
                        manip.intakeStop();
                        drive.followTrajectorySequenceAsync(trajectory3);
                    }
                    break;
                case TRAJECTORY_3:
                    // Check if the timer has exceeded the specified wait time
                    // If so, move on to the TURN_2 state
                    if (!drive.isBusy()) {
                        currentState = State.IDLE;
                    }
                    break;
                case IDLE:
                    telemetry.addLine("Finished");
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
