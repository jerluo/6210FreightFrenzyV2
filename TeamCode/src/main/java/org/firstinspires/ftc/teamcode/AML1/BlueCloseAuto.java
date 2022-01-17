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

@Autonomous(name = "BlueClose", group = "testTest")
public class BlueCloseAuto extends LinearOpMode {

    enum State {
        TRAJECTORY_1,   // Go to carousel
        CAROUSEL,       // Spin carousel
        TRAJECTORY_2,   // Go to depot
        LIFT,           // Lift cargo
        OUTTAKE,        // Outtake cargo
        RETRACT,        // Retract lift
        TRAJECTORY_3,   // Go to warehouse
        IDLE            // Our bot will enter the IDLE state when done
    }

    State currentState = State.IDLE;
    Pose2d startPose = new Pose2d(-24, -70, Math.toRadians(270));

    @Override
    public void runOpMode() throws InterruptedException{

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Manipulators manip = new Manipulators(hardwareMap);
        VuforiaBM vuforia = new VuforiaBM(this);

        drive.setPoseEstimate(startPose);

        //First trajectory to carousel
        Trajectory trajectory1 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-59.75, 62.5, Math.toRadians(131)))
                .build();

        //Wait during carousel
        double waitTime1 = 5;
        //Wait during outtake
        double waitTime2 = 3;
        ElapsedTime waitTimer = new ElapsedTime();

        // Second trajectory to depot
        // Ensure that we call trajectory1.end() as the start for this one
        Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end())
                .lineToLinearHeading(new Pose2d(-11.3, 36, Math.toRadians(261.5)))
                .build();

        // Third trajectory into the warehouse
        TrajectorySequence trajectory3 = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(10.3, 67.2, Math.toRadians(350)))
                .lineToLinearHeading(new Pose2d(50, 67.2, Math.toRadians(348)))
                .build();

        // Start doing vision
        int pos = vuforia.blueCarouselPosition();

        waitForStart();

        if (isStopRequested()) return;

        currentState = State.TRAJECTORY_1;
        drive.followTrajectoryAsync(trajectory1);

        while (opModeIsActive() && !isStopRequested()) {


            switch (currentState) {

                case TRAJECTORY_1:
                    telemetry.addData("Trajectory 1", currentState);
                    telemetry.update();
                    if (!drive.isBusy()) {
                        currentState = State.CAROUSEL;
                        manip.redCarousel();
                        waitTimer.reset();
                    }
                    break;
                case CAROUSEL:
                    telemetry.addData("Carousel", currentState);
                    telemetry.update();
                    if (waitTimer.seconds() >= waitTime1) {
                        currentState = State.TRAJECTORY_2;
                        drive.followTrajectoryAsync(trajectory2);
                        manip.carouselStop();
                    }
                    break;
                case TRAJECTORY_2:
                    if (!drive.isBusy()) {
                        currentState = State.LIFT;
                        manip.automaticLift(pos);
                    }
                    break;
                case LIFT:
                    if (!manip.liftIsBusy()) {
                        currentState = State.OUTTAKE;
                        // manip.intake() or something like that
                    }
                    break;
                case OUTTAKE:
                    if (waitTimer.seconds() >= waitTime2) {
                        currentState = State.RETRACT;
                        manip.automaticLift(0);
                    }
                    break;
                case RETRACT:
                    if (!manip.liftIsBusy()) {
                        currentState = State.TRAJECTORY_3;
                        drive.followTrajectorySequenceAsync(trajectory3);
                    }
                    break;
                case TRAJECTORY_3:
                    if (!drive.isBusy()) {
                        currentState = State.IDLE;
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
