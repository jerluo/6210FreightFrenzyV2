package org.firstinspires.ftc.teamcode.AML2;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AML1.RedCloseAuto;
import org.firstinspires.ftc.teamcode.Manipulators;
import org.firstinspires.ftc.teamcode.VuforiaBM;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name = "Blue Carousel", group = "blue")
public class BlueCarousel extends LinearOpMode {

    enum State {
        ARM,
        DEPOT,                  // Go to depot
        OUTTAKE,                // Outtake cargo
        RETRACT,                // Retract lift
        CAROUSEL_TRAJECTORY,    // Drive to carousel
        CAROUSEL,               // Carousel
        DUCK,                   // Drive and intake for duck
        // DEPOT -> LIFT -> OUTTAKE -> RETRACT
        PARK,                   // Drive
        IDLE,                    // Our bot will enter the IDLE state when done
        TEST
    }

    Pose2d startPose = new Pose2d(-33, 70, Math.toRadians(270));

    // DEPOT TRAJECTORY
    public static double depotX = -14.5;
    public static double depotY = 55;
    public static double depotAng = 270; // Degrees

    // CAROUSEL TRAJECTORY
    public static double carouselX = -65;
    public static double carouselY = 64;
    public static double carouselAng = 305; // Degrees

    // INTAKE TRAJECTORY (FORWARD OUT OF CAROUSEL)
    public static double intakeX = -55;
    public static double intakeY = 50;
    public static double intakeAng = 315; // Degrees

    // INTAKE TRAJECTORY (POINT TO WALL)
    public static double intake2X = -35;
    public static double intake2Y = 67;
    public static double intake2Ang = 315; // Degrees

    // INTAKE TRAJECTORY (BRING TOWARD CAROUSEL)
    public static double intake3X = -54;
    public static double intake3Y = 67;
    public static double intake3Ang = 315; // Degrees

    // WIGGLE ANGLES1 (PICK UP DUCK)
    public static double wiggle1X = -64;
    public static double wiggle1Y = 58;
    public static double wiggle1Ang = 290; // Degrees

    public static double wiggle2X = -46;
    public static double wiggle2Y = 67;
    public static double wiggle2Ang = 270; // Degrees

    // PARK TRAJECTORY
    public static double parkX = -64;
    public static double parkY = 45;
    public static double parkAng = 180; // Degrees

    // Decrease to be closer to the hub
    public static double offsetMid = 3;
    public static double offsetLow = 2.5;


    State currentState = State.IDLE;

    @Override
    public void runOpMode() throws InterruptedException{

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Manipulators manip = new Manipulators(hardwareMap);
        VuforiaBM vuforia = new VuforiaBM(this);

        drive.setPoseEstimate(startPose);

        double waitOuttake = 1.5;
        double waitArm = 1;
        double waitIntake = 4;
        double waitCarousel = 2.5;
        ElapsedTime waitTimer = new ElapsedTime();

        int cycles = 1;

        manip.gate(false);
        telemetry.addLine("init done");
        telemetry.update();

        waitForStart();

        // Start doing vision
        int pos = vuforia.blueCarouselPosition();

        double offset = 0;

        if (pos == 1) offset = offsetLow;
        if (pos == 2) offset = offsetMid;

        // Trajectory to depot
        Trajectory depot = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(depotX, depotY + offset, Math.toRadians(depotAng)))
                .build();

        // Trajectory to carousel
        Trajectory carousel = drive.trajectoryBuilder(depot.end())
                .lineToLinearHeading(new Pose2d(carouselX, carouselY, Math.toRadians(carouselAng)))
                .build();

        // Trajectory intaking duck
        TrajectorySequence intake = drive.trajectorySequenceBuilder(carousel.end())
                .lineToLinearHeading(new Pose2d(intakeX, intakeY, Math.toRadians(intakeAng)))
                .lineToLinearHeading(new Pose2d(intake2X, intake2Y, Math.toRadians(intake2Ang)))
                .lineToLinearHeading(new Pose2d(intake3X, intake3Y, Math.toRadians(intake3Ang)))
                .lineToLinearHeading(new Pose2d(wiggle2X, wiggle2Y, Math.toRadians(wiggle2Ang)),
                        SampleMecanumDrive.getVelocityConstraint(Math.toRadians(240), DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                .build();

        // Trajectory to park
        TrajectorySequence park = drive.trajectorySequenceBuilder(depot.end())
                .lineToLinearHeading(new Pose2d(parkX, parkY, Math.toRadians(parkAng)))
                .build();

        currentState = State.ARM;

        manip.automaticLift(pos);
        waitTimer.reset();

        while (opModeIsActive() && !isStopRequested()) {


            switch (currentState) {

                case ARM:
                    if (waitTimer.seconds() >= waitArm) {
                        drive.followTrajectoryAsync(depot);
                        currentState = State.DEPOT;
                    }

                    break;

                // Drive to depot while arm is moving up
                // When arrived, move gates and outtake

                case DEPOT:

                    if (!drive.isBusy()) {
                        currentState = State.OUTTAKE;

                        waitTimer.reset();
                    }

                    break;

                case OUTTAKE:

                    // After outtaking for <waitOuttake> seconds
                    // stop the outtake and retract arm. Decide whether
                    // to continue to cycle going to carousel or park

                    if (waitTimer.seconds() >= waitOuttake - 1.25) {
                        manip.gatePos(pos);
                        pos = 3;
                    }

                    if (waitTimer.seconds() >= waitOuttake) {
                        currentState = State.RETRACT;

                        manip.automaticLift(0);

                        if (cycles > 0){
                            currentState = State.CAROUSEL_TRAJECTORY;
                            drive.followTrajectoryAsync(carousel);

                            cycles--;
                            pos = 3;
                            waitArm = 0;

                            depot = drive.trajectoryBuilder(startPose)
                                    .lineToLinearHeading(new Pose2d(depotX, depotY, Math.toRadians(depotAng)))
                                    .build();
                        }
                        else {
                            currentState = State.PARK;
                            drive.followTrajectorySequenceAsync(park);
                        }

                        waitTimer.reset();
                    }

                    break;

                case CAROUSEL_TRAJECTORY:

                    // Get to carousel and start spinning

                    if (!drive.isBusy()) {
                        currentState = State.CAROUSEL;

                        manip.stopLift();
                        manip.blueCarousel();

                        waitTimer.reset();
                    }

                    break;
                case CAROUSEL:

                    // Spin the carousel for <waitCarousel> seconds
                    // then stop and start intaking for duck

                    if (waitTimer.seconds() >= waitCarousel) {
                        currentState = State.DUCK;
                        drive.followTrajectorySequenceAsync(intake);

                        manip.carouselStop();

                        manip.intake(false);
                        waitTimer.reset();

                    }

                    break;
                case DUCK:

                    // Intake for duck, lift arm,
                    // and go back to depot

                    if (!drive.isBusy()) {
                        if (waitTimer.seconds() >= waitIntake) {
                            currentState = State.DEPOT;
                            drive.followTrajectoryAsync(depot);

                            manip.gate(false);
                            manip.intakeStop();
                            manip.automaticLift(3);
                        }

                    }

                    // GO BACK TO DEPOT CYCLE

                    break;
                case PARK:

                    if (!drive.isBusy()) {
                        currentState = State.IDLE;
                    }

                    break;

                case IDLE:
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
