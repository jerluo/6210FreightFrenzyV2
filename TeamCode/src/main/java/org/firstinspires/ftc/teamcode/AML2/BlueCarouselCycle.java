package org.firstinspires.ftc.teamcode.AML2;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Manipulators;
import org.firstinspires.ftc.teamcode.VuforiaBM;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name = "Hydra Special", group = "blue")
public class BlueCarouselCycle extends LinearOpMode {

    enum State {
        WAIT,
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
        WAREHOUSE_IN,
        WAREHOUSE_OUT
    }

    Pose2d startPose = new Pose2d(-33, 70, Math.toRadians(270));
    Pose2d poseEstimate;

    // DEPOT TRAJECTORY
    public static double depotX = -14.5;
    public static double depotY = 55; //before change this was 50
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
    public static double wiggle2Y = 68;
    public static double wiggle2Ang = 270; // Degrees

    // PARK TRAJECTORY
    public static double parkX = -64;
    public static double parkY = 45;
    public static double parkAng = 180; // Degrees

    // Decrease to be closer to the hub
    public static double offsetMid = 3;
    public static double offsetLow = 2.5;

    // WAREHOUSE INSIDE TRAJECTORY (should be right on barrier entrance or exit won't work)
    public static double warehouseInX = 27;
    public static double warehouseInY = 74;

    // INTAKE TRAJECTORY
    public static double intakeCycleX = 45;
    public static double intakeCycleY = 74;
    public static double intakeCycleAngle = 345; // 345

    // DEPOT CYCLE TRAJECTORY
    public static double depotCycleX = -8;
    public static double depotCycleY = 56;
    public static double depotCycleAng = 250;


    public static double startWait = 0;


    State currentState = State.IDLE;

    @Override
    public void runOpMode() throws InterruptedException{

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Manipulators manip = new Manipulators(hardwareMap);
        VuforiaBM vuforia = new VuforiaBM(this);

        drive.setPoseEstimate(startPose);

        double waitOuttake = 0.5;
        double waitArm = 0.5;
        double waitIntake = 4;
        double waitIntakeCycle = 2;
        double waitCarousel = 3.5;
        double waitIntakeOut = 0.2;
        ElapsedTime waitTimer = new ElapsedTime();

        int cycles = 3;
        int cycleX = 3;

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
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                .build();

        // Trajectory to warehouse
        TrajectorySequence warehouseIn = drive.trajectorySequenceBuilder(depot.end())
                .setReversed(true)
                //.splineTo(new Vector2d(warehouseInX, warehouseInY), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(warehouseInX, warehouseInY, Math.toRadians(180)), Math.toRadians(0))
                .splineTo(new Vector2d(intakeCycleX + cycleX, intakeCycleY), Math.toRadians(0))
                .build();

        TrajectorySequence warehouseOut = drive.trajectorySequenceBuilder(warehouseIn.end())
                .setReversed(false)
                .splineTo(new Vector2d(warehouseInX, warehouseInY), Math.toRadians(180))
                .splineTo(new Vector2d(depotCycleX, depotCycleY), Math.toRadians(depotCycleAng))
                .build();


        currentState = State.WAIT;

        manip.automaticLift(pos);
        waitTimer.reset();

        while (opModeIsActive() && !isStopRequested()) {

            poseEstimate = drive.getPoseEstimate();

            switch (currentState) {

                case WAIT:
                    if (waitTimer.seconds() >= startWait) {
                        currentState = State.ARM;
                    }

                case ARM:
                    if (waitTimer.seconds() >= waitArm) {
                        drive.followTrajectoryAsync(depot);
                        currentState = State.DEPOT;
                        waitTimer.reset();

                    }

                    break;

                // Drive to depot while arm is moving up
                // When arrived, move gates and outtake

                case DEPOT:

                    if (pos != 3) {
                        if (waitTimer.seconds() >= 1.6) {
                            manip.gatePos(pos);
                        }
                    }

                    else if (waitTimer.seconds() >= 1) {

                        manip.gatePos(pos);

                    }

                    if (!drive.isBusy()) {
                        currentState = State.OUTTAKE;
                        manip.gate(true);
                        waitTimer.reset();
                    }

                    break;

                case OUTTAKE:

                    // After outtaking for <waitOuttake> seconds
                    // stop the outtake and retract arm. Decide whether
                    // to continue to cycle going to carousel or park

                    if (waitTimer.seconds() >= waitOuttake) {

                        manip.automaticLift(0);
                        pos = 3;

                        if (cycles == 3){
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
                            currentState = State.WAREHOUSE_IN;

                            drive.followTrajectorySequenceAsync(warehouseIn);
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

                        manip.slowIntake();
                        waitTimer.reset();

                    }

                    break;
                case DUCK:

                    // Intake for duck, lift arm,
                    // and go back to depot

                    if (!drive.isBusy()) {
                        if (waitTimer.seconds() >= waitIntakeCycle) {
                            currentState = State.DEPOT;
                            drive.followTrajectoryAsync(depot);

                            manip.gate(false);
                            manip.intakeStop();
                            manip.automaticLift(3);
                        }

                    }

                    // GO BACK TO DEPOT CYCLE

                    break;

                case WAREHOUSE_IN:

                    if (waitTimer.seconds() >= waitIntake) {

                        manip.intake(false);
                        // Check if freight inside the bucket -> stops intake
                        if (manip.senseColor()){
                            manip.intakeStop();
                            drive.followTrajectorySequence(null);
                        }

                    }

                    if (!drive.isBusy()) {

                        // Continue
                        if (cycles > 0) {
                            currentState = State.WAREHOUSE_OUT;
                            cycleX += 2;

                            warehouseOut = drive.trajectorySequenceBuilder(poseEstimate)
                                    .setReversed(false)
                                    .splineTo(new Vector2d(warehouseInX, warehouseInY), Math.toRadians(180))
                                    .splineTo(new Vector2d(depotCycleX, depotCycleY), Math.toRadians(depotCycleAng))
                                    .build();

                            warehouseIn = drive.trajectorySequenceBuilder(warehouseOut.end())
                                    .setReversed(true)
                                    //.splineTo(new Vector2d(warehouseInX, warehouseInY), Math.toRadians(0))
                                    .splineToSplineHeading(new Pose2d(warehouseInX, warehouseInY, Math.toRadians(180)), Math.toRadians(0))
                                    //.lineToSplineHeading(new Pose2d(intakeCycleX + cycleX, warehouseInY, Math.toRadians(165)))
                                    .splineTo(new Vector2d(intakeCycleX + cycleX, intakeCycleY), Math.toRadians(intakeCycleAngle))
                                    .build();

                            drive.followTrajectorySequenceAsync(warehouseOut);

                            waitTimer.reset();
                            cycles--;

                        }
                        // Stop
                        else {
                            currentState = State.IDLE;

                            manip.intakeStop();
                            manip.gate(false);
                        }
                    }

                    break;

                case WAREHOUSE_OUT:

                    if (waitTimer.seconds() >= waitIntakeOut) {

                        manip.intake(true);

                    }

                    if (poseEstimate.getX() < warehouseInX) {

                        manip.gate(false);
                        manip.automaticLift(3);
                        manip.intakeStop();

                    }

                    /*
                    if (waitTimer.seconds() >= waitLift) {

                        manip.gate(false);
                        manip.automaticLift(3);
                        manip.intakeStop();

                    }*/

                    if (poseEstimate.getY() < depotCycleY + 2) manip.gate(true);

                    if (!drive.isBusy() || poseEstimate.getY() < depotCycleY) {
                        currentState = State.OUTTAKE;

                        waitTimer.reset();

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
            telemetry.addData("position", pos);

            telemetry.update();
        }


    }
}
