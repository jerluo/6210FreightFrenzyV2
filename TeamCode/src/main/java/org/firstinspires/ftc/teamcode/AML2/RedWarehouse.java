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
@Autonomous(name = "Red Warehouse", group = "red")
public class RedWarehouse extends LinearOpMode {

    enum State {
        ARM,
        DEPOT,                  // Go to depot and lift while (line)
        OUTTAKE,                // Outtake cargo
        RETRACT,
        WAREHOUSE_IN,           // Drive to warehouse and lower lift while (spline)
        WAREHOUSE_OUT,          // Drive out of warehouse (spline)
        // GO BACK TO OUTTAKE FOR MY CYCLES
        IDLE                    // Our bot will enter the IDLE state when done
    }

    Pose2d startPose = new Pose2d(5, -70, Math.toRadians(90));

    // DEPOT TRAJECTORY
    public static double depotX = -14.5;
    public static double depotY = -55;
    public static double depotAng = 90; // Degrees

    // WAREHOUSE INSIDE TRAJECTORY (should be right on barrier entrance or exit won't work)
    public static double warehouseInX = 24;
    public static double warehouseInY = -74;

    public static double warehouseOutX = 5;
    public static double warehouseOutY = -68;

    // INTAKE TRAJECTORY
    public static double intakeX = 47;
    public static double intakeY = -74;
    public static double intakeAngle = 0;

    // INTAKE TRAJECTORY
    public static double intakeCycleX = 47;
    public static double intakeCycleY = -67;
    public static double intakeCycleAngle = 25;

    // DEPOT CYCLE TRAJECTORY
    public static double depotCycleX = -5;
    public static double depotCycleY = -54;
    public static double depotCycleAng = 115;

    // Decrease to be closer to the hub
    public static double offsetMid = 2.5;
    public static double offsetLow = 4;

    int cycles = 3;

    State currentState = State.IDLE;

    @Override
    public void runOpMode() throws InterruptedException{

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Manipulators manip = new Manipulators(hardwareMap);
        VuforiaBM vuforia = new VuforiaBM(this);

        drive.setPoseEstimate(startPose);

        double waitArm = 0.5;
        double waitOuttake = 0.2;
        double waitIntake = 1;
        double waitIntakeOut = 0.5;
        double waitLift = 1.5;
        ElapsedTime waitTimer = new ElapsedTime();

        manip.gate(false);
        telemetry.addLine("init done");
        telemetry.update();

        waitForStart();

        // Do vision
        int pos = vuforia.blueCarouselPosition();

        double offset = 0;

        if (pos == 1) offset = offsetLow;
        if (pos == 2) offset = offsetMid;

        // Trajectory to depot
        Trajectory depot = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(depotX, depotY - offset, Math.toRadians(depotAng)))
                .build();

        // Trajectory to warehouse
        TrajectorySequence warehouseIn = drive.trajectorySequenceBuilder(depot.end())
                .setReversed(true)
                .splineTo(new Vector2d(warehouseInX, warehouseInY), Math.toRadians(0))
                .splineTo(new Vector2d(intakeX, intakeY), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(18, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        // Trajectory to out of warehouse
        TrajectorySequence warehouseOut = drive.trajectorySequenceBuilder(warehouseIn.end())
                .setReversed(false)
                .splineTo(new Vector2d(warehouseInX, warehouseInY), Math.toRadians(180))
                .splineTo(new Vector2d(depotCycleX, depotCycleY), Math.toRadians(depotCycleAng))
                .build();

        TrajectorySequence warehouseInCycle = drive.trajectorySequenceBuilder(warehouseOut.end())
                .setReversed(true)
                .splineTo(new Vector2d(warehouseInX, warehouseInY), Math.toRadians(0))
                .splineTo(new Vector2d(intakeX, intakeY), Math.toRadians(0))
                .build();

        currentState = State.ARM;

        manip.automaticLift(pos);
        waitTimer.reset();

        double cycleX = 2;
        double cycleY = 1;

        while (opModeIsActive() && !isStopRequested()) {


            switch (currentState) {

                case ARM:
                    if (waitTimer.seconds() >= waitArm) {
                        drive.followTrajectoryAsync(depot);
                        currentState = State.DEPOT;
                        waitTimer.reset();

                    }

                    break;

                case DEPOT:

                    // Go to depot and start outtaking
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
                        waitTimer.reset();
                    }

                    break;

                case OUTTAKE:

                    // Outtake for <waitOuttake> seconds
                    // Then stop intake and retract
                    // Go into warehouse trajectory

                    if (waitTimer.seconds() >= waitOuttake) {
                        currentState = State.WAREHOUSE_IN;

                        pos = 3;

                        manip.automaticLift(0);

                        drive.followTrajectorySequenceAsync(warehouseIn);

                        waitTimer.reset();
                    }

                    break;

                case WAREHOUSE_IN:

                    // Intake <waitIntake> seconds in
                    // Then go to next trajectory and stop intake
                    // Decide whether to continue doing cycles or go to idle


                    if (waitTimer.seconds() >= waitIntake) {

                        manip.intake(false);

                    }

                    if (manip.senseColor()) manip.intakeStop();

                    if (!drive.isBusy()) {

                        // Continue
                        if (cycles > 1 ) {
                            currentState = State.WAREHOUSE_OUT;
                            drive.followTrajectorySequenceAsync(warehouseOut);

                            waitTimer.reset();
                            cycles--;

                            cycleX += 1.25;


                            warehouseIn = drive.trajectorySequenceBuilder(warehouseOut.end())
                                    .setReversed(true)
                                    .splineTo(new Vector2d(warehouseInX, warehouseInY), Math.toRadians(0))
                                    .splineTo(new Vector2d(intakeCycleX + cycleX, intakeCycleY), Math.toRadians(intakeCycleAngle),
                                            SampleMecanumDrive.getVelocityConstraint(19, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                    .build();

                            warehouseOut = drive.trajectorySequenceBuilder(warehouseIn.end())
                                    .setReversed(false)
                                    .splineTo(new Vector2d(warehouseInX, warehouseInY), Math.toRadians(180))
                                    .splineTo(new Vector2d(depotCycleX, depotCycleY), Math.toRadians(depotCycleAng))
                                    .build();
                        }

                        else if (cycles > 0) {
                            currentState = State.WAREHOUSE_OUT;
                            drive.followTrajectorySequenceAsync(warehouseOut);

                            waitTimer.reset();
                            cycles--;

                            cycleX += 1.25;


                            warehouseIn = drive.trajectorySequenceBuilder(warehouseOut.end())
                                    .setReversed(true)
                                    .splineTo(new Vector2d(warehouseInX, warehouseInY), Math.toRadians(0))
                                    .splineTo(new Vector2d(intakeCycleX, intakeCycleY), Math.toRadians(intakeCycleAngle))
                                    .build();

                            warehouseOut = drive.trajectorySequenceBuilder(warehouseIn.end())
                                    .setReversed(false)
                                    .splineTo(new Vector2d(warehouseInX, warehouseInY), Math.toRadians(180))
                                    .splineTo(new Vector2d(depotCycleX, depotCycleY), Math.toRadians(depotCycleAng))
                                    .build();
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

                    // Drive out of the warehouse (to depot) and
                    // outtake to make sure we picked up only 1 block
                    // Start bring lift up <waitLift> seconds into
                    // trajectory. Depot and then either park or do
                    // another cycle at outtake


                    if (waitTimer.seconds() >= waitIntakeOut) {

                        manip.intake(true);

                    }

                    if (waitTimer.seconds() >= waitLift) {

                        manip.intakeStop();
                        manip.gate(false);
                        manip.automaticLift(3);

                    }

                    if (waitTimer.seconds() >= waitLift+1.25) {

                        manip.gate(true);

                    }

                    if (!drive.isBusy()) {
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
            telemetry.addData("state", currentState);
            telemetry.addData("color", manip.colorValue());
            telemetry.update();
        }


    }
}