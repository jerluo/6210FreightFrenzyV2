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
        STUCK,
        IDLE                    // Our bot will enter the IDLE state when done
    }

    Pose2d startPose = new Pose2d(5, -70, Math.toRadians(90));

    // DEPOT TRAJECTORY
    public static double depotX = -14.5;
    public static double depotY = -56;
    public static double depotAng = 90; // Degrees

    // WAREHOUSE INSIDE TRAJECTORY (should be right on barrier entrance or exit won't work)
    public static double warehouseInX = 24;
    public static double warehouseInY = -76;

    public static double warehouseOutX = 5;
    public static double warehouseOutY = -68;

    // INTAKE TRAJECTORY
    public static double intakeX = 43;
    public static double intakeY = -76;
    public static double intakeAngle = 0;

    // INTAKE TRAJECTORY
    public static double intakeCycleX = 46;
    public static double intakeCycleY = -68;
    public static double intakeCycleAngle = 10;

    // DEPOT CYCLE TRAJECTORY
    public static double depotCycleX = -5;
    public static double depotCycleY = -54.5;
    public static double depotCycleAng = 110;

    // Decrease to be closer to the hub
    public static double offsetMid = 2.5;
    public static double offsetLow = 3.5;

    int cycles = 4;

    BlueWarehouse.State currentState = BlueWarehouse.State.IDLE;

    @Override
    public void runOpMode() throws InterruptedException{

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Manipulators manip = new Manipulators(hardwareMap);
        VuforiaBM vuforia = new VuforiaBM(this);

        drive.setPoseEstimate(startPose);
        Pose2d poseEstimate;
        boolean stopTrajectory = false;
        double voltage = 100;

        double waitArm = 0.4;
        double waitOuttake = 0.2;
        double waitIntake = 1.75;
        double waitIntakeOut = 0.01;
        double waitLift = 0.75;
        ElapsedTime waitTimer = new ElapsedTime();
        ElapsedTime matchTime = new ElapsedTime();

        manip.gate(false);
        telemetry.addLine("init done");
        telemetry.update();

        waitForStart();

        matchTime.reset();

        // Do vision
        int pos = vuforia.blueCarouselPosition();

        double offset = 0;

        if (pos == 1) offset = offsetLow;
        if (pos == 2) offset = offsetMid;

        // Trajectory to depot
        Trajectory depot = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(depotX, depotY + offset, Math.toRadians(depotAng)))
                .build();

        // Trajectory to warehouse
        TrajectorySequence warehouseIn = drive.trajectorySequenceBuilder(depot.end())
                .setReversed(true)
                //.splineTo(new Vector2d(warehouseInX, warehouseInY), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(warehouseInX, warehouseInY, Math.toRadians(180)), Math.toRadians(0))
                .splineTo(new Vector2d(intakeX, intakeY), Math.toRadians(0))
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

        currentState = BlueWarehouse.State.ARM;

        waitTimer.reset();

        double cycleX = 0;

        while (opModeIsActive() && !isStopRequested()) {

            poseEstimate = drive.getPoseEstimate();

            switch (currentState) {

                case ARM:

                    manip.automaticLift(pos);

                    if (waitTimer.seconds() >= waitArm) {

                        drive.followTrajectoryAsync(depot);
                        currentState = BlueWarehouse.State.DEPOT;
                        waitTimer.reset();

                    }

                    break;

                case DEPOT:

                    manip.automaticLift(pos);

                    // Go to depot and start outtaking
                    if (pos != 3) {
                        if (waitTimer.seconds() >= 1.6) {
                            manip.gatePos(pos);
                        }
                    }

                    else if (waitTimer.seconds() >= 1) {

                        manip.gatePos(pos);

                    }

                    if (!drive.isBusy() || poseEstimate.getY() > depotY - 1) {
                        currentState = BlueWarehouse.State.OUTTAKE;
                        manip.gate(true);
                        waitTimer.reset();
                    }

                    break;

                case OUTTAKE:

                    // Outtake for <waitOuttake> seconds
                    // Then stop intake and retract
                    // Go into warehouse trajectory

                    // waitTimer.seconds() >= waitOuttake
                    if (waitTimer.seconds() >= waitOuttake) {
                        currentState = BlueWarehouse.State.WAREHOUSE_IN;

                        voltage = manip.getVoltage();

                        pos = 3;

                        drive.followTrajectorySequenceAsync(warehouseIn);

                        waitTimer.reset();
                    }

                    break;

                case WAREHOUSE_IN:

                    // Intake <waitIntake> seconds in
                    // Then go to next trajectory and stop intake
                    // Decide whether to continue doing cycles or go to idle

                    manip.automaticLift(0);

                    if (waitTimer.seconds() >= waitIntake) {

                        manip.intake(false, 1);
                        // Check if freight inside the bucket -> stops intake
                        if (manip.senseColor() && poseEstimate.getX() > warehouseInX){
                            manip.intakeStop();
                            stopTrajectory = true;
                            drive.followTrajectorySequence(null);
                        }

                    }




                    // If voltage spike, outtake fast and then continue intaking
                    if (manip.getVoltage() < voltage - 2) {
                        currentState = BlueWarehouse.State.STUCK;
                        waitTimer.reset();
                    }

                    // || poseEstimate.getX() > depotCycleX + cycleX
                    if (!drive.isBusy() || stopTrajectory) {

                        stopTrajectory = false;

                        // Continue
                        if (cycles > 1 && matchTime.seconds() > 4) {
                            currentState = BlueWarehouse.State.WAREHOUSE_OUT;
                            cycleX += 3.5;

                            warehouseOut = drive.trajectorySequenceBuilder(poseEstimate)
                                    .setReversed(false)
                                    .splineToConstantHeading(new Vector2d(warehouseInX, warehouseInY), Math.toRadians(180))
                                    .splineTo(new Vector2d(depotCycleX, depotCycleY), Math.toRadians(depotCycleAng))
                                    .build();

                            warehouseIn = drive.trajectorySequenceBuilder(warehouseOut.end())
                                    .setReversed(true)
                                    //.splineTo(new Vector2d(warehouseInX, warehouseInY), Math.toRadians(0))
                                    .splineToSplineHeading(new Pose2d(warehouseInX, warehouseInY, Math.toRadians(180)), Math.toRadians(0))
                                    //.lineToSplineHeading(new Pose2d(intakeCycleX + cycleX, warehouseInY, Math.toRadians(165)))
                                    .splineTo(new Vector2d(intakeX + cycleX, intakeY), Math.toRadians(0))
                                    .build();

                            drive.followTrajectorySequenceAsync(warehouseOut);

                            waitTimer.reset();
                            cycles--;

                        }

                        else if (cycles > 0 ) {
                            currentState = BlueWarehouse.State.WAREHOUSE_OUT;

                            warehouseOut = drive.trajectorySequenceBuilder(poseEstimate)
                                    .setReversed(false)
                                    .splineToConstantHeading(new Vector2d(warehouseInX, warehouseInY), Math.toRadians(180))
                                    .splineTo(new Vector2d(depotCycleX, depotCycleY), Math.toRadians(depotCycleAng))
                                    .build();

                            warehouseIn = drive.trajectorySequenceBuilder(warehouseOut.end())
                                    .setReversed(true)
                                    .splineToSplineHeading(new Pose2d(warehouseInX, warehouseInY, Math.toRadians(180)), Math.toRadians(0))
                                    //.splineTo(new Vector2d(warehouseInX, warehouseInY), Math.toRadians(0))
                                    .splineTo(new Vector2d(intakeX, intakeY ), Math.toRadians(0))
                                    .build();

                            drive.followTrajectorySequenceAsync(warehouseOut);

                            waitTimer.reset();
                            cycles--;

                        }

                        // Stop
                        else {
                            currentState = BlueWarehouse.State.IDLE;

                            manip.intakeStop();
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

                        manip.intake(true, 1);

                    }

                    if (poseEstimate.getX() < warehouseInX - 6) {

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

                    if (poseEstimate.getY() > depotCycleY - 2) manip.gate(true);

                    if (!drive.isBusy() || poseEstimate.getY() > depotCycleY - 1) {
                        currentState = BlueWarehouse.State.OUTTAKE;


                        waitTimer.reset();
                    }

                    break;

                case STUCK:
                    // Outtake for 0.5 seconds then start intaking
                    // Go back to warehouse in trajectory
                    // Should still be following trajectory as this occurs
                    manip.intake(true, 1);

                    if (waitTimer.seconds() > 0.25) {
                        manip.intake(false, 1);
                        currentState = BlueWarehouse.State.WAREHOUSE_IN;
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



            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("state", currentState);
            telemetry.addData("color", manip.colorValue());
            telemetry.addData("voltage", manip.getVoltage());
            telemetry.addData("voltage check: ", voltage);
            telemetry.addData("velocity", manip.intakeVelocity());
            telemetry.addData("match time", matchTime.seconds());
            telemetry.update();
        }


    }
}