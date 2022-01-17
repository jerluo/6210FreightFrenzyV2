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
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name = "Blue Warehouse", group = "blue")
public class BlueWarehouse extends LinearOpMode {

    enum State {
        DEPOT,                  // Go to depot and lift while (line)
        OUTTAKE,                // Outtake cargo
        WAREHOUSE_IN,           // Drive to warehouse and lower lift while (spline)
        WAREHOUSE_OUT,          // Drive out of warehouse (spline)
        // GO BACK TO OUTTAKE FOR MY CYCLES
        IDLE                    // Our bot will enter the IDLE state when done
    }

    Pose2d startPose = new Pose2d(10, 70, Math.toRadians(270));

    // DEPOT TRAJECTORY
    public static double depotX = -3;
    public static double depotY = 50;
    public static double depotAng = 251.5; // Degrees

    // WAREHOUSE INSIDE TRAJECTORY (should be right on barrier entrance or exit won't work)
    public static double warehouseInX = 25;
    public static double warehouseInY = 71;

    public static double warehouseOutX = 5;
    public static double warehouseOutY = 68;

    // INTAKE TRAJECTORY
    public static double intakeX = 55;
    public static double intakeY = 71;

    // DEPOT CYCLE TRAJECTORY
    public static double depotCycleX = 0;
    public static double depotCycleY = 48;
    public static double depotCycleAng = 270;

    public static double offsetMid = 3.8;
    public static double offsetLow = 3.8;

    int cycles = 2;

    State currentState = State.IDLE;

    @Override
    public void runOpMode() throws InterruptedException{

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Manipulators manip = new Manipulators(hardwareMap);
        VuforiaBM vuforia = new VuforiaBM(this);

        drive.setPoseEstimate(startPose);

        double waitOuttake = 1;
        double waitIntake = 2;
        double waitIntakeOut = 0.25;
        double waitLift = 2;
        ElapsedTime waitTimer = new ElapsedTime();

        manip.gate(false);
        telemetry.addLine("init done");
        telemetry.update();

        waitForStart();

        // Do vision
        int pos = vuforia.blueWarehousePosition();

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
                //.splineTo(new Vector2d(warehouseOutX, warehouseOutY), Math.toRadians(0))
                .splineTo(new Vector2d(warehouseInX, warehouseInY), Math.toRadians(0))
                .splineTo(new Vector2d(intakeX, intakeY), Math.toRadians(0))
                .build();

        // Trajectory to out of warehouse
        TrajectorySequence warehouseOut = drive.trajectorySequenceBuilder(warehouseIn.end())
                .setReversed(false)
                .splineTo(new Vector2d(warehouseInX, warehouseInY), Math.toRadians(180))
                .splineTo(new Vector2d(depotCycleX, depotCycleY), Math.toRadians(depotCycleAng))
                .build();

        currentState = State.DEPOT;
        drive.followTrajectoryAsync(depot);

        manip.automaticLift(pos);

        int cycleX = 3;

        while (opModeIsActive() && !isStopRequested()) {


            switch (currentState) {

                case DEPOT:

                    // Go to depot and start outtaking

                    if (!drive.isBusy()) {
                        currentState = State.OUTTAKE;

                        manip.gatePos(pos);

                        pos = 3;

                        waitTimer.reset();
                    }

                    break;

                case OUTTAKE:

                    // Outtake for <waitOuttake> seconds
                    // Then stop intake and retract
                    // Go into warehouse trajectory

                    if (waitTimer.seconds() >= waitOuttake) {
                        // Trajectory
                        currentState = State.WAREHOUSE_IN;
                        drive.followTrajectorySequenceAsync(warehouseIn);

                        // Manipulator
                        manip.automaticLift(0);

                        waitTimer.reset();
                    }

                    break;

                case WAREHOUSE_IN:

                    // Intake <waitIntake> seconds in
                    // Then go to next trajectory and stop intake
                    // Decide whether to continue doing cycles or go to idle

                    if (waitTimer.seconds() >= waitIntake) {

                        //manip.intake(false);

                    }


                    if (!drive.isBusy()) {

                        // Continue
                        if (cycles > 0 ) {
                            currentState = State.WAREHOUSE_OUT;
                            drive.followTrajectorySequenceAsync(warehouseOut);

                            manip.gate(false);
                            //manip.intakeStop();

                            waitTimer.reset();
                            cycles--;

                            cycleX += 3;

                            /*
                            warehouseIn = drive.trajectorySequenceBuilder(depot.end())
                                    .setReversed(true)
                                    .splineTo(new Vector2d(warehouseOutX, warehouseOutY), Math.toRadians(warehouseOutAng))
                                    .splineTo(new Vector2d(warehouseInX, warehouseInY), Math.toRadians(warehouseInAng))
                                    .splineTo(new Vector2d(intakeX + cycleX, intakeY), Math.toRadians(intakeAng))
                                    .build();

                            warehouseOut = drive.trajectorySequenceBuilder(warehouseIn.end())
                                    .setReversed(false)
                                    .splineTo(new Vector2d(warehouseInX, warehouseInY), Math.toRadians(warehouseInAng))
                                    .splineTo(new Vector2d(warehouseOutX, warehouseOutY), Math.toRadians(warehouseOutAng))
                                    .splineTo(new Vector2d(intakeX, intakeY), Math.toRadians(intakeAng))
                                    .build();*/
                        }

                        // Stop
                        else {
                            currentState = State.IDLE;

                            //manip.intakeStop();
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

                        //manip.intake(true);

                    }

                    if (waitTimer.seconds() >= waitLift) {

                        //manip.intakeStop();
                        manip.automaticLift(3);

                    }

                    if (!drive.isBusy()) {
                        currentState = State.OUTTAKE;

                        manip.gate(true);

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
            telemetry.update();
        }


    }
}
