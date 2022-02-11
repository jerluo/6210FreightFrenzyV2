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
@Autonomous(name = "Blue Warehouse", group = "blue")
public class BlueWarehouse extends LinearOpMode {

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

    Pose2d startPose = new Pose2d(5, 70, Math.toRadians(270));

    // DEPOT TRAJECTORY
    public static double depotX = -14.5;
    public static double depotY = 55;
    public static double depotAng = 270; // Degrees

    // WAREHOUSE INSIDE TRAJECTORY (should be right on barrier entrance or exit won't work)
    public static double warehouseInX = 27;
    public static double warehouseInY = 74;


    // INTAKE TRAJECTORY
    public static double intakeX = 47;
    public static double intakeY = 74;
    public static double intakeAngle = 0;

    // INTAKE TRAJECTORY
    public static double intakeCycleX = 49;
    public static double intakeCycleY = 68; // 67
    public static double intakeCycleAngle = 345; // 345

    // DEPOT CYCLE TRAJECTORY
    public static double depotCycleX = -5;
    public static double depotCycleY = 54;
    public static double depotCycleAng = 250;

    // Decrease to be closer to the hub
    public static double offsetMid = 3;
    public static double offsetLow = 4.5;

    int cycles = 3;

    State currentState = State.IDLE;

    @Override
    public void runOpMode() throws InterruptedException{

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Manipulators manip = new Manipulators(hardwareMap);
        VuforiaBM vuforia = new VuforiaBM(this);

        drive.setPoseEstimate(startPose);
        Pose2d poseEstimate;
        double voltage = 100;

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
                .splineTo(new Vector2d(warehouseInX, warehouseInY), Math.toRadians(0))
                .splineTo(new Vector2d(intakeX, intakeY), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
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

        while (opModeIsActive() && !isStopRequested()) {

            poseEstimate = drive.getPoseEstimate();

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

                        voltage = manip.getVoltage();

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
                        // Check if freight inside the bucket -> stops intake
                        if (manip.senseColor()){
                            manip.intakeStop();
                            drive.followTrajectorySequence(null);
                        }

                    }




                    // If voltage spike, outtake fast and then continue intaking
                    if (manip.getVoltage() < voltage - 2) {
                        currentState = State.STUCK;
                        waitTimer.reset();
                    }

                    // if getvoltage < voltage - 2
                    // waittime reset
                    // change state
                    // outtake for .5
                    // start intaking again
                    // change state warehouse_in



                    // If velocity slows, assume freight is inside intake, stop trajectory
                    // if velocityIntake < velocity - 2
                    // drive.followTrajectorySequence(null);
                    // waitIntake += 0.5?




                    if (!drive.isBusy()) {

                        // Continue
                        if (cycles > 1 ) {
                            currentState = State.WAREHOUSE_OUT;
                            cycleX += 2;

                            warehouseOut = drive.trajectorySequenceBuilder(poseEstimate)
                                    .setReversed(false)
                                    .splineTo(new Vector2d(warehouseInX, warehouseInY), Math.toRadians(180))
                                    .splineTo(new Vector2d(depotCycleX, depotCycleY), Math.toRadians(depotCycleAng))
                                    .build();

                            warehouseIn = drive.trajectorySequenceBuilder(warehouseOut.end())
                                    .setReversed(true)
                                    .splineTo(new Vector2d(warehouseInX, warehouseInY), Math.toRadians(0))
                                    .splineTo(new Vector2d(intakeCycleX + cycleX, intakeCycleY), Math.toRadians(intakeCycleAngle),
                                            SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                    .build();

                            drive.followTrajectorySequenceAsync(warehouseOut);

                            waitTimer.reset();
                            cycles--;

                        }

                        else if (cycles > 0) {
                            currentState = State.WAREHOUSE_OUT;

                            warehouseOut = drive.trajectorySequenceBuilder(poseEstimate)
                                    .setReversed(false)
                                    .splineTo(new Vector2d(warehouseInX, warehouseInY), Math.toRadians(180))
                                    .splineTo(new Vector2d(depotCycleX, depotCycleY), Math.toRadians(depotCycleAng))
                                    .build();

                            warehouseIn = drive.trajectorySequenceBuilder(warehouseOut.end())
                                    .setReversed(true)
                                    .splineTo(new Vector2d(warehouseInX + cycleX, warehouseInY), Math.toRadians(0))
                                    .splineTo(new Vector2d(intakeX, intakeY), Math.toRadians(0))
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

                    if (waitTimer.seconds() >= waitLift+1.1) {

                        manip.gate(true);

                    }

                    if (!drive.isBusy()) {
                        currentState = State.OUTTAKE;

                        waitTimer.reset();
                    }

                    break;

                case STUCK:
                    // Outtake for 0.5 seconds then start intaking
                    // Go back to warehouse in trajectory
                    // Should still be following trajectory as this occurs
                    manip.intake(true);

                    if (waitTimer.seconds() > 0.25) {
                        manip.intake(false);
                        currentState = State.WAREHOUSE_IN;
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
            telemetry.update();
        }


    }
}