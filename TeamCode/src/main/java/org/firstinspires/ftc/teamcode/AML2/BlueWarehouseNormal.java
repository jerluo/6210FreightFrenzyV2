package org.firstinspires.ftc.teamcode.AML2;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Manipulators;
import org.firstinspires.ftc.teamcode.VuforiaBM;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name = "Blue Warehouse Normal", group = "blue")
public class BlueWarehouseNormal extends LinearOpMode {

    enum State {
        DEPOT,                  // Go to depot
        OUTTAKE,                // Outtake cargo
        RETRACT,                // Retract lift
        PARK,                   // Drive
        IDLE                    // Our bot will enter the IDLE state when done
    }

    Pose2d startPose = new Pose2d(10, 70, Math.toRadians(270));

    // DEPOT TRAJECTORY
    public static double depotX = -3;
    public static double depotY = 54;
    public static double depotAng = 260; // Degrees

    // PARK TRAJECTORY
    public static double parkX = 55;
    public static double parkY = 71;
    public static double parkAng = 190; // Degrees

    public static double offsetMid = 3.45;
    public static double offsetLow = 3.5;

    public static double warehouseX = 0;
    public static double warehouseY = 70;

    State currentState = State.IDLE;

    @Override
    public void runOpMode() throws InterruptedException{

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Manipulators manip = new Manipulators(hardwareMap);
        VuforiaBM vuforia = new VuforiaBM(this);

        drive.setPoseEstimate(startPose);

        double waitOuttake = 3;
        double waitIntake = 5;
        double waitCarousel = 4;
        ElapsedTime waitTimer = new ElapsedTime();

        int cycles = 1;



        manip.gate(false);
        telemetry.addLine("init done");
        telemetry.update();

        waitForStart();

        // Start doing vision
        int pos = vuforia.blueWarehousePosition();

        double offset = 0;

        if (pos == 1) offset = offsetLow;
        if (pos == 2) offset = offsetMid;

        // Trajectory to depot
        Trajectory depot = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(depotX, depotY + offset, Math.toRadians(depotAng)))
                .build();

        // Trajectory to park
        TrajectorySequence park = drive.trajectorySequenceBuilder(depot.end())
                .lineToLinearHeading(new Pose2d(warehouseX, warehouseY, Math.toRadians(parkAng)))
                .lineToLinearHeading(new Pose2d(parkX, parkY, Math.toRadians(parkAng)))
                .build();

        currentState = State.DEPOT;

        drive.followTrajectoryAsync(depot);
        manip.automaticLift(pos);

        while (opModeIsActive() && !isStopRequested()) {


            switch (currentState) {

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

                    if (waitTimer.seconds() >= waitOuttake - 1) {
                        manip.gatePos(pos);
                    }
                    if (waitTimer.seconds() >= waitOuttake) {
                        currentState = State.RETRACT;

                        manip.automaticLift(0);

                        currentState = State.PARK;
                        drive.followTrajectorySequenceAsync(park);

                        waitTimer.reset();
                    }

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
