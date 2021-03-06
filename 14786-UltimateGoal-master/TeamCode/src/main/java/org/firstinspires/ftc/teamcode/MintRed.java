package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.nextcore.Mechanisms;

@Config
@TeleOp(name = "T - Mint Red")
public class MintRed extends LinearOpMode {

    // FIELD SPECIFIC DATA
    public static double MAX_VALUE = 72;

    public static double TILE = 24;
    public static double HALF_TILE = TILE / 2;

    public static double TAPE_WIDTH = 2;

    public static double LAUNCH_LINE_X = 80 - MAX_VALUE + (TAPE_WIDTH / 2);

    public static double RED_SHOOTING_X = -1;
    public static double RED_SHOOTING_Y = -34;

    public static double RED_ENDING_X = 16; // STARTING X FOR TELEOP + ENDING X FOR AUTON
    public static double RED_ENDING_Y = -42; // STARTING Y FOR TELEOP + ENDING Y FOR AUTON

    // from left to right
    public static double RED_POWERSHOT_X = -2;

    public static double RED_POWERSHOT_Y_1 = -12;
    public static double RED_POWERSHOT_Y_2 = -13;
    public static double RED_POWERSHOT_Y_3 = -18;

    public static double RED_POWERSHOT_STRAFE_DISTANCE = 7;

    public static boolean SLOWMODE = false;

    enum Mode {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL
    }

    Mode currentMode = Mode.DRIVER_CONTROL;

    Vector2d SHOOTING_POSITION = new Vector2d(RED_SHOOTING_X, RED_SHOOTING_Y);

    Vector2d POWERSHOT_POSITION1 = new Vector2d(RED_POWERSHOT_X, RED_POWERSHOT_Y_1);

    Vector2d POWERSHOT_1 = new Vector2d(RED_POWERSHOT_X, RED_POWERSHOT_Y_1);
    Vector2d POWERSHOT_2 = new Vector2d(RED_POWERSHOT_X, RED_POWERSHOT_Y_2);
    Vector2d POWERSHOT_3 = new Vector2d(RED_POWERSHOT_X, RED_POWERSHOT_Y_3);

    double TARGET_ANGLE = Math.toRadians(0); // might need to change

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Pose2d startPose = new Pose2d(RED_ENDING_X, RED_ENDING_Y, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            drive.update();

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Print pose to telemetry
            telemetry.addData("mode", currentMode);
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();

            switch (currentMode) {
                case DRIVER_CONTROL:
                    // Gamepad 1 - Ian
                    //Mecanum Drive (GAMEPAD1)
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -gamepad1.left_stick_y,
                                    -gamepad1.left_stick_x,
                                    -gamepad1.right_stick_x
                            )
                    );

                    //Intake States (GAMEPAD1)

                    //Toggle slow mode (GAMEPAD1)
                    /*
                    if (gamepad1.a && SLOWMODE == true) {
                        drive.setWeightedDrivePower(
                                new Pose2d(
                                        -(0.25)*gamepad1.left_stick_y, //rangeclip
                                        -(0.25)*gamepad1.left_stick_x,
                                        -(0.25)*gamepad1.right_stick_x
                                )
                        );
                        SLOWMODE = !SLOWMODE;
                    } else if (gamepad1.a && SLOWMODE == false) {
                        drive.setWeightedDrivePower(
                                new Pose2d(
                                        -gamepad1.left_stick_y,
                                        -gamepad1.left_stick_x,
                                        -gamepad1.right_stick_x
                                )
                        );
                        SLOWMODE = !SLOWMODE;
                    }
                    */

                    // Gamepad 2 - Michael/Atharv
                    //Wobble Arm (GAMEPAD2)


                    //AUTOMATIC CONTROL (GAMEPAD1)
                    //High Goal

                    if (gamepad1.right_bumper) {
                        /*
                        Trajectory traj = drive.trajectoryBuilder(poseEstimate)
                                .splineTo(SHOOTING_POSITION, TARGET_ANGLE)
                                .build();
                        drive.followTrajectory(traj);

                        currentMode = Mode.AUTOMATIC_CONTROL;
                        */

                    }

                    // TRIPLE Powershot
                    if (gamepad1.left_bumper) {
                        /*
                        Trajectory traj = drive.trajectoryBuilder(poseEstimate)
                                .splineTo(POWERSHOT_POSITION1, TARGET_ANGLE)
                                .build();
                        Trajectory traj2 = drive.trajectoryBuilder(traj.end())
                                .strafeRight(RED_POWERSHOT_STRAFE_DISTANCE)
                                .build();
                        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                                .strafeRight(RED_POWERSHOT_STRAFE_DISTANCE)
                                .build();

                        mech.setShooter(Mechanisms.motorPower.MED);
                        mech.wait(1000);

                        drive.followTrajectory(traj);
                        mech.wait(1000);
                        /*
                        telemetry.addLine("first powershot");
                        mech.wait(1000);
                        mech.pushRing();
                        mech.wait(1000);
                        drive.followTrajectory(traj2);
                        telemetry.addLine("second powershot");
                        mech.wait(500);
                        mech.pushRing();
                        mech.wait(500);
                        drive.followTrajectory(traj3);
                        telemetry.addLine("third powershot");
                        mech.wait(500);
                        mech.pushRing();
                        mech.wait(3000);
                        mech.setShooter(Mechanisms.motorPower.OFF);
                           */

                        currentMode = Mode.AUTOMATIC_CONTROL;

                    }

                    break;
                case AUTOMATIC_CONTROL:
                    // If x is pressed, we break out of the automatic following
                    if (gamepad1.start || gamepad1.start) {
                        drive.cancelFollowing();
                        currentMode = Mode.DRIVER_CONTROL;
                    }

                    // If drive finishes its task, cede control to the driver
                    if (!drive.isBusy()) {
                        currentMode = Mode.DRIVER_CONTROL;
                    }
                    break;
            }

        }
    }

}