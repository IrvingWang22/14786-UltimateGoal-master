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
    public static double RED_SHOOTING_Y = -39;

    public static double RED_ENDING_X = LAUNCH_LINE_X; // STARTING X FOR TELEOP + ENDING X FOR AUTON
    public static double RED_ENDING_Y = RED_SHOOTING_Y; // STARTING Y FOR TELEOP + ENDING Y FOR AUTON

    // from left to right

    public static double RED_POWERSHOT_X = -2;

    public static double RED_POWERSHOT_Y_1 = -12;
    public static double RED_POWERSHOT_Y_2 = -13;
    public static double RED_POWERSHOT_Y_3 = -18;

    public static double RED_POWERSHOT_STRAFE_DISTANCE = 6;

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
        Mechanisms mech = new Mechanisms(hardwareMap);

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
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -gamepad1.left_stick_y,
                                    -gamepad1.left_stick_x,
                                    -gamepad1.right_stick_x
                            )
                    );

                    // Top and Left Buttons
                    if (gamepad1.x) mech.runIntake(Mechanisms.intakeState.IN);
                    if (gamepad1.b) mech.runIntake(Mechanisms.intakeState.OUT);
                    if (gamepad1.y) mech.runIntake(Mechanisms.intakeState.OFF);


                    if (gamepad2.back) {
                        mech.stickOne.setPosition(0.8);
                        telemetry.addLine("50%");
                        telemetry.addLine();
                    }

                    if (gamepad2.start) {
                        mech.stickOne.setPosition(0.35);
                        telemetry.addLine("50%");
                        telemetry.addLine();
                    }

                    if (gamepad2.y) mech.wobbleArmControl(Mechanisms.wobbleArmPos.UP);
                    if (gamepad2.a) mech.wobbleArmControl(Mechanisms.wobbleArmPos.DOWN);
                    //if (gamepad2.dpad_left) mech.wobbleArmControl(Mechanisms.wobbleArmPos.OVER);

                    if (gamepad1.right_trigger > 0.5) mech.pushRings();
                    if (gamepad1.left_trigger > 0.5) mech.pushRing();

                    if (gamepad2.b) mech.wobbleControl(Mechanisms.wobblePos.OPEN);
                    if (gamepad2.x) mech.wobbleControl(Mechanisms.wobblePos.CLOSE);

                    // Gamepad 2 - Michael/Atharv
                    if (gamepad2.dpad_up) mech.setShooter(Mechanisms.motorPower.HIGH);
                    if (gamepad2.dpad_down) mech.setShooter(Mechanisms.motorPower.OFF);
                    if (gamepad2.dpad_left) mech.setShooter(Mechanisms.motorPower.MED);

                    /*
                    if (gamepad2.left_bumper) {
                        Trajectory traj = drive.trajectoryBuilder(poseEstimate)
                                .splineTo(POWERSHOT_1, TARGET_ANGLE)
                                .build();

                        drive.followTrajectoryAsync(traj);

                        currentMode = Mode.AUTOMATIC_CONTROL;

                    //} else if (gamepad2.dpad_right) {
                        //Trajectory traj = drive.trajectoryBuilder(poseEstimate);
                        //drive.followTrajectoryAsync(traj);

                        currentMode = Mode.AUTOMATIC_CONTROL;

                    } else if (gamepad2.left_bumper) {
                        Trajectory traj = drive.trajectoryBuilder(poseEstimate)
                                .splineTo(POWERSHOT_3, TARGET_ANGLE)
                                .build();

                        drive.followTrajectoryAsync(traj);

                                //.splineTo(POWERSHOT_2, TARGET_ANGLE)
                                //.build();

                        currentMode = Mode.AUTOMATIC_CONTROL;

                    } else if (gamepad2.right_bumper) {
                        Trajectory traj = drive.trajectoryBuilder(poseEstimate)
                                .splineTo(SHOOTING_POSITION, TARGET_ANGLE)
                                .build();

                        drive.followTrajectoryAsync(traj);

                        currentMode = Mode.AUTOMATIC_CONTROL;
                    }
                    */
                    // TRIPLE Powershot
                    if (gamepad2.y && gamepad2.b) {
                        Trajectory traj = drive.trajectoryBuilder(poseEstimate)
                                .splineTo(POWERSHOT_POSITION1, TARGET_ANGLE)
                                .build();
                        Trajectory traj2 = drive.trajectoryBuilder(traj.end())
                                .strafeRight(RED_POWERSHOT_STRAFE_DISTANCE+2)
                                .build();
                        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                                .strafeRight(RED_POWERSHOT_STRAFE_DISTANCE+1)
                                .build();

                        mech.setShooter(Mechanisms.motorPower.MED);
                        mech.wait(1000);

                        drive.followTrajectory(traj);
                        telemetry.addLine("first traj");
                        mech.wait(1000);
                        mech.pushRing();
                        mech.wait(1000);

                        drive.followTrajectory(traj2);
                        telemetry.addLine("second traj");
                        mech.wait(500);
                        mech.pushRing();
                        mech.wait(500);

                        drive.followTrajectory(traj3);
                        telemetry.addLine("third traj");
                        mech.wait(500);
                        mech.pushRing();
                        mech.wait(3000);

                        mech.setShooter(Mechanisms.motorPower.OFF);

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