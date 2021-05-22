package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.nextcore.Mechanisms;
import org.firstinspires.ftc.teamcode.nextcore.ObjectDetection;

import java.util.List;

@Config
@Autonomous(name = "A - Sight Red")
public class SightRed extends LinearOpMode {


    // FIELD SPECIFIC DATA
    public static double MAX_VALUE = 72;
    public double MIN_VALUE = -MAX_VALUE;

    public static double TILE = 24;
    public static double HALF_TILE = TILE / 2;
    public static double ONE_HALF_TILE = TILE + HALF_TILE;

    public static double ANGLE_SCALE = 16 / 15;

    public static double TAPE_WIDTH = 2;

    // launch line ~ 80 from audience perimeter wall, tape width is 2, want to be on the middle
    public static double LAUNCH_LINE_X = 80 - MAX_VALUE + (TAPE_WIDTH / 2);

    // ROBOT SPECIFIC DATA
    public static double ROBOT_LENGTH = 18; // LENGTH OF WHEELS SIDE
    public static double ROBOT_WIDTH = ROBOT_LENGTH; // LENGTH OF INTAKE SIDE

    // RED SPECIFIC DATA
    public static double RED_STARTING_X = -63;  // STARTING X FOR AUTON
    public static double RED_STARTING_Y = -25.5;  // STARTING Y FOR AUTON

    public static double RED_JUNCTION_X = -12;
    public static double RED_JUNCTION_Y = -12;

    public static double RED_JUNCTION_X2 = 36;
    public static double RED_JUNCTION_Y2 =  0;

    public static double RED_JUNCTION_X3 = -53;
    public static double RED_JUNCTION_Y3 =  -25;

    public static double RED_SHOOTING_X = -1;
    public static double RED_SHOOTING_Y = -34;

    public static double RED_ENDING_X = LAUNCH_LINE_X; // STARTING X FOR TELEOP + ENDING X FOR AUTON
    public static double RED_ENDING_Y = RED_SHOOTING_Y; // STARTING Y FOR TELEOP + ENDING Y FOR AUTON

    public static double RED_WOBBLE_X_0 = HALF_TILE; //12
    public static double RED_WOBBLE_Y_0 = -ONE_HALF_TILE; //-36

    public static double RED_WOBBLE_X_1 = ONE_HALF_TILE; //36
    public static double RED_WOBBLE_Y_1 = -HALF_TILE; //-12

    public static double RED_WOBBLE_X_4 = 2 * TILE + HALF_TILE; //60
    public static double RED_WOBBLE_Y_4 = -ONE_HALF_TILE; //-12

    public double RED_WOBBLE_X, RED_WOBBLE_Y;

    public static double RED_SECOND_WOBBLE_X = -TILE*2;
    public static double RED_SECOND_WOBBLE_Y = -ONE_HALF_TILE; //36

    // from left to right
    public static double RED_POWERSHOT_X = RED_SHOOTING_X;

    public static double RED_POWERSHOT_Y_1 = -31;
    public static double RED_POWERSHOT_Y_2 = -37;
    public static double RED_POWERSHOT_Y_3 = -42;


    private boolean autonRunning = true;

    @Override
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Mechanisms mech = new Mechanisms(hardwareMap);
        ObjectDetection od = new ObjectDetection(hardwareMap);

        Pose2d startPose = new Pose2d(RED_STARTING_X, RED_STARTING_Y, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        Pose2d poseEstimate = drive.getPoseEstimate();

        waitForStart();
        if (isStopRequested()) return;

        if (opModeIsActive()) {
            while (opModeIsActive() && autonRunning) {
                if (od.tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = od.tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
//                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            od.data(i, recognition, telemetry);
                            i++;
                            if (recognition.getLabel().equals("Quad")) {
                                // QUAD RINGS
                                telemetry.addData("QUAD FOUND", recognition.getConfidence());
                                RED_WOBBLE_X = RED_WOBBLE_X_4;
                                RED_WOBBLE_Y = RED_WOBBLE_Y_4;
                                break;
                            } else if (recognition.getLabel().equals("Single")) {
                                // SINGLE RING
                                telemetry.addData("SINGLE FOUND", recognition.getConfidence());
                                RED_WOBBLE_X = RED_WOBBLE_X_1;
                                RED_WOBBLE_Y = RED_WOBBLE_Y_1;
                                break;
                            } else {
                                // NO RINGS
                                telemetry.addLine("NONE FOUND");
                                RED_WOBBLE_X = RED_WOBBLE_X_0;
                                RED_WOBBLE_Y = RED_WOBBLE_Y_0;
                                break;
                            }

                        }
                        telemetry.update();

//                        //first wobble (all)
//                        Trajectory dropWobble = drive.trajectoryBuilder(startPose)
//                                .splineTo(new Vector2d(RED_JUNCTION_X, RED_JUNCTION_Y), Math.toRadians(0))
//                                .lineTo(new Vector2d(RED_WOBBLE_X, RED_WOBBLE_Y))
//                                .addDisplacementMarker(() -> {
//                                    mech.setShooter(Mechanisms.motorPower.HIGH);
//                                })
//                                .build();
//
//                        //shoot first rings (all)
//                        Trajectory shootRings1 = drive.trajectoryBuilder(dropWobble.end())
//                                .lineTo(new Vector2d(RED_SHOOTING_X, RED_SHOOTING_Y))
//                                .build();
//
//                        //second set of rings (single and quad)
//                        Trajectory intakeRings = drive.trajectoryBuilder(shootRings1.end())
//                                .addDisplacementMarker(() -> {
//                                    mech.runIntake(Mechanisms.intakeState.IN);
//                                })
//                                .back(2 * TILE) //need to slow down
//                                .build();
//
//                        Trajectory shootRings2 = drive.trajectoryBuilder(intakeRings.end())
//                                .lineTo(new Vector2d(RED_SHOOTING_X, RED_SHOOTING_Y))
//                                .build();
//
//                        //last ring (quad only)
//                        Trajectory intakeLastRing = drive.trajectoryBuilder(shootRings2.end())
//                                .addDisplacementMarker(() -> {
//                                    mech.runIntake(Mechanisms.intakeState.IN);
//                                })
//                                .back((2*TILE)+6)
//                                .build();
//
//                        Trajectory shootRings3 = drive.trajectoryBuilder(intakeLastRing.end())
//                                .lineTo(new Vector2d(RED_SHOOTING_X, RED_SHOOTING_Y))
//                                .build();
//

                        //Second Wobble (all)
//                        Trajectory getSecondWobble = drive.trajectoryBuilder(poseEstimate)
//                                .splineToConstantHeading(new Vector2d(RED_SECOND_WOBBLE_X, RED_SECOND_WOBBLE_Y), Math.toRadians(0))
//                                .build();
//
//                        Trajectory getSecondWobbleNONE = drive.trajectoryBuilder(shootRings1.end())
//                                .splineToConstantHeading(new Vector2d(RED_SECOND_WOBBLE_X, RED_SECOND_WOBBLE_Y), Math.toRadians(0))
//                                .build();
//                        Trajectory getSecondWobbleSINGLE = drive.trajectoryBuilder(shootRings2.end())
//                                .splineToConstantHeading(new Vector2d(RED_SECOND_WOBBLE_X, RED_SECOND_WOBBLE_Y), Math.toRadians(0))
//                                .build();
//                        Trajectory getSecondWobbleQUAD = drive.trajectoryBuilder(shootRings3.end())
//                                .splineToConstantHeading(new Vector2d(RED_SECOND_WOBBLE_X, RED_SECOND_WOBBLE_Y), Math.toRadians(0))
//                                .build();
//
//                        Trajectory dropSecondWobble = drive.trajectoryBuilder(getSecondWobble.end())
//                                .lineTo(new Vector2d(RED_WOBBLE_X, RED_WOBBLE_Y))
//                                .build();
//
//                        //park (all)
//                        Trajectory park = drive.trajectoryBuilder(dropSecondWobble.end())
//                                .addDisplacementMarker(() -> {
//                                    mech.setShooter(Mechanisms.motorPower.OFF);
//                                })
//                                .splineTo(new Vector2d(RED_ENDING_X, RED_STARTING_Y), Math.toRadians(0))
//                                .build();



                        //drive to first wobble drop location
//                        drive.followTrajectory(dropWobble);
                        //drop first wobble
//                        mech.wait(500);
//                        mech.wobbleArmControl(Mechanisms.wobbleArmPos.DOWN);
//                        mech.wait(1000);
//                        mech.wobbleControl(Mechanisms.wobblePos.OPEN);
//                        mech.wait(500);
//                        mech.wobbleArmControl(Mechanisms.wobbleArmPos.UP);
//
//                        //drive to shoot first rings
//                        drive.followTrajectory(shootRings1);
//                        //shoot first rings
//                        mech.wait(500);
//                        mech.pushRings();
//                        mech.wait(500);
//
//                        //see if we need to shoot a second set of rings (is skipped if no rings detected)
//                        if (detected.equals("SINGLE") || detected.equals("QUAD")) {
//
//                            //drive to and intake second set of rings
//                            drive.followTrajectory(intakeRings);
//
//                            //turn off intake
//                            mech.wait(500);
//                            mech.runIntake(Mechanisms.intakeState.OFF);
//
//                            //drive to shoot second set of rings
//                            drive.followTrajectory(shootRings2);
//
//                            //shoot second set of rings
//                            mech.wait(500);
//                            mech.pushRings();
//                            mech.wait(500);
//                        }
//
//                        if (detected.equals("QUAD")) {
//                            //drive to and intake last ring
//                            drive.followTrajectory(intakeLastRing);
//
//                            //turn off intake
//                            mech.wait(500);
//                            mech.runIntake(Mechanisms.intakeState.OFF);
//
//                            //drive to shoot last ring
//                            drive.followTrajectory(shootRings3);
//
//                            //shoot last ring
//                            mech.wait(500);
//                            mech.pushRing();
//                            mech.wait(500);
//                        }

                        /*
                        //drive to get second wobble
                        poseEstimate = drive.getPoseEstimate();
                        drive.followTrajectory(getSecondWobble);
                        //pick up second wobble
                        mech.wait(500);
                        mech.wobbleControl(Mechanisms.wobblePos.CLOSE);
                        mech.wait(500);

                        //drive to drop second wobble
                        drive.followTrajectory(dropSecondWobble);
                        //drop second wobble
                        mech.wait(500);
                        mech.wobbleArmControl(Mechanisms.wobbleArmPos.DOWN);
                        mech.wait(1000);
                        mech.wobbleControl(Mechanisms.wobblePos.OPEN);
                        mech.wait(500);

                        drive.followTrajectory(park);
                           */

                        Trajectory dropWobble1 = drive.trajectoryBuilder(startPose)
                                .splineTo(new Vector2d(RED_JUNCTION_X, RED_JUNCTION_Y), Math.toRadians(0))
                                .splineTo(new Vector2d(RED_WOBBLE_X, RED_WOBBLE_Y),Math.toRadians(0))
                                .addDisplacementMarker(() -> {
                                    mech.setShooter(Mechanisms.motorPower.HIGH);
                                })
                                .build();

                        /*
                        Trajectory dropWobble = drive.trajectoryBuilder(startPose)
                                .splineTo(new Vector2d(RED_JUNCTION_X, RED_JUNCTION_Y), Math.toRadians(0))
                                .splineTo(new Vector2d(RED_WOBBLE_X, RED_WOBBLE_Y),Math.toRadians(0))
                                .addDisplacementMarker(() -> {
                                    mech.setShooter(Mechanisms.motorPower.HIGH);
                                })
                                .build();
                        */

                        Trajectory shootRings1 = drive.trajectoryBuilder(dropWobble1.end())
                                .splineToConstantHeading(new Vector2d(RED_JUNCTION_X2, RED_JUNCTION_Y2), Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(RED_SHOOTING_X, RED_SHOOTING_Y), Math.toRadians(0))
                                .build();


                        Trajectory intakeRings = drive.trajectoryBuilder(shootRings1.end())
                                .addDisplacementMarker(() -> {
                                    mech.runIntake(Mechanisms.intakeState.IN);
                                })
                                .back(2 * TILE)
                                .build();

                        Trajectory shootRings2 = drive.trajectoryBuilder(intakeRings.end())
                                .splineToConstantHeading(new Vector2d(RED_SHOOTING_X, RED_SHOOTING_Y), Math.toRadians(0))
                                .build();
                        /*
                        Trajectory getSecondWobble = drive.trajectoryBuilder(shootRings2.end())
                                .splineToConstantHeading(new Vector2d(RED_JUNCTION_X3, RED_JUNCTION_Y3), Math.toRadians(0))
                                .build();

                        Trajectory getSecondWobble2 = drive.trajectoryBuilder(getSecondWobble.end())
                                .strafeRight(20)
                                .build();

                         Trajectory dropSecondWobble = drive.trajectoryBuilder(getSecondWobble2.end())
                                .splineToConstantHeading(new Vector2d(RED_JUNCTION_X2, RED_JUNCTION_Y2), Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(RED_WOBBLE_X, RED_WOBBLE_Y), Math.toRadians(0))
                                .build();
                        */
                        Trajectory park = drive.trajectoryBuilder(shootRings2.end())
                                .addDisplacementMarker(() -> {
                                    mech.setShooter(Mechanisms.motorPower.OFF);
                                })
                                .splineToConstantHeading(new Vector2d(RED_ENDING_X, RED_ENDING_Y), Math.toRadians(0))
                                .build();

                        drive.followTrajectory(dropWobble1);

                        mech.wait(500);
                        mech.wobbleArmControl(Mechanisms.wobbleArmPos.DOWN);
                        mech.wait(500);
                        mech.wobbleControl(Mechanisms.wobblePos.OPEN);
                        mech.wait(500);
                        mech.wobbleArmControl(Mechanisms.wobbleArmPos.UP);
                        mech.wait(500);

                        drive.followTrajectory(shootRings1);

                        mech.wait(500);
                        mech.pushRings();
                        mech.wait(500);

                        drive.followTrajectory(intakeRings);

                        mech.wait(500);
                        mech.runIntake(Mechanisms.intakeState.OFF);

                        drive.followTrajectory(shootRings2);

                        mech.wait(500);
                        mech.pushRings();
                        mech.wait(500);

                        //drive to get second wobble
                        /*
                        drive.followTrajectory(getSecondWobble);
                        //pick up second wobble
                        mech.wait(500);
                        mech.wobbleArmControl(Mechanisms.wobbleArmPos.DOWN);
                        mech.wait(500);
                        drive.followTrajectory(getSecondWobble2);
                        mech.wait(500);
                        mech.wobbleControl(Mechanisms.wobblePos.CLOSE);
                        mech.wait(500);

                        mech.wobbleArmControl(Mechanisms.wobbleArmPos.UP);
                        mech.wait(500);

                        //drive to drop second wobble
                        drive.followTrajectory(dropSecondWobble);

                        //drop second wobble
                        mech.wobbleArmControl(Mechanisms.wobbleArmPos.DOWN);
                        mech.wait(500);
                        mech.wobbleControl(Mechanisms.wobblePos.OPEN);
                        mech.wait(1000);
                        mech.wobbleArmControl(Mechanisms.wobbleArmPos.UP);
                        mech.wait(500);
                           */
                        drive.followTrajectory(park);

                        autonRunning = false;
                    }
                }
            }
        }

        od.shutdown();
    }
}