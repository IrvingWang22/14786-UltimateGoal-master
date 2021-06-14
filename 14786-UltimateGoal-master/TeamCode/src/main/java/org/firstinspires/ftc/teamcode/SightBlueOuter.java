package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.nextcore.Mechanisms;
import org.firstinspires.ftc.teamcode.nextcore.ObjectDetection;

import java.util.List;

@Config
@Autonomous(name = "A - Sight Blue Outer")
public class SightBlueOuter extends LinearOpMode {


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
    public static double BLUE_STARTING_X = -63;  // STARTING X FOR AUTON
    public static double BLUE_STARTING_Y = 50;  // STARTING Y FOR AUTON

    public static double BLUE_JUNCTION_X = -12;
    public static double BLUE_JUNCTION_Y = 60;

    //public static double RED_JUNCTION_X2 = 36;
    //public static double RED_JUNCTION_Y2 =  0;

    //public static double RED_JUNCTION_X3 = -53;
    //public static double RED_JUNCTION_Y3 =  -25;

    public static double BLUE_SHOOTING_X = -4;
    public static double BLUE_SHOOTING_Y = 30;

    public static double BLUE_ENDING_X = 16; // STARTING X FOR TELEOP + ENDING X FOR AUTON
    public static double BLUE_ENDING_Y = 42; // STARTING Y FOR TELEOP + ENDING Y FOR AUTON

    public static double BLUE_WOBBLE_X_0 = 24;
    public static double BLUE_WOBBLE_Y_0 = 48;
    public static double BLUE_WOBBLE_HEADING_0 = Math.toRadians(180);
    public static double BLUE_SHOOTING_HEADING_0 = 10;

    public static double BLUE_WOBBLE_X_1 = 20;
    public static double BLUE_WOBBLE_Y_1 = 24;
    public static double BLUE_WOBBLE_HEADING_1 = Math.toRadians(90);
    public static double BLUE_SHOOTING_HEADING_1 = -6;

    public static double BLUE_WOBBLE_X_4 = 48;
    public static double BLUE_WOBBLE_Y_4 = 48;
    public static double BLUE_WOBBLE_HEADING_4 = Math.toRadians(90);
    public static double BLUE_SHOOTING_HEADING_4 = 0;

    //default position on 0 ring
    public double BLUE_WOBBLE_X = BLUE_WOBBLE_X_0;
    public double BLUE_WOBBLE_Y = BLUE_WOBBLE_Y_0;
    public double BLUE_WOBBLE_HEADING = BLUE_WOBBLE_HEADING_0;
    public static double BLUE_SHOOTING_HEADING = BLUE_SHOOTING_HEADING_0;

    public static double BLUE_SECOND_WOBBLE_X = -TILE*2;
    public static double BLUE_SECOND_WOBBLE_Y = -ONE_HALF_TILE; //36

    // from left to right
    public static double BLUE_POWERSHOT_X = BLUE_SHOOTING_X;

    public static double BLUE_POWERSHOT_Y_1 = 31;
    public static double BLUE_POWERSHOT_Y_2 = 37;
    public static double BLUE_POWERSHOT_Y_3 = 42;

    public static int AUTON_DELAY = 10000;

    private boolean autonRunning = true;

    @Override
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Mechanisms mech = new Mechanisms(hardwareMap);
        ObjectDetection od = new ObjectDetection(hardwareMap);

        Pose2d startPose = new Pose2d(BLUE_STARTING_X, BLUE_STARTING_Y, Math.toRadians(0));

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
                                BLUE_WOBBLE_X = BLUE_WOBBLE_X_4;
                                BLUE_WOBBLE_Y = BLUE_WOBBLE_Y_4;
                                BLUE_WOBBLE_HEADING = BLUE_WOBBLE_HEADING_4;
                                BLUE_SHOOTING_HEADING = BLUE_SHOOTING_HEADING_4;
                                break;
                            } else if (recognition.getLabel().equals("Single")) {
                                // SINGLE RING
                                telemetry.addData("SINGLE FOUND", recognition.getConfidence());
                                BLUE_WOBBLE_X = BLUE_WOBBLE_X_1;
                                BLUE_WOBBLE_Y = BLUE_WOBBLE_Y_1;
                                BLUE_WOBBLE_HEADING = BLUE_WOBBLE_HEADING_1;
                                BLUE_SHOOTING_HEADING = BLUE_SHOOTING_HEADING_1;
                                break;
                            } else {
                                // NO RINGS
                                telemetry.addData("NONE FOUND", recognition.getConfidence());
                                BLUE_WOBBLE_X = BLUE_WOBBLE_X_0;
                                BLUE_WOBBLE_Y = BLUE_WOBBLE_Y_0;
                                BLUE_WOBBLE_HEADING = BLUE_WOBBLE_HEADING_0;
                                BLUE_SHOOTING_HEADING = BLUE_SHOOTING_HEADING_0;
                                break;
                            }

                        }
                        telemetry.update();

                        Trajectory dropWobble = drive.trajectoryBuilder(startPose)
                                .splineToConstantHeading(new Vector2d(BLUE_JUNCTION_X, BLUE_JUNCTION_Y), Math.toRadians(0))
                                .build();

                        Trajectory dropWobble2 = drive.trajectoryBuilder(dropWobble.end())
                                //.splineToConstantHeading(new Vector2d(BLUE_JUNCTION_X, BLUE_JUNCTION_Y), Math.toRadians(0))
                                .lineToLinearHeading(new Pose2d(BLUE_WOBBLE_X, BLUE_WOBBLE_Y, BLUE_WOBBLE_HEADING))
                                .addDisplacementMarker(() -> {
                                    mech.setShooter(Mechanisms.motorPower.HIGH);
                                })
                                .build();


                        Trajectory shootRings1 = drive.trajectoryBuilder(dropWobble2.end())
                                //.splineTo(new Vector2d(BLUE_JUNCTION_X2, BLUE_JUNCTION_Y2), (Math.toRadians(0) + 1e-6))
                                .lineToLinearHeading(new Pose2d(BLUE_SHOOTING_X, BLUE_SHOOTING_Y, Math.toRadians(BLUE_SHOOTING_HEADING)))
                                .build();


                        Trajectory intakeRings = drive.trajectoryBuilder(shootRings1.end())
                                .addDisplacementMarker(() -> {
                                    mech.runIntake(Mechanisms.intakeState.IN);
                                })

                                .back(2 * TILE)
                                //SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                //SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                .build();

                        Trajectory shootRings2 = drive.trajectoryBuilder(intakeRings.end())
                                .splineToConstantHeading(new Vector2d(BLUE_SHOOTING_X, BLUE_SHOOTING_Y), Math.toRadians(0))
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
                                .splineToConstantHeading(new Vector2d(BLUE_ENDING_X, BLUE_ENDING_Y), Math.toRadians(0)) //SPLINE TO CONSTANT
                                .build();

                        //initial delay
                        //mech.wait(AUTON_DELAY);

                        //drop wobble
                        mech.wobbleArmControl(Mechanisms.wobbleArmPos.UP);
                        drive.followTrajectory(dropWobble);
                        drive.followTrajectory(dropWobble2);
                        mech.wait(500);
                        mech.wobbleArmControl(Mechanisms.wobbleArmPos.DOWN);
                        mech.wait(500);
                        mech.wobbleControl(Mechanisms.wobblePos.OPEN);
                        mech.wait(500);
                        mech.wobbleArmControl(Mechanisms.wobbleArmPos.UP);
                        mech.wait(500);

                        //shoot first set of rings
                        drive.followTrajectory(shootRings1);
                        mech.wait(500);
                        mech.pushRings();
                        mech.wait(500);
                        //collect new rings
                        /*
                        drive.followTrajectory(intakeRings);
                        mech.wait(1000);
                        mech.runIntake(Mechanisms.intakeState.OFF);
                        //shoot second set of rings
                        drive.followTrajectory(shootRings2);
                        mech.wait(500);
                        mech.pushRings();
                        mech.wait(500);
                        */
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