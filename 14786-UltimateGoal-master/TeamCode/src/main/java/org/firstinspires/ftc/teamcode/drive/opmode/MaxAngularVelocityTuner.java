package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.Objects;

/**
 * This routine is designed to calculate the maximum angular velocity your bot can achieve under load.
 * <p>
 * Upon pressing start, your bot will turn at max power for RUNTIME seconds.
 * <p>
 * Further fine tuning of MAX_ANG_VEL may be desired.
 */

@Config
@Disabled
@Autonomous(group = "drive")
public class MaxAngularVelocityTuner extends LinearOpMode {
    public static double RUNTIME = 4.0;

    private ElapsedTime timer;
    private double maxAngVelocity = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Your bot will turn at full speed for " + RUNTIME + " seconds.");
        telemetry.addLine("Please ensure you have enough space cleared.");
        telemetry.addLine("");
        telemetry.addLine("Press start when ready.");
        telemetry.update();

        waitForStart();

        telemetry.clearAll();
        telemetry.update();

        drive.setDrivePower(new Pose2d(0, 0, 1));
        timer = new ElapsedTime();

        while (!isStopRequested() && timer.seconds() < RUNTIME) {
            drive.updatePoseEstimate();

            Pose2d poseVelo = Objects.requireNonNull(drive.getPoseVelocity(), "poseVelocity() must not be null. Ensure that the getWheelVelocities() method has been overridden in your localizer.");

            maxAngVelocity = Math.max(poseVelo.getHeading(), maxAngVelocity);
        }

        drive.setDrivePower(new Pose2d());

        telemetry.addData("Max Angular Velocity (rad)", maxAngVelocity);
        telemetry.addData("Max Angular Velocity (deg)", Math.toDegrees(maxAngVelocity));
        telemetry.update();

        while (!isStopRequested()) idle();
    }

    /**
     * This routine is designed to calculate the maximum velocity your bot can achieve under load. It
     * will also calculate the effective kF value for your velocity PID.
     * <p>
     * Upon pressing start, your bot will run at max power for RUNTIME seconds.
     * <p>
     * Further fine tuning of kF may be desired.
     */
    @Config
    //@Disabled
    @Autonomous(group = "drive")
    public static class MaxVelocityTuner extends LinearOpMode {
        public static double RUNTIME = 2.0;

        private ElapsedTime timer;
        private double maxVelocity = 0.0;

        private VoltageSensor batteryVoltageSensor;

        @Override
        public void runOpMode() throws InterruptedException {
            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

            drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

            telemetry.addLine("Your bot will go at full speed for " + RUNTIME + " seconds.");
            telemetry.addLine("Please ensure you have enough space cleared.");
            telemetry.addLine("");
            telemetry.addLine("Press start when ready.");
            telemetry.update();

            waitForStart();

            telemetry.clearAll();
            telemetry.update();

            drive.setDrivePower(new Pose2d(1, 0, 0));
            timer = new ElapsedTime();

            while (!isStopRequested() && timer.seconds() < RUNTIME) {
                drive.updatePoseEstimate();

                Pose2d poseVelo = Objects.requireNonNull(drive.getPoseVelocity(), "poseVelocity() must not be null. Ensure that the getWheelVelocities() method has been overridden in your localizer.");

                maxVelocity = Math.max(poseVelo.vec().norm(), maxVelocity);
            }

            drive.setDrivePower(new Pose2d());

            double effectiveKf = DriveConstants.getMotorVelocityF(veloInchesToTicks(maxVelocity));

            telemetry.addData("Max Velocity", maxVelocity);
            telemetry.addData("Voltage Compensated kF", effectiveKf * batteryVoltageSensor.getVoltage() / 12);
            telemetry.update();

            while (!isStopRequested() && opModeIsActive()) idle();
        }

        private double veloInchesToTicks(double inchesPerSec) {
            return inchesPerSec / (2 * Math.PI * DriveConstants.WHEEL_RADIUS) / DriveConstants.GEAR_RATIO * DriveConstants.TICKS_PER_REV;
        }
    }
}