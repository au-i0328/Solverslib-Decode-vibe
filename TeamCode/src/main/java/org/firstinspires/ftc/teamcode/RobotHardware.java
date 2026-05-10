package org.firstinspires.ftc.teamcode;

import static com.seattlesolvers.solverslib.util.MathUtils.clamp;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoEx;
import com.seattlesolvers.solverslib.controller.PIDF;
import com.seattlesolvers.solverslib.hardware.MotorEx;
import com.seattlesolvers.solverslib.hardware.RevIMU;
import com.seattlesolvers.solverslib.kinematics.DifferentialOdometry;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.ejml.simple.SimpleMatrix;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.kinematics.HolonomicOdometry;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Rotation2d;

import org.openftc.limelight.Limelight3A;

/**
 * Hardware singleton for the FTC robot.
 * Declares and initializes all motors, servos, sensors, Limelight, IMU, and odometry.
 * Exposes all tuning constants as public static fields for easy access.
 */
public class RobotHardware {

    public static HardwareMap hardwareMap;

    // ==================== MOTORS ====================
    public DcMotor FL;
    public DcMotor FR;
    public DcMotor BL;
    public DcMotor BR;
    public DcMotorEx flywheelL;
    public DcMotorEx flywheelR;
    public DcMotorEx intake;

    // Odometry pods — 2 dead-wheel (left + right parallel)
    public MotorEx odomLeft;   // left parallel pod
    public MotorEx odomRight;   // right parallel pod

    // ==================== SERVOS ====================
    public ServoEx hoodL;
    public ServoEx hoodR;
    public ServoEx gate;

    // ==================== SENSORS ====================
    public RevIMU imu;
    public HolonomicOdometry odometry;

    // ==================== LIMELIGHT ====================
    public Limelight3A limelight;

    // ==================== ALLIANCE ====================
    public enum Alliance {
        RED,
        BLUE
    }
    public static Alliance alliance = Alliance.RED;

    // ==================== ROBOT STATE ====================
    public enum RobotState {
        INIT,
        INTAKE,
        INTAKE_REVERSE,
        ALIGNING,
        ALIGNED,
        SHOOT
    }
    public RobotState currentState = RobotState.INIT;

    // ==================== TUNING CONSTANTS ====================

    // Alliance goal coordinates (field-relative, in inches)
    public static double RED_GOAL_X = 0;
    public static double RED_GOAL_Y = 0;
    public static double BLUE_GOAL_X = 0;
    public static double BLUE_GOAL_Y = 0;

    // Goal heights for limelight distance calculation (mm)
    public static double RED_GOAL_HEIGHT_MM = 0;
    public static double BLUE_GOAL_HEIGHT_MM = 0;

    // Flywheel PIDF coefficients
    public static PIDF FLYWHEEL_L_PIDF = new PIDF(0, 0, 0, 0);
    public static PIDF FLYWHEEL_R_PIDF = new PIDF(0, 0, 0, 0);

    // Flywheel constant target velocity (ticks/sec)
    public static double flywheelTargetVelocity = 0;
    public static double readyToShootVelocityTolerance = 50;
    public static double flywheelVelocityJump = 50;

    // Shoot delay (seconds)
    public static double shootDelay = 2.5;

    // Gate servo positions
    public static double gateOpen = 1.0;
    public static double gateClose = 0.0;

    // Hood hardstops (servo position range)
    public static double hoodMin = 0.0;
    public static double hoodMax = 1.0;

    // Hood interpolation LUT: maps distance (mm) to hood angle (servo position)
    public static InterpLUT hoodAngleLUT = new InterpLUT();

    // Limelight mount angle (degrees)
    public static double limelightMountAngle = 0;
    // Limelight distance offset (mm)
    public static double limelightDistOffset = 0;

    // Hood angle jump for manual adjustment
    public static double hoodAngleJump = 0.01;

    // Hood compensation coefficient for flywheel velocity drop
    public static double hoodCompensationCoefficient = 0;

    // Drive stall current threshold
    public static double driveStallCurrentThreshold = 0;

    // Alignment delay (ms) — time on target before transitioning to ALIGNED
    public static double alignmentDelay = 500;

    // Odometry tuning constants — 2 dead-wheel + IMU fusion
    public static double ODOM_LEFT_WHEEL_DPP = 1.0 / 8192.0;  // distance per pulse (REV encoder)
    public static double ODOM_RIGHT_WHEEL_DPP = 1.0 / 8192.0;
    public static double ODOM_TRACKWIDTH = 18.0;                // inches between left and right pods

    // Input deadzone and drive speed multiplier
    public static double inputDeadzone = 0.1;
    public static double driveSpeedMultiplier = 1.0;

    // ==================== AUTO-INITIALIZATION ====================
    static {
        // Initialize hood angle LUT (distance mm -> servo position)
        // Example values — tune these
        hoodAngleLUT.add(500, 0.10);
        hoodAngleLUT.add(700, 0.15);
        hoodAngleLUT.add(900, 0.22);
        hoodAngleLUT.add(1100, 0.30);
        hoodAngleLUT.add(1300, 0.38);
        hoodAngleLUT.add(1500, 0.45);
        hoodAngleLUT.add(1700, 0.50);
        hoodAngleLUT.add(1900, 0.55);
        hoodAngleLUT.add(2100, 0.60);
        hoodAngleLUT.createLUT();
    }

    // ==================== HARDWARE INITIALIZATION ====================

    public void initHardware(HardwareMap ahwMap) {
        hardwareMap = ahwMap;

        // Enable Lynx bulk caching for all modules
        for (LynxModule module : ahwMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // ---- Drive Motors ----
        FL = ahwMap.get(DcMotor.class, "FL");
        FR = ahwMap.get(DcMotor.class, "FR");
        BL = ahwMap.get(DcMotor.class, "BL");
        BR = ahwMap.get(DcMotor.class, "BR");

        FL.setDirection(DcMotor.Direction.FORWARD);
        FR.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.FORWARD);
        BR.setDirection(DcMotor.Direction.REVERSE);

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // ---- Flywheel Motors ----
        flywheelL = ahwMap.get(DcMotorEx.class, "flywheelL");
        flywheelR = ahwMap.get(DcMotorEx.class, "flywheelR");

        flywheelL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // ---- Intake Motor ----
        intake = ahwMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ---- Odometry Pods (2 dead-wheel + IMU fusion) ----
        odomLeft = new MotorEx(ahwMap, "odomLeft");
        odomLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        odomLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        odomLeft.setDistancePerPulse(ODOM_LEFT_WHEEL_DPP);

        odomRight = new MotorEx(ahwMap, "odomRight");
        odomRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        odomRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        odomRight.setDistancePerPulse(ODOM_RIGHT_WHEEL_DPP);

        // ---- Servos ----
        hoodL = ahwMap.get(ServoEx.class, "hoodL");
        hoodR = ahwMap.get(ServoEx.class, "hoodR");
        gate = ahwMap.get(ServoEx.class, "gate");

        hoodL.setDirection(Servo.Direction.REVERSE);
        gate.setPosition(gateClose);

        // ---- IMU ----
        imu = new RevIMU(ahwMap);
        imu.init();

        // ---- Limelight ----
        limelight = ahwMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();

        // Set default alliance
        alliance = Alliance.RED;
        limelight.setPipelineIndex(0);
    }

    // ==================== BULK CACHE ====================

    public void clearBulkCache() {
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.clearBulkCache();
        }
    }

    // ==================== MOTOR HELPERS ====================

    public void setMotorPower(double power, DcMotor... motors) {
        for (DcMotor m : motors) {
            m.setPower(power);
        }
    }

    public void setMotorMode(DcMotor.RunMode mode, DcMotor... motors) {
        for (DcMotor m : motors) {
            m.setMode(mode);
        }
    }

    public void setMotorTarget(int target, DcMotor... motors) {
        for (DcMotor m : motors) {
            m.setTargetPosition(target);
        }
    }

    // ==================== IMU ====================

    public void resetIMU() {
        imu.reset();
    }

    // ==================== POWER STALL MANAGEMENT ====================

    public boolean isDriveMotorStalling() {
        DcMotorEx flEx = (DcMotorEx) FL;
        DcMotorEx frEx = (DcMotorEx) FR;
        DcMotorEx blEx = (DcMotorEx) BL;
        DcMotorEx brEx = (DcMotorEx) BR;

        return Math.abs(flEx.getCurrent()) > driveStallCurrentThreshold
            || Math.abs(frEx.getCurrent()) > driveStallCurrentThreshold
            || Math.abs(blEx.getCurrent()) > driveStallCurrentThreshold
            || Math.abs(brEx.getCurrent()) > driveStallCurrentThreshold;
    }

    public void reduceStallPower() {
        DcMotorEx flEx = (DcMotorEx) FL;
        DcMotorEx frEx = (DcMotorEx) FR;
        DcMotorEx blEx = (DcMotorEx) BL;
        DcMotorEx brEx = (DcMotorEx) BR;

        if (Math.abs(flEx.getCurrent()) > driveStallCurrentThreshold) {
            FL.setPower(FL.getPower() * 0.5);
        }
        if (Math.abs(frEx.getCurrent()) > driveStallCurrentThreshold) {
            FR.setPower(FR.getPower() * 0.5);
        }
        if (Math.abs(blEx.getCurrent()) > driveStallCurrentThreshold) {
            BL.setPower(BL.getPower() * 0.5);
        }
        if (Math.abs(brEx.getCurrent()) > driveStallCurrentThreshold) {
            BR.setPower(BR.getPower() * 0.5);
        }
    }

    // ==================== GOAL COORDS HELPERS ====================

    public static double getGoalX() {
        return alliance == Alliance.RED ? RED_GOAL_X : BLUE_GOAL_X;
    }

    public static double getGoalY() {
        return alliance == Alliance.RED ? RED_GOAL_Y : BLUE_GOAL_Y;
    }

    public static double getGoalHeightMM() {
        return alliance == Alliance.RED ? RED_GOAL_HEIGHT_MM : BLUE_GOAL_HEIGHT_MM;
    }

    // ==================== LIMELIGHT HELPERS ====================

    public boolean hasValidTarget() {
        return limelight.hasValidTarget();
    }

    public double getLimelightDistanceMM() {
        // ty = vertical offset from crosshair in degrees
        double ty = limelight.getty();
        double mountAngleRad = Math.toRadians(limelightMountAngle);
        double heightDiff = getGoalHeightMM() - 0; // robot height assumed 0 reference
        if (Math.abs(Math.sin(mountAngleRad)) < 0.001) return 0;
        double dist = heightDiff / Math.tan(mountAngleRad + Math.toRadians(ty));
        return dist + limelightDistOffset;
    }

    // ==================== HOOD HELPERS ====================

    public double getHoodAngleFromDistance(double distanceMM) {
        if (hoodAngleLUT.size() == 0) return (hoodMin + hoodMax) / 2;
        return clamp(hoodAngleLUT.get(distanceMM), hoodMin, hoodMax);
    }

    public void setHoodPosition(double position) {
        double clamped = clamp(position, hoodMin, hoodMax);
        hoodL.setPosition(clamped);
        hoodR.setPosition(clamped);
    }
}
