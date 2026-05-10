package org.firstinspires.ftc.teamcode;

import static com.seattlesolvers.solverslib.util.MathUtils.clamp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.seattlesolvers.solverslib.controller.PIDF;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Rotation2d;
import com.seattlesolvers.solverslib.kinematics.HolonomicOdometry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Main TeleOp OpMode for the FTC robot.
 * Orchestrates state machine, drivetrain, flywheel PIDF, hood control, and power management.
 */
public class TeleOpOpMode extends LinearOpMode {

    private RobotHardware robot = new RobotHardware();
    private MecanumDriveExt mecanum = new MecanumDriveExt();
    private GamepadEx driverOp;
    private GamepadEx toolOp;

    // Flywheel PIDF controllers (one per motor)
    private PIDF flywheelLPidf = new PIDF(
        RobotHardware.FLYWHEEL_L_PIDF.getK_P(),
        RobotHardware.FLYWHEEL_L_PIDF.getK_I(),
        RobotHardware.FLYWHEEL_L_PIDF.getK_D(),
        RobotHardware.FLYWHEEL_L_PIDF.getK_F()
    );
    private PIDF flywheelRPidf = new PIDF(
        RobotHardware.FLYWHEEL_R_PIDF.getK_P(),
        RobotHardware.FLYWHEEL_R_PIDF.getK_I(),
        RobotHardware.FLYWHEEL_R_PIDF.getK_D(),
        RobotHardware.FLYWHEEL_R_PIDF.getK_F()
    );

    // Shoot timer
    private ElapsedTime shootTimer = new ElapsedTime();
    private boolean shootTimerStarted = false;

    // Alignment timer
    private ElapsedTime alignmentTimer = new ElapsedTime();
    private boolean alignmentTimerStarted = false;

    // Hood manual offset
    private double hoodOffset = 0.0;

    // Lock toggle (gamepad2 triangle)
    private boolean lockToggled = false;
    private boolean lastTriangle2 = false;

    // Vector weight driver toggle (optional feature)
    private static final boolean USE_VECTOR_WEIGHT_DRIVER = true;

    // Odometry
    private HolonomicOdometry odometry;

    // ==================== ALLIANCE SELECTION ====================

    private void selectAlliance() {
        telemetry.addLine("SELECT ALLIANCE:");
        telemetry.addLine("  Triangle -> RED  (pipeline 0)");
        telemetry.addLine("  Circle   -> BLUE (pipeline 10)");
        telemetry.update();

        boolean selected = false;
        while (!selected && !isStopRequested()) {
            if (gamepad1.triangle) {
                RobotHardware.alliance = RobotHardware.Alliance.RED;
                robot.limelight.setPipelineIndex(0);
                telemetry.addLine("Alliance: RED");
                telemetry.update();
                sleep(300);
                selected = true;
            } else if (gamepad1.circle) {
                RobotHardware.alliance = RobotHardware.Alliance.BLUE;
                robot.limelight.setPipelineIndex(10);
                telemetry.addLine("Alliance: BLUE");
                telemetry.update();
                sleep(300);
                selected = true;
            }
        }
    }

    // ==================== READY TO SHOOT CHECK ====================

    private boolean isReadyToShoot() {
        double actualVel = (robot.flywheelL.getVelocity() + robot.flywheelR.getVelocity()) / 2.0;
        double targetVel = RobotHardware.flywheelTargetVelocity;
        boolean velocityInRange = Math.abs(actualVel - targetVel) < RobotHardware.readyToShootVelocityTolerance;
        boolean hasTarget = robot.hasValidTarget();
        double distance = robot.getLimelightDistanceMM();
        boolean distanceValid = distance > 0 && distance < 5000;
        return velocityInRange && hasTarget && distanceValid;
    }

    // ==================== AUTO ALIGN PROCESS ====================

    private boolean performAutoAlign() {
        // Check if AprilTag is visible
        if (robot.hasValidTarget()) {
            // Use limelight crosshair servoing to turn to correct heading
            double tx = robot.limelight.gettx();
            // Proportional control for horizontal alignment
            double turn = clamp(tx * 0.05, -0.3, 0.3);
            mecanum.setAutoAlignTurn(turn);
            return Math.abs(tx) < 1.0; // Consider aligned when tx is close to 0
        } else {
            // No tag visible — use odometry pose to turn toward general goal direction
            if (odometry != null) {
                Pose2d pose = odometry.getPose();
                double goalX = RobotHardware.getGoalX();
                double goalY = RobotHardware.getGoalY();
                double dx = goalX - pose.getX();
                double dy = goalY - pose.getY();
                double desiredHeading = Math.atan2(dy, dx);
                double currentHeading = pose.getRotation().getRadians();
                double headingError = desiredHeading - currentHeading;
                // Normalize to -pi to pi
                while (headingError > Math.PI) headingError -= 2 * Math.PI;
                while (headingError < -Math.PI) headingError += 2 * Math.PI;
                double turn = clamp(headingError * 0.5, -0.3, 0.3);
                mecanum.setAutoAlignTurn(turn);
                return Math.abs(headingError) < 0.1; // Consider aligned when heading error is small
            }
            return false;
        }
    }

    // ==================== OPTIONAL: LAUNCH ZONE CHECK ====================
    // NOTE: The full "run to pose" optional feature described in Instructions.md is NOT
    // fully implemented. The code below provides vector-weight driver (vector addition), which
    // blends a launch-zone approach vector with joystick input. A true "run to pose" command
    // would autonomously drive the robot to the launch zone; this is not yet wired up.

    private boolean isInLaunchZone() {
        if (odometry == null) return true;
        Pose2d pose = odometry.getPose();
        double goalX = RobotHardware.getGoalX();
        // Check if robot x is near goal x (within 5cm = ~2 inches)
        return Math.abs(pose.getX() - goalX) < 3.0;
    }

    private void updateVectorWeightDriver() {
        if (!USE_VECTOR_WEIGHT_DRIVER) return;
        if (robot.currentState != RobotHardware.RobotState.ALIGNING) {
            mecanum.setVectorWeightDriver(0, 0);
            return;
        }
        if (isInLaunchZone()) {
            mecanum.setVectorWeightDriver(0, 0);
        } else {
            // Calculate vector toward launch zone and blend with joystick
            Pose2d pose = odometry != null ? odometry.getPose() : new Pose2d(0, 0, Rotation2d.fromDegrees(0));
            double goalX = RobotHardware.getGoalX();
            double dx = goalX - pose.getX();
            double dy = 0 - pose.getY();
            double dist = Math.sqrt(dx * dx + dy * dy);
            if (dist > 0.1) {
                double vx = (dx / dist) * RobotHardware.vectorWeightDriver;
                double vy = (dy / dist) * RobotHardware.vectorWeightDriver;
                mecanum.setVectorWeightDriver(vx, vy);
            } else {
                mecanum.setVectorWeightDriver(0, 0);
            }
        }
    }

    // ==================== MAIN ====================

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware
        robot.initHardware(hardwareMap);
        mecanum.init(robot);

        // Initialize gamepads
        driverOp = new GamepadEx(gamepad1);
        toolOp = new GamepadEx(gamepad2);

        // Initialize flywheel PIDF setpoints
        flywheelLPidf.setSetPoint(RobotHardware.flywheelTargetVelocity);
        flywheelRPidf.setSetPoint(RobotHardware.flywheelTargetVelocity);

        // Initialize odometry — 1 parallel + 1 perpendicular dead-wheel track x/y position.
        // IMU fusion is applied separately in the main loop: odometry tracks x/y via dead wheels,
        // and the IMU heading is injected each iteration via updatePose() to correct drift.
        // NOTE: if HolonomicOdometry does not accept an IMU reference in its constructor, the fusion
        // approach here (correcting only the heading via updatePose) is the correct workaround.
        odometry = new HolonomicOdometry(
            () -> robot.odomLeft.getDistance(),
            () -> robot.odomCenter.getDistance(),
            RobotHardware.ODOM_TRACKWIDTH,
            RobotHardware.ODOM_CENTER_WHEEL_OFFSET
        );

        // Alliance selection
        selectAlliance();

        robot.currentState = RobotHardware.RobotState.INIT;
        telemetry.addLine("Press PLAY to start!");
        telemetry.update();
        waitForStart();

        // Main loop
        while (opModeIsActive()) {
            // Clear bulk cache at the top of each loop
            robot.clearBulkCache();

            // Update odometry with IMU-fused heading
            // Only correct the heading; preserve dead-wheel x/y tracking from odometry pods
            if (odometry != null) {
                Pose2d currentPose = odometry.getPose();
                double imuHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                Pose2d imuPose = new Pose2d(currentPose.getX(), currentPose.getY(), Rotation2d.fromRadians(imuHeading));
                odometry.updatePose(imuPose);
            }

            // Read gamepads
            driverOp.readButtons();
            toolOp.readButtons();

            // ==================== STATE MACHINE ====================

            // LEFT_BUMPER is king — highest priority
            if (driverOp.isDown(GamepadKeys.Button.LEFT_BUMPER)) {
                if (robot.currentState != RobotHardware.RobotState.INTAKE_REVERSE) {
                    robot.currentState = RobotHardware.RobotState.INTAKE_REVERSE;
                    shootTimerStarted = false;
                }
            } else {
                // Left bumper released — handle transitions
                switch (robot.currentState) {
                    case INTAKE_REVERSE:
                        robot.currentState = RobotHardware.RobotState.INTAKE;
                        break;

                    case ALIGNING:
                    case ALIGNED:
                    case SHOOT:
                        // Left bumper released → go to INTAKE
                        robot.currentState = RobotHardware.RobotState.INTAKE;
                        shootTimerStarted = false;
                        break;

                    default:
                        break;
                }
            }

            // Right bumper → INTAKE
            if (driverOp.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                robot.currentState = RobotHardware.RobotState.INTAKE;
                shootTimerStarted = false;
                mecanum.setAligningActive(false);
            }

            // Right trigger held → ALIGNING
            if (driverOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5f) {
                robot.currentState = RobotHardware.RobotState.ALIGNING;
                shootTimerStarted = false;
                mecanum.setAligningActive(true);

                // Auto-align process
                boolean aligned = performAutoAlign();

                // Alignment timer
                if (aligned) {
                    if (!alignmentTimerStarted) {
                        alignmentTimer.reset();
                        alignmentTimerStarted = true;
                    }
                    if (alignmentTimer.milliseconds() >= RobotHardware.alignmentDelay) {
                        robot.currentState = RobotHardware.RobotState.ALIGNED;
                        alignmentTimerStarted = false;
                    }
                } else {
                    alignmentTimerStarted = false;
                }
            }

            // Right trigger released → INTAKE
            if (driverOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) <= 0.5f) {
                if (robot.currentState == RobotHardware.RobotState.ALIGNING
                 || robot.currentState == RobotHardware.RobotState.ALIGNED) {
                    robot.currentState = RobotHardware.RobotState.INTAKE;
                    shootTimerStarted = false;
                    mecanum.setAligningActive(false);
                }
            }

            // Target lost in ALIGNED → ALIGNING
            // Only trigger this if the right trigger is still held; otherwise the trigger-release
            // handler (above) owns the state transition and takes priority.
            boolean rightTriggerHeld = driverOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5f;
            if (robot.currentState == RobotHardware.RobotState.ALIGNED
                    && !robot.hasValidTarget()
                    && rightTriggerHeld) {
                robot.currentState = RobotHardware.RobotState.ALIGNING;
                mecanum.setAligningActive(true);
                alignmentTimerStarted = false;
            }

            // Left trigger → SHOOT (only if ready); Share button bypasses all other isReadyToShoot checks
            if (driverOp.wasJustPressed(GamepadKeys.Trigger.LEFT_TRIGGER) && driverOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5f) {
                if (isReadyToShoot() || sharePressed) {
                    robot.currentState = RobotHardware.RobotState.SHOOT;
                    shootTimerStarted = true;
                    shootTimer.reset();
                    // Vibrate gamepad with 2 short bursts of 50ms
                    gamepad1.rumbleBlips(2, 50);
                }
            }

            // Shoot timer
            if (robot.currentState == RobotHardware.RobotState.SHOOT && shootTimerStarted) {
                if (shootTimer.seconds() >= RobotHardware.shootDelay) {
                    robot.currentState = RobotHardware.RobotState.INTAKE;
                    shootTimerStarted = false;
                }
            }

            // Circle → reset to INIT
            if (toolOp.wasJustPressed(GamepadKeys.Button.CIRCLE)) {
                robot.currentState = RobotHardware.RobotState.INIT;
                shootTimerStarted = false;
                alignmentTimerStarted = false;
                mecanum.setShootLock(false);
                mecanum.setAligningActive(false);
            }

            // ==================== DRIVETRAIN ====================

            // Triangle → vibrate 200ms, reset IMU
            boolean imuReset = driverOp.wasJustPressed(GamepadKeys.Button.TRIANGLE);
            if (imuReset) {
                gamepad1.rumble(200);
                mecanum.resetIMU();
            }

            // Shoot lock
            mecanum.setShootLock(robot.currentState == RobotHardware.RobotState.SHOOT);

            // Update vector weight driver
            updateVectorWeightDriver();

            // Drive
            mecanum.drive(gamepad1, imuReset);

            // Set locking based on state
            switch (robot.currentState) {
                case INTAKE:
                case INTAKE_REVERSE:
                case INIT:
                    mecanum.setLockEnabled(true);
                    break;
                case ALIGNING:
                    mecanum.setLockEnabled(false);
                    break;
                case ALIGNED:
                    mecanum.setLockEnabled(true);
                    break;
                case SHOOT:
                    // Lock is forced by shoot lock
                    break;
            }

            // ==================== FLYWHEEL PIDF ====================

            // Run flywheel at constant target velocity all the time
            double flywheelTarget = RobotHardware.flywheelTargetVelocity;

            // Dpad up/down adjusts target velocity
            if (toolOp.isDown(GamepadKeys.Button.DPAD_UP)) {
                flywheelTarget += RobotHardware.flywheelVelocityJump;
            }
            if (toolOp.isDown(GamepadKeys.Button.DPAD_DOWN)) {
                flywheelTarget -= RobotHardware.flywheelVelocityJump;
            }

            flywheelLPidf.setSetPoint(flywheelTarget);
            flywheelRPidf.setSetPoint(flywheelTarget);

            double lPower = flywheelLPidf.calculate(robot.flywheelL.getVelocity());
            double rPower = flywheelRPidf.calculate(robot.flywheelR.getVelocity());

            robot.flywheelL.setPower(clamp(lPower, -1.0, 1.0));
            robot.flywheelR.setPower(clamp(rPower, -1.0, 1.0));

            // ==================== HOOD ====================

            double hoodBase = 0.5; // default

            if (robot.hasValidTarget()) {
                double distance = robot.getLimelightDistanceMM();
                if (distance > 0) {
                    hoodBase = robot.getHoodAngleFromDistance(distance);
                }
            }

            // Manual hood offset from gamepad2 dpad left/right
            if (toolOp.isDown(GamepadKeys.Button.DPAD_LEFT)) {
                hoodOffset += RobotHardware.hoodAngleJump;
            }
            if (toolOp.isDown(GamepadKeys.Button.DPAD_RIGHT)) {
                hoodOffset -= RobotHardware.hoodAngleJump;
            }

            double hoodPosition = hoodBase + hoodOffset;

            // In SHOOT state: apply velocity compensation
            if (robot.currentState == RobotHardware.RobotState.SHOOT) {
                double actualVel = (robot.flywheelL.getVelocity() + robot.flywheelR.getVelocity()) / 2.0;
                double velocityDrop = flywheelTarget - actualVel;
                double compensation = RobotHardware.hoodCompensationCoefficient * velocityDrop;
                compensation = clamp(compensation, -0.1, 0.1); // cap compensation
                hoodPosition += compensation;
            }

            robot.setHoodPosition(hoodPosition);

            // ==================== INTAKE ====================

            switch (robot.currentState) {
                case INTAKE:
                    robot.intake.setPower(1.0);
                    break;
                case INTAKE_REVERSE:
                    robot.intake.setPower(-1.0);
                    break;
                case SHOOT:
                    robot.intake.setPower(1.0);
                    break;
                default:
                    robot.intake.setPower(0.0);
                    break;
            }

            // ==================== GATE ====================

            if (robot.currentState == RobotHardware.RobotState.SHOOT) {
                robot.gate.setPosition(RobotHardware.gateOpen);
            } else {
                robot.gate.setPosition(RobotHardware.gateClose);
            }

            // ==================== POWER STALL MANAGEMENT ====================

            if (robot.isDriveMotorStalling()) {
                robot.reduceStallPower();
            }

            // ==================== GAMEPAD2 TOGGLE: DRIVE LOCK ====================

            boolean triangle2 = toolOp.isDown(GamepadKeys.Button.TRIANGLE);
            if (triangle2 && !lastTriangle2) {
                lockToggled = !lockToggled;
            }
            lastTriangle2 = triangle2;

            // Share button: force isReadyToShoot = true while pressed (per instructions: "Share → set isReadyToShoot = true while pressed, neglecting other factors")
            boolean sharePressed = toolOp.isDown(GamepadKeys.Button.SHARE);

            // ==================== TELEMETRY ====================

            telemetry.addData("State", robot.currentState);
            telemetry.addData("Alliance", RobotHardware.alliance);
            telemetry.addData("Drive State", mecanum.getDriveState());
            telemetry.addData("Flywheel L Vel", "%.1f", robot.flywheelL.getVelocity());
            telemetry.addData("Flywheel R Vel", "%.1f", robot.flywheelR.getVelocity());
            telemetry.addData("Flywheel Target", "%.1f", flywheelTarget);
            telemetry.addData("isReadyToShoot", isReadyToShoot() || sharePressed);
            telemetry.addData("Limelight Valid", robot.hasValidTarget());
            telemetry.addData("Limelight tx", "%.2f", robot.limelight.gettx());
            telemetry.addData("Limelight ty", "%.2f", robot.limelight.getty());
            telemetry.addData("Limelight Dist (mm)", "%.1f", robot.getLimelightDistanceMM());
            telemetry.addData("Hood Position", "%.3f", hoodPosition);
            telemetry.addData("Hood Offset", "%.3f", hoodOffset);
            telemetry.addData("Gate Position", "%.3f", robot.gate.getPosition());
            telemetry.update();
        }

        // Stop everything on exit
        robot.FL.setPower(0);
        robot.FR.setPower(0);
        robot.BL.setPower(0);
        robot.BR.setPower(0);
        robot.flywheelL.setPower(0);
        robot.flywheelR.setPower(0);
        robot.intake.setPower(0);
    }
}
