package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Extended Mecanum Drive that wraps the referenced MecanumDrive locking logic,
 * adding shoot lock, input curve application, and vector weight driver.
 */
public class MecanumDriveExt {

    private RobotHardware robot;

    // === Shoot lock ===
    private boolean shootLock = false;

    // === Vector weight driver ===
    private double vectorWeightX = 0;
    private double vectorWeightY = 0;

    public enum DriveState {
        DRIVE,
        STOPPING,
        LOCKED
    }

    public DriveState drivetrainState = DriveState.DRIVE;
    private ElapsedTime stopTimer = new ElapsedTime();
    private boolean isAligningActive = false;
    private double autoAlignTurn = 0;
    private ElapsedTime lockTimer = new ElapsedTime();

    // External control for lock
    private boolean lockEnabled = true;
    private boolean userWantsToMove = false;

    // Pedro Pathing override - when true, don't control motors (Pedro has control)
    private boolean pedroOverride = false;

    // ==================== INPUT CURVE ====================

    /**
     * Apply an input curve to joystick values for smoother response.
     * Uses a power curve: sign(x) * |x|^exp
     */
    private double applyInputCurve(double raw) {
        if (Math.abs(raw) < robot.inputDeadzone) return 0;
        double sign = raw > 0 ? 1.0 : -1.0;
        double abs = Math.abs(raw);
        return sign * Math.pow(abs, robot.inputCurveExponent);
    }

    // ==================== SHOOT LOCK ====================

    /**
     * When shootLock is true, forces DRIVE_STATE = LOCKED and neglects all gamepad input.
     */
    public void setShootLock(boolean locked) {
        this.shootLock = locked;
        if (locked) {
            // Engage lock immediately
            engageLock();
        }
    }

    public boolean isShootLockActive() {
        return shootLock;
    }

    // ==================== VECTOR WEIGHT DRIVER ====================

    /**
     * Add a velocity vector to be blended with joystick input.
     * Used for automatic launch-zone approach.
     */
    public void setVectorWeightDriver(double vx, double vy) {
        this.vectorWeightX = vx;
        this.vectorWeightY = vy;
    }

    // ==================== ALIGNMENT ====================

    public void setAligningActive(boolean aligning) {
        this.isAligningActive = aligning;
        if (aligning) {
            // During alignment, always DRIVE
            if (drivetrainState == DriveState.LOCKED) {
                disengageLock();
            }
            drivetrainState = DriveState.DRIVE;
        }
    }

    public void setAutoAlignTurn(double turn) {
        this.autoAlignTurn = turn;
    }

    // ==================== LOCK CONTROL ====================

    public void setLockEnabled(boolean enabled) {
        this.lockEnabled = enabled;
    }

    public void setUserWantsToMove(boolean wantsToMove) {
        this.userWantsToMove = wantsToMove;
    }

    /**
     * When true, MecanumDriveExt will not control the motors,
     * allowing an external controller (like Pedro Pathing) to take over.
     */
    public void setPedroOverride(boolean override) {
        this.pedroOverride = override;
    }

    public boolean isPedroOverrideActive() {
        return pedroOverride;
    }

    public void init(RobotHardware robot) {
        this.robot = robot;
    }

    private boolean shouldEngageLock() {
        if (robot.currentState == RobotHardware.RobotState.ALIGNING) {
            return false;
        }
        return lockEnabled && !userWantsToMove;
    }

    private void engageLock() {
        if (drivetrainState != DriveState.LOCKED) {
            robot.setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, robot.FL, robot.FR, robot.BL, robot.BR);
            robot.setMotorTarget(0, robot.FL, robot.FR, robot.BL, robot.BR);
            robot.setMotorMode(DcMotor.RunMode.RUN_TO_POSITION, robot.FL, robot.FR, robot.BL, robot.BR);
            robot.setMotorPower(0.3, robot.FL, robot.FR, robot.BL, robot.BR);
            lockTimer.reset();
            drivetrainState = DriveState.LOCKED;
        }
    }

    private void disengageLock() {
        robot.setMotorPower(0, robot.FL, robot.FR, robot.BL, robot.BR);
        robot.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER, robot.FL, robot.FR, robot.BL, robot.BR);
        drivetrainState = DriveState.DRIVE;
    }

    // ==================== MAIN DRIVE ====================

    public void drive(Gamepad gamepad1, boolean imuResetButton) {
        // If Pedro has override, let it control the motors
        if (pedroOverride) {
            return;
        }

        // In shoot lock state, all gamepad inputs are neglected
        if (shootLock) {
            if (drivetrainState != DriveState.LOCKED) {
                engageLock();
            }
            // Still apply lock power ramp so the robot holds
            applyLockPower();
            return;
        }

        double rawY = -gamepad1.left_stick_y;
        double rawX = gamepad1.left_stick_x;
        double rawRx = gamepad1.right_stick_x;

        // Apply input curve to all joystick values
        double y = applyInputCurve(rawY);
        double x = applyInputCurve(rawX);
        double rx = applyInputCurve(rawRx);

        // Blend vector weight driver if set
        if (Math.abs(vectorWeightX) > 0.001 || Math.abs(vectorWeightY) > 0.001) {
            x = clamp(x + vectorWeightX, -1.0, 1.0);
            y = clamp(y + vectorWeightY, -1.0, 1.0);
        }

        boolean movementInput = Math.abs(x) > robot.inputDeadzone ||
                                Math.abs(y) > robot.inputDeadzone ||
                                (Math.abs(rx) > robot.inputDeadzone && !isAligningActive);

        switch (drivetrainState) {
            case DRIVE:
                if (!movementInput) {
                    if (shouldEngageLock()) {
                        robot.setMotorPower(0, robot.FL, robot.FR, robot.BL, robot.BR);
                        stopTimer.reset();
                        drivetrainState = DriveState.STOPPING;
                    }
                } else {
                    if (imuResetButton) {
                        robot.resetIMU();
                    }

                    double botHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                    double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                    double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

                    rotX = rotX * robot.driveSpeedMultiplier;
                    rotY = rotY * robot.driveSpeedMultiplier;

                    double finalRx = isAligningActive ? autoAlignTurn : rx;

                    double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(finalRx), 1);
                    double frontLeftPower = (rotY + rotX + finalRx) / denominator;
                    double backLeftPower = (rotY - rotX + finalRx) / denominator;
                    double frontRightPower = (rotY - rotX - finalRx) / denominator;
                    double backRightPower = (rotY + rotX - finalRx) / denominator;

                    robot.FL.setPower(frontLeftPower);
                    robot.BL.setPower(backLeftPower);
                    robot.FR.setPower(frontRightPower);
                    robot.BR.setPower(backRightPower);
                }
                break;

            case STOPPING:
                if (!shouldEngageLock()) {
                    if (movementInput) {
                        robot.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER, robot.FL, robot.FR, robot.BL, robot.BR);
                        drivetrainState = DriveState.DRIVE;
                    }
                } else if (stopTimer.milliseconds() >= 200) {
                    engageLock();
                } else if (movementInput) {
                    robot.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER, robot.FL, robot.FR, robot.BL, robot.BR);
                    drivetrainState = DriveState.DRIVE;
                }
                break;

            case LOCKED:
                applyLockPower();
                if (movementInput || !shouldEngageLock()) {
                    disengageLock();
                }
                break;
        }
    }

    private void applyLockPower() {
        double lockPower;
        if (lockTimer.milliseconds() >= 1000) {
            lockPower = 0.9;
        } else {
            lockPower = 0.3 + (0.55 * (lockTimer.milliseconds() / 1000.0));
        }
        robot.setMotorPower(lockPower, robot.FL, robot.FR, robot.BL, robot.BR);
    }

    public void resetIMU() {
        robot.resetIMU();
    }

    public DriveState getDriveState() {
        return drivetrainState;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
