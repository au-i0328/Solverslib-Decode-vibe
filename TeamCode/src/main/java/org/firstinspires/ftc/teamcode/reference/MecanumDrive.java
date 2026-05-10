package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Mecanum Drive utility class - contains all drive logic
 *
 * LOCK BEHAVIOR:
 * - Lock is controlled by setLockEnabled() - when true, lock engages after no input
 * - In ALIGNING state: lock is ALWAYS disabled (free movement during alignment)
 * - In all other states: lock enabled based on setLockEnabled()
 * - Lock disengages when userWantsToMove is true
 */
public class MecanumDrive {
    
    // Reference to shared hardware
    private RobotHardware robot;
    
    // State
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
    
    /**
     * Set the auto-align turn power
     * This is added to the drive stick input during auto-alignment
     * @param turn Turn power from -1 to 1
     */
    public void setAutoAlignTurn(double turn) {
        this.autoAlignTurn = turn;
    }
    
    /**
     * Set whether alignment mode is active
     * When aligning:
     * - Lock is DISENGAGED (allows robot to move freely)
     * - Auto-align turn is applied (keeps rotation locked)
     * @param aligning true if alignment mode is active
     */
    public void setAligningActive(boolean aligning) {
        this.isAligningActive = aligning;
    }
    
    /**
     * Set whether lock is enabled for this state
     * Note: Lock is ALWAYS disabled in ALIGNING state regardless of this setting
     * @param enabled true if lock should be enabled
     */
    public void setLockEnabled(boolean enabled) {
        this.lockEnabled = enabled;
    }
    
    /**
     * Set whether the user wants to move (joystick input)
     * When true, lock will disengage
     * @param wantsToMove true if joystick has input
     */
    public void setUserWantsToMove(boolean wantsToMove) {
        this.userWantsToMove = wantsToMove;
    }
    
    /**
     * Initialize the mecanum drive
     * @param robot The RobotHardware instance
     */
    public void init(RobotHardware robot) {
        this.robot = robot;
    }
    
    /**
     * Determine if lock should actually engage
     * Lock is engaged when:
     * 1. lockEnabled is true
     * 2. NOT in ALIGNING state (always disabled during alignment)
     * 3. NOT in SHOOT state (handled by state machine externally)
     * 4. userWantsToMove is false (no joystick input)
     */
    private boolean shouldEngageLock() {
        // Always disabled in ALIGNING state
        if (robot.currentState == RobotHardware.RobotState.ALIGNING) {
            return false;
        }
        
        // Otherwise, depends on lockEnabled and user input
        return lockEnabled && !userWantsToMove;
    }

    /**
     * Main drive method.
     * Handles both normal driving and auto-align mode
     * 
     * In auto-align mode:
     * - Robot rotation is locked to AprilTag
     * - Robot can still translate (strafe/forward/back)
     * - Lock is controlled by shouldEngageLock()
     * 
     * @param gamepad1 The first gamepad
     * @param imuResetButton Whether to check for IMU reset
     */
    public void drive(com.qualcomm.robotcore.hardware.Gamepad gamepad1, boolean imuResetButton) {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        // Check if user wants to move
        // In align mode: ignore rx (rotation is auto-controlled)
        // In normal mode: all inputs count
        boolean movementInput = Math.abs(x) > robot.inputDeadzone || 
                              Math.abs(y) > robot.inputDeadzone || 
                              (Math.abs(rx) > robot.inputDeadzone && !isAligningActive);

        switch (drivetrainState) {
            case DRIVE:
                if (!movementInput) {
                    // No joystick input
                    // Check if we should engage lock
                    if (shouldEngageLock()) {
                        // Lock is enabled - enter STOPPING to wait for lock
                        robot.setMotorPower(0, robot.FL, robot.FR, robot.BL, robot.BR);
                        stopTimer.reset();
                        drivetrainState = DriveState.STOPPING;
                    }
                    // If lock NOT enabled, just keep motors at 0 (BRAKE handles staying still)
                } else {
                    // User is moving - always drive
                    if (imuResetButton) {
                        robot.resetIMU();
                    }

                    double botHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                    // Rotate the movement direction counter to the bot's rotation
                    double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                    double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

                    rotX = rotX * robot.driveSpeedMultiplier;  // Counteract imperfect strafing

                    // Use autoAlignTurn if aligning, otherwise use gamepad rx
                    double finalRx = isAligningActive ? autoAlignTurn : rx;

                    // Denominator is the largest motor power (absolute value) or 1
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
                // Check if we should engage lock
                if (!shouldEngageLock()) {
                    // Lock disabled - stay stopped but don't lock
                    if (movementInput) {
                        robot.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER, robot.FL, robot.FR, robot.BL, robot.BR);
                        drivetrainState = DriveState.DRIVE;
                    }
                    // Otherwise just stay in STOPPING with motors at 0
                } else if (stopTimer.milliseconds() >= 200) {
                    // LOCK SEQUENCE:
                    // Step 1: Reset encoders to 0 (current position becomes the zero reference)
                    robot.setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER, robot.FL, robot.FR, robot.BL, robot.BR);
                    // Step 2: Set target position to 0 (hold where it is)
                    robot.setMotorTarget(0, robot.FL, robot.FR, robot.BL, robot.BR);
                    // Step 3: Enable RUN_TO_POSITION mode
                    robot.setMotorMode(DcMotor.RunMode.RUN_TO_POSITION, robot.FL, robot.FR, robot.BL, robot.BR);
                    // Step 4: Start with low power, will ramp up
                    robot.setMotorPower(0.3, robot.FL, robot.FR, robot.BL, robot.BR);
                    // Step 5: Start the ramp timer
                    lockTimer.reset();

                    drivetrainState = DriveState.LOCKED;
                } else if (movementInput) {
                    // Interrupt the stop if driver touches sticks
                    robot.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER, robot.FL, robot.FR, robot.BL, robot.BR);
                    drivetrainState = DriveState.DRIVE;
                }
                break;

            case LOCKED:
                // Power ramp: 0.3 at start → 0.85 over 1 second
                double lockPower;
                if (lockTimer.milliseconds() >= 1000) {
                    lockPower = 0.85;
                } else {
                    // Linear interpolation from 0.3 to 0.85
                    lockPower = 0.3 + (0.55 * (lockTimer.milliseconds() / 1000.0));
                }
                robot.setMotorPower(lockPower, robot.FL, robot.FR, robot.BL, robot.BR);

                // Any joystick input OR lock becomes disabled → disengage
                if (movementInput || !shouldEngageLock()) {
                    // Return to driving mode immediately
                    robot.setMotorPower(0, robot.FL, robot.FR, robot.BL, robot.BR);
                    robot.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER, robot.FL, robot.FR, robot.BL, robot.BR);
                    drivetrainState = DriveState.DRIVE;
                }
                break;
        }
    }
    
    /**
     * Reset the IMU yaw
     */
    public void resetIMU() {
        robot.resetIMU();
    }
    
    /**
     * Get current drive state for telemetry
     */
    public DriveState getDriveState() {
        return drivetrainState;
    }
}
