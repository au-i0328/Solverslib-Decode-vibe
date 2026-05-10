init state = init motor, servo, limelight, set gate servo to close
intake state = set gate servo to close, continously run intake power 1 (create command)
intakeReverse state = set gate servo to close, continuosly run intake power -1 (create command)
shoot state = run intake power 1, gate to position open, wait n amount of seconds, go to state Intake
aligning state = robot drivetrain auto align via servoing to limelight crosshairs, if apriltag not visible, use robot pose from odometry to turn drivetrain to the general direction of the goal. time on target needs to be larger than (alignment_delay) before switching to state = ALIGNED
aligned state = limelight crosshair aligns to apriltag, lock drivetrain

Auto Align Process (only heading, run to pose refer to optional features): Check if tag is detected in limelight, yes -> use limelight servoing to turn to correct heading; no -> use bot pose obtained by dead wheel to turn to the general direction of the goal, and then use limelight crosshair to align.

Use the following code for the MecanumDrive locking logic, add the ability to handle the absence of encoders gracefully and not affect anything: /Users/aurora/SolversLib-Quickstart/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/reference/MecanumDrive.java
An input curve needs to be applied any time for the MecanumDrive.


when state = intake or intakeReverse or init, follow original drivetrain lock logic

when state = ALIGNING, drivetrain state always = DRIVE

when state = ALIGNED, follow original drivetrain lock logic

when state = shoot, drivetrain state always = lock, ALL GAMEPAD INPUTS ARE NEGLECTED IN SHOOT STATE

on left and right trigger release, return to state = intake

right_bumper pressed -> state = intake
right trigger held -> state = ALIGNING -> ALIGNED
left trigger pressed -> if isReadytoShoot = true, enter state = shoot, timer 2 seconds (adjustable in RobotHardware.java as shootDelay variable) -> state = intake
if isReadytoShoot = false, do nothing and maintain current state.

isReadytoShoot is determined by: state = ALIGNED, Flywheel getVelocity within range, limelight result is valid, limelight has valid target, limelight distance value is reasonable, then true, else false

When isReadytoShoot = true, vibrate gamepad with 2 short bursts of 50ms

Robot Hardware should initialize the following: 
Motor FL, FR, BL, BR (FR and BR Direction Reverse), flywheelL (run with encoder), flywheelR (run with encoder), intake
Odom_pods para perpend
IMU imu
Servo hoodL hoodR gate
Limelight Camera limelight (public void init() {
    limelight = hardwareMap.get(Limelight3A.class, "limelight");
    limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
    limelight.start(); // This tells Limelight to start looking!
})

When the opmode is initatilized, allow gamepad1 to select alliance by pressing Triangle and Circle, Triangle = RED, Circle = BLUE.

Alliance = RED: limlight set pipeline 0, set goal coords to RED COORDS
Alliance = BLUE: limlight set pipeline 10, set goal coords to BLUE COORDS

RobotHardware.java should allow easy tuning of the following: RED AND BLUE goal coords, goal heights for limelight, PIDF coefficients of flywheelL and flywheelR, shoot delay (seconds), gate servo open position, close position, Flywheel constant target velocity, readyToShoot tolerance for flywheel velocity, limelight distance range, hood position to distance from goal interpolation table, hood position upper and lwoer hardstop, limelight mounting angle, limelight distance offset, flywheel velocity offset jump, hood angle offset jump, hood compensation coefficient, driveStallCurrentThreshold, aligntment_delay, vectorWeightDriver (optional feature)

Flywheel velocity for both motors are set to a constant. Run 2 seperate PIDF loops for the two motors. The motors spin for the entirety of the match at the target velocity. 


Master Controller Logic
    direction TB
    [*] --> INIT
    
    state INIT {
        direction LR
        Wait_For_Start --> Alliance_Selection
        Alliance_Selection --> RED: Press Triangle (Pipeline 0)
        Alliance_Selection --> BLUE: Press Circle (Pipeline 10)
    }
    
    INIT --> INTAKE : Play Pressed
    
    INTAKE --> INTAKE_REVERSE : Hold Left Bumper
    INTAKE_REVERSE --> INTAKE : Release Left Bumper
    
    INTAKE --> ALIGNING : Hold Right Trigger
    ALIGNING --> ALIGNED : Limelight Aligns to Tag
    
    ALIGNED --> SHOOT : Left Trigger [IF isReadyToShoot]
    ALIGNED --> ALIGNING : Target Lost / Needs Correction
    
    SHOOT --> INTAKE : 2-Second Timer Complete
    
    %% Trigger Release Logic
    ALIGNING --> INTAKE : Release Right Trigger
    ALIGNED --> INTAKE : Release Right Trigger
    
    %% Bumper Override
    ALIGNING --> INTAKE_REVERSE : Left Bumper Pressed
    ALIGNED --> INTAKE_REVERSE : Left Bumper Pressed
    SHOOT --> INTAKE_REVERSE : Left Bumper Pressed

    Drive Logic
    Joystick 1 --> Drive
    Joystick 2 --> Rotation
    Option --> Vibrate 200ms, Reset IMU
    
    Others
    Share --> set isReadyToShoot to true while pressed, neglecting other factors

 
    Once SHOOT Timer begins, gamepads are neglected, drivetrain is locked. When timer is over, state = INTAKE.

    LEFT_BUMPER is king, it should be respected before any other inputs.

    Gamepad2
    dpad_up: manually add (flywheel velocity offset jump) to flywheel target velocity
    dpad_down: manually subtract (flywheel velocity offset jump) to flywheel target velocity
    dpad_left: manually add (hood angle offset jump) to all hood positions with respect to hardstops
    dpad_right: manually subtract (hood angle offset jump) to all hood positions with respect to hardstops
    triangle: toggle between mecanum drive locking logic and normal generic field centric mecanum drive with no locking (constant state at DRIVE)
    circle: instantly returns to state = INIT, disregarding everything else


Loop Time

Enable Lynx Module Bulk Caching in your init() block.
Set it to LynxModule.BulkCachingMode.MANUAL.
Call clearBulkCache() exactly once at the top of your main while (opModeIsActive()) loop.

Hood
The hood angle is set at all times if a limelight distance reading is available. 

in State = SHOOT, General Hood Angle obtained by the interpolation table is locked, however some compensation for the drop in velocity needs to be applied.

Use a Proportional Compensation Factor ($kH$) for that purpose.
Calculate the difference between the target velocity and the velocity we actually have at that exact millisecond. Multiply that RPM drop by (hood compensation coefficient) to get our hood offset.

When the lookedup angle is exceeds either of the hardstops, make sure the servo is stopped at that hardstop value.

Power Management
When motor stalling in the drive motors are detected (not any other), reduce power to the individual drive motors until they are not stalling. This overrides anything.


Optional Features

Get robot pose from odom pods, when right trigger is held, check if robot is in launch zone, if not, run to closest calculated launch zone location while aligning via run to pose command. Draw a line with 5cm offset from the launch zone line as our own launch zone. 

Vector Addition: The robot calculates the velocity vector required to drive to the launch zone, and adds the driver's joystick input vectors on top of it. This allows the driver to "dodge" an opponent while the robot still tries to pull itself toward the line.