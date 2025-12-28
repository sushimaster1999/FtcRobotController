package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "SampleAuto: Shoot 3 and Move", group = "Blue")
public class SampleAuto extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor elevator = null;
    private DcMotor intake = null;
    private CRServo indexerL = null;
    private CRServo indexerR = null;
    private DcMotorEx launcher = null;

    private final ElapsedTime runtime = new ElapsedTime();

    // Encoder and motor related constants
    static final double DRIVE_COUNTS_PER_MOTOR_REV = 384.5; // GoBilda 5203-2402-0014 (435 RPM)
    static final double LAUNCHER_COUNTS_PER_MOTOR_REV = 28;    // GoBilda 5202 Series (6000 RPM)

    static final double DRIVE_GEAR_REDUCTION = 1.0; 
    static final double WHEEL_DIAMETER_INCHES = 4.09449; // 104mm
    static final double COUNTS_PER_INCH = (DRIVE_COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    
    static final double ROBOT_TRACK_WIDTH_INCHES = 16.375; 

    static final double LAUNCHER_RPM = 2250;
    // Ticks per second = (RPM / 60) * Ticks per Revolution
    static final double LAUNCHER_VELOCITY_TICKS_PER_SEC = (LAUNCHER_RPM / 60) * LAUNCHER_COUNTS_PER_MOTOR_REV;
    static final double INTAKE_SPEED = 1.0;
    static final double ELEVATOR_SPEED = 0.5;
    static final double INDEXER_SPEED = 1.0;
    static final double DriveMotorSpeed = 0.4;
    static final double TURN_SPEED = 0.3;


    @Override
    public void runOpMode() {

        // Initialize the drive system variables.

        //Hardware Map:
        frontLeftDrive = hardwareMap.get(DcMotor.class, "FL");
        frontRightDrive = hardwareMap.get(DcMotor.class, "FR");
        backLeftDrive = hardwareMap.get(DcMotor.class, "BL");
        backRightDrive = hardwareMap.get(DcMotor.class, "BR");
        elevator = hardwareMap.get(DcMotor.class, "elevator");
        intake = hardwareMap.get(DcMotor.class, "intake");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        indexerL = hardwareMap.get(CRServo.class, "indexerL");
        indexerR = hardwareMap.get(CRServo.class, "indexerR");


        //Set motor encoder states
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set motor Direction
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        launcher.setDirection(DcMotor.Direction.FORWARD);
        indexerL.setDirection(CRServo.Direction.FORWARD);
        indexerR.setDirection(CRServo.Direction.REVERSE);

        //Set Zero Power Behavior
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //stop and Reset encoders
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        waitForStart();

        // --- AUTONOMOUS SEQUENCE ---

        //activate flywheels
        launcher.setVelocity(LAUNCHER_VELOCITY_TICKS_PER_SEC);
        // Drive backwards 6 inches 0.2 speed
        encoderDrive(-0.2, -6, -6, 4.0);
        // Activate subsystems
        intake.setPower(INTAKE_SPEED);
        elevator.setPower(ELEVATOR_SPEED);
        indexerL.setPower(INDEXER_SPEED);
        indexerR.setPower(INDEXER_SPEED);


        // Wait for 4 seconds
        sleep(4000);

        // Stop the elevator and indexers
        elevator.setPower(0);
        indexerL.setPower(0);
        indexerR.setPower(0);

        // Go backwards 10 inches
        encoderDrive(-.2, -10, -10, 5.0);

        // Turn right 90 degrees
        encoderTurn(TURN_SPEED, 90, 4.0);

        // Go backwards 5 inches
        //encoderDrive(DriveMotorSpeed, -5, -5, 4.0);

        // Turn left 90 degrees
        //encoderTurn(TURN_SPEED, -90, 2.0);

        // Stop all subsystems
        intake.setPower(0);
        launcher.setVelocity(0);
        indexerL.setPower(0);
        indexerR.setPower(0);

        // ---------------------------

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000); // pause to display final telemetry message.
    }

    /**
     * Method to perform a relative move, based on encoder counts.
     * @param speed The speed to drive at, from 0.0 to 1.0
     * @param leftInches The distance for the left wheels to travel in inches.
     * @param rightInches The distance for the right wheels to travel in inches.
     * @param timeoutS The maximum time this step is allowed to take, in seconds.
     */
        //The timeout allows the robot to continue the autonomous even if stuck on something
    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
        int newFrontLeftTarget, newFrontRightTarget, newBackLeftTarget, newBackRightTarget;

        if (opModeIsActive()) {

            // Step 1: Determine new target position for each motor
            // Add the desired encoder counts to the current position to get the new target
            newFrontLeftTarget = frontLeftDrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newFrontRightTarget = frontRightDrive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            newBackLeftTarget = backLeftDrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newBackRightTarget = backRightDrive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);

            // Step 2: Set the target position for each motor controller.
            frontLeftDrive.setTargetPosition(newFrontLeftTarget);
            frontRightDrive.setTargetPosition(newFrontRightTarget);
            backLeftDrive.setTargetPosition(newBackLeftTarget);
            backRightDrive.setTargetPosition(newBackRightTarget);

            // Step 3: Turn On RUN_TO_POSITION mode.
            // This tells the motor to actively attempt to reach the target position.
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Step 4: Reset the timeout time and start motion.
            runtime.reset();
            frontLeftDrive.setPower(Math.abs(speed));
            frontRightDrive.setPower(Math.abs(speed));
            backLeftDrive.setPower(Math.abs(speed));
            backRightDrive.setPower(Math.abs(speed));

            // Step 5: Keep looping while we are still active, we haven't timed out, and the motors are busy.
            // The isBusy() method returns true while the motor is still trying to reach its target position.
            while (opModeIsActive() && (runtime.seconds() < timeoutS) &&
                   (frontLeftDrive.isBusy() || frontRightDrive.isBusy() || backLeftDrive.isBusy() || backRightDrive.isBusy())) {

                // Provide telemetry data for monitoring during the move.
                telemetry.addData("Running to", " %7d :%7d :%7d :%7d", newFrontLeftTarget, newFrontRightTarget, newBackLeftTarget, newBackRightTarget);
                telemetry.addData("Currently at", " at %7d :%7d :%7d :%7d",
                        frontLeftDrive.getCurrentPosition(), frontRightDrive.getCurrentPosition(), backLeftDrive.getCurrentPosition(), backRightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Step 6: Stop all motion once the loop is exited (target reached, timeout, or opmode stopped).
            frontLeftDrive.setPower(0); frontRightDrive.setPower(0); backLeftDrive.setPower(0); backRightDrive.setPower(0);

            // Step 7: Turn off RUN_TO_POSITION and switch back to RUN_USING_ENCODER.
            // This is important for consistency if you use other drive methods or switch to TeleOp.
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            
            //sleep(100); // optional pause after each move.
        }
    }
    
    /**
     * Method to perform a relative turn, based on encoder counts.
     * @param speed The speed to turn at, from 0.0 to 1.0
     * @param degrees The number of degrees to turn. Positive values turn right, negative values turn left.
     * @param timeoutS The maximum time this step is allowed to take, in seconds.
     */
    public void encoderTurn(double speed, double degrees, double timeoutS) {
        // Calculate the arc length for the turn
        double arcInches = (degrees / 360.0) * (ROBOT_TRACK_WIDTH_INCHES * Math.PI);
        
        double leftInches = -arcInches;
        double rightInches = arcInches;

        // Call encoderDrive with the calculated distances
        encoderDrive(speed, leftInches, rightInches, timeoutS);
    }
}
