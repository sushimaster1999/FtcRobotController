package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Test: Motor Direction", group = "Test")
public class MotorDirectionTest extends LinearOpMode {

    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;

    @Override
    public void runOpMode() {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "FL");
        frontRightDrive = hardwareMap.get(DcMotor.class, "FR");
        backLeftDrive = hardwareMap.get(DcMotor.class, "BL");
        backRightDrive = hardwareMap.get(DcMotor.class, "BR");

        // Directions from SampleAuto
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        
        // We are not using encoders, so let's set them to run without.
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.addData(">", "Press Play to start test.");
        telemetry.addData(">", "Each motor will spin FORWARD for 2 seconds.");
        telemetry.update();

        waitForStart();

        // Test Front Left
        telemetry.clearAll();
        telemetry.addData("Testing", "Front Left Motor");
        telemetry.update();
        frontLeftDrive.setPower(0.5);
        sleep(2000);
        frontLeftDrive.setPower(0);

        sleep(1000); // Pause between tests

        // Test Front Right
        telemetry.clearAll();
        telemetry.addData("Testing", "Front Right Motor");
        telemetry.update();
        frontRightDrive.setPower(0.5);
        sleep(2000);
        frontRightDrive.setPower(0);
        
        sleep(1000); // Pause between tests

        // Test Back Left
        telemetry.clearAll();
        telemetry.addData("Testing", "Back Left Motor");
        telemetry.update();
        backLeftDrive.setPower(0.5);
        sleep(2000);
        backLeftDrive.setPower(0);
        
        sleep(1000); // Pause between tests

        // Test Back Right
        telemetry.clearAll();
        telemetry.addData("Testing", "Back Right Motor");
        telemetry.update();
        backRightDrive.setPower(0.5);
        sleep(2000);
        backRightDrive.setPower(0);

        telemetry.clearAll();
        telemetry.addData("Test", "Complete");
        telemetry.update();
    }
}
