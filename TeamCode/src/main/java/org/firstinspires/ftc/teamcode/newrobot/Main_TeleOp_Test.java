package org.firstinspires.ftc.teamcode.newrobot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * Control Hub Hardware Profile:
 *   Hardware Variables:
 *     Control Hub
 *       DcMotor:  motor_right_rear, motor port 0, GoBILDA Series 5203
 *       DcMotor:  motor_right_forward, motor port 1, GoBILDA Series 5203
 *       DcMotor:  motor_left_forward, motor port 2, GoBILDA Series 5203
 *       DcMotor:  motor_left_rear, motor port 3, GoBILDA Series 5203
 *       Servo:    servo_rgb_light, servo port 0, GoBILDA RGB Indicator Light
 *       I2C:      pinpoint_odometry_computer, I2C port 2, GoBILDA Pinpoint Odometry Computer
 *     Expansion Hub
 *       DcMotor:  motor_turret, port, motor port 0, GoBILDA Series 5203
 *       DcMotor:  motor_artifact_intake, motor port 1, GoBILDA Series 5203
 *       DcMotor:  motor_main_flywheel, motor port 2, GoBILDA Series 5203
 *       DcMotor:  motor_aux_flywheel, motor port 3, GoBILDA Series 5203
 *       Servo:    servo_flipper, servo port 0, Axon MINI MK2
 * Notes:
 *      Lowest Max Velocity of drive motors:  1880
 * Syntax types:
 *   Class - Pascal Case:     ThisIsPascalCase
 *   Functions - Camel Case:  thisIsCamelCase
 *   Variables - Camel Case:  thisIsCamelCase
 *   Hardware - Snake Case:   this_is_snake_case
 *   Constants - Upper Snake Case:  THIS_IS_UPPER_CASE
 */

@SuppressWarnings({"FieldMayBeFinal","FieldCanBeLocal"})
@TeleOp(group="Test_Group")
public class Main_TeleOp_Test extends LinearOpMode {

//    Set to true to use demo mode for kids
    boolean inDemoMode = false;

//    Intake Variables
    private int artifactIntakeCount = 0;

    private double currentServoAngle = 0;
    private double servoFlipperStartingAngle = 0.93;
    private double servoFlipperEndingAngle = 0.7;

    private double currentTurretAngle = 0;
    private double resultsMaxVelocityTest = 1880;
    private double F = 32767.0 / resultsMaxVelocityTest;
    private double kP = F * 0.1; // Large increase of acceleration or deceleration
    private double kI = kP * 0.1; // Small increment of acceleration or deceleration
    private double kD = kP * 0.01; // Slows down the motor once the kP gets closer to it goal, minimizing overshoot
    private double position = 5.0; //
    private double ticksPerRev;

    // Turret encoder constants (adjust to your specific GoBilda motor gear ratio)
    private final int TICKS_PER_DEGREE = 5;
    private final int MAX_TURRET_TICKS = 1000; // Example limits
    private final int MIN_TURRET_TICKS = -1000;
    private int targetPosition = 0;


//    DriveTrain Hardware Variables
    private DcMotorEx motorRightForward;
    private DcMotorEx motorRightRear;
    private DcMotorEx motorLeftForward;
    private DcMotorEx motorLeftRear;

//    Shooter Hardware Variables
    private DcMotorEx motorTurret;
    private DcMotorEx motorArtifactIntake;
    private DcMotorEx motorMainFlywheel;
    private DcMotorEx motorAuxFlywheel;
    private Servo servoFlipper;


    boolean previousButtonAState = false;
    boolean previousButtonXState = false;
    boolean previousButtonYState = false;

    DriveTrain driveTrain = new DriveTrain();

    double forward, strafe, rotate;

    double distance, mainPower, auxPower;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        while (!isStarted()) {
            opModeTelemetry();
        }
        waitForStart();
        while (opModeIsActive()) {
            teleOpControls();
            opModeTelemetry();
        }
    }

    public void initHardware() {
//        Initiate drive train hardware
        Initiate_Hardware driveTrainHardware = new Initiate_Hardware();
        motorRightForward = driveTrainHardware.initMotor(hardwareMap,"motor_right_forward", "REVERSE", "BRAKE", true);
        motorRightRear = driveTrainHardware.initMotor(hardwareMap,"motor_right_rear", "REVERSE", "BRAKE", true);
        motorLeftForward = driveTrainHardware.initMotor(hardwareMap,"motor_left_forward", "FORWARD", "BRAKE", true);
        motorLeftRear = driveTrainHardware.initMotor(hardwareMap,"motor_left_rear", "FORWARD", "BRAKE", true);

//        Initiate shooter hardware
        Initiate_Hardware shooterHardware = new Initiate_Hardware();
        motorTurret = shooterHardware.initTurretMotor(hardwareMap,"motor_turret", kP, kI, kD, F, position);
        ticksPerRev = motorTurret.getMotorType().getTicksPerRev();
        motorArtifactIntake = shooterHardware.initMotor(hardwareMap,"motor_artifact_intake", "REVERSE", "FLOAT", false);
        motorMainFlywheel = shooterHardware.initMotor(hardwareMap,"motor_main_flywheel", "REVERSE", "FLOAT", true);
        motorAuxFlywheel = shooterHardware.initMotor(hardwareMap,"motor_aux_flywheel", "REVERSE", "FLOAT", true);
//        servoFlipper = shooterHardware.initServo(hardwareMap,"servo_flipper");
        servoFlipper = hardwareMap.get(Servo.class, "servo_flipper");
        servoFlipper.setPosition(servoFlipperStartingAngle);
    }

    private void teleOpControls() {
//        Drive Train controls
        forward = gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;
        driveTrain.drive(motorRightForward, motorRightRear, motorLeftForward, motorLeftRear, forward, strafe, rotate, inDemoMode);

//        Shooter controls
        artifactIntakeCount = 3; // Temp var to disable intake motor during testing
        if (artifactIntakeCount < 3) {
            motorArtifactIntake.setPower(1);
        } else {
            motorArtifactIntake.setPower(0);
        }

        boolean currentButtonAState = gamepad1.a;
        if (currentButtonAState && !previousButtonAState) {
            artifactIntakeCount++;
        }
        previousButtonAState = currentButtonAState;

//        if (gamepad1.right_bumper) {
//            artifactIntakeCount = 0;
//        }



        boolean currentButtonXState = gamepad1.x;
        if (currentButtonXState && !previousButtonXState) {
            servoFlipper.setPosition(servoFlipperStartingAngle);
        }
        previousButtonXState = currentButtonXState;

        boolean currentButtonYState = gamepad1.y;
        if (currentButtonYState && !previousButtonYState) {
            servoFlipper.setPosition(servoFlipperEndingAngle);
        }
        previousButtonYState = currentButtonYState;

        currentServoAngle = servoFlipper.getPosition();

//        currentTurretAngle = motorTurret.getCurrentPosition() / ticksPerRev; // normalizing ticket to revolutions


        if (gamepad1.right_bumper) {
            targetPosition += 5; // Rotate right
        } else if (gamepad1.left_bumper) {
            targetPosition -= 5; // Rotate left
        }

        // Limit the turret so it doesn't over-rotate and bind wires
        if (targetPosition > MAX_TURRET_TICKS) targetPosition = MAX_TURRET_TICKS;
        if (targetPosition < MIN_TURRET_TICKS) targetPosition = MIN_TURRET_TICKS;

        // --- 2. Apply Position ---
        motorTurret.setTargetPosition(targetPosition);
        motorTurret.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        // Set power. When running to position, power acts as a speed limit.
        // GoBilda motors geared for high speed may need a lower max power (e.g., 0.3) for precision.
        motorTurret.setPower(0.5);



    }

    public void setServoFlipper (double angle) {
        servoFlipper.setPosition(angle);
    }

    private void opModeTelemetry() {
        telemetry.addData("artifactIntakeCount", artifactIntakeCount);
        telemetry.addData("current servo angle", currentServoAngle);
        telemetry.addData("Target Ticks", targetPosition);
        telemetry.addData("Current Ticks", motorTurret.getCurrentPosition());
        telemetry.update();
    }
}
