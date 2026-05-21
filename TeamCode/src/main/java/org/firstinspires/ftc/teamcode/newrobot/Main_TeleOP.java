package org.firstinspires.ftc.teamcode.newrobot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Control Hub Hardware Profile:
 *   Hardware Variables:
 *     DcMotor:  motor_right_rear, port 0, GoBILDA Series 5203
 *     DcMotor:  motor_right_forward, port 1, GoBILDA Series 5203
 *     DcMotor:  motor_left_forward, port 2, GoBILDA Series 5203
 *     DcMotor:  motor_left_rear, port 3, GoBILDA Series 5203
 *     DcMotor:  motor_intake,
 *     DcMotor:  motor_turret,
 *     DcMotor:  motor_front_shooter,
 *     DcMotor:  motor_top_shooter,
 *     Servo:    servo_flipper,
 * Syntax types:
 *   Class - Pascal Case:     ThisIsPascalCase
 *   Functions - Camel Case:  thisIsCamelCase
 *   Variables - Camel Case:  thisIsCamelCase
 *   Hardware - Snake Case:   this_is_snake_case
 *   Constants - Upper Snake Case:  THIS_IS_UPPER_CASE
 */

@TeleOp(name="_Main")
public class Main_TeleOP extends LinearOpMode {

    double motorSensitivity = 1.0;
    double initMotorPower = 0;


    DcMotor motorLeftForward;
    DcMotor motorLeftRear;
    DcMotor motorRightForward;
    DcMotor motorRightRear;



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
        initMotorLeftForward();
        initMotorLeftRear();
        initMotorRightForward();
        initMotorRightRear();
    }

    private void initMotorLeftForward() {
        motorLeftForward = hardwareMap.get(DcMotor.class, "motor_left_forward");
        motorLeftForward.setPower(initMotorPower);
        motorLeftForward.setDirection(DcMotor.Direction.FORWARD);
        motorLeftForward.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLeftForward.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftForward.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void initMotorLeftRear() {
        motorLeftRear = hardwareMap.get(DcMotor.class, "motor_left_rear");
        motorLeftRear.setPower(initMotorPower);
        motorLeftRear.setDirection(DcMotor.Direction.FORWARD);
        motorLeftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void initMotorRightForward() {
        motorRightForward = hardwareMap.get(DcMotor.class, "motor_right_forward");
        motorRightForward.setPower(initMotorPower);
        motorRightForward.setDirection(DcMotor.Direction.FORWARD);
        motorRightForward.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRightForward.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightForward.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void initMotorRightRear() {
        motorRightRear = hardwareMap.get(DcMotor.class, "motor_right_rear");
        motorRightRear.setPower(initMotorPower);
        motorRightRear.setDirection(DcMotor.Direction.FORWARD);
        motorRightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void teleOpControls() {
        motorLeftForward.setPower(gamepad1.left_stick_x * motorSensitivity);
        motorLeftRear.setPower(gamepad1.left_stick_x * motorSensitivity);
        motorRightForward.setPower(gamepad1.left_stick_x * motorSensitivity);
        motorRightRear.setPower(gamepad1.left_stick_x * motorSensitivity);
    }

    private void opModeTelemetry() {
        telemetry.addData("mlf power: ", "Encoder: %2d, Power: %.2f", motorLeftForward.getPower());
        telemetry.addData("mlf controller: ", motorLeftForward.getController());
        telemetry.addData("mlf port#: ", motorLeftForward.getPortNumber());
        telemetry.addData("mlf connection info: ", motorLeftForward.getConnectionInfo());
        telemetry.addData("mlf device name: ", motorLeftForward.getDeviceName());
        telemetry.addData("mlf manufacture: ", motorLeftForward.getManufacturer());
        telemetry.addData("mlf version: ", motorLeftForward.getVersion());
        telemetry.addData("mlf class: ", motorLeftForward.getClass());
        telemetry.addData("mlf mode: ", motorLeftForward.getMode());
        telemetry.update();
    }
}
