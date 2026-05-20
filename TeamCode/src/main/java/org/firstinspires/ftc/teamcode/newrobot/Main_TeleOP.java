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

    DcMotor motorLeftForward;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        while (!isStarted()) {
            opModeTelemetry();
        }
        waitForStart();
        while (opModeIsActive()) {
            opModeTelemetry();
        }
    }
    public void initHardware() {
        initMotorLeftFront();
    }

    private void initMotorLeftFront() {
        motorLeftForward = hardwareMap.get(DcMotor.class, "motor_left_forward");
        motorLeftForward.setDirection(DcMotor.Direction.FORWARD);

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
        telemetry.update();
    }
}
