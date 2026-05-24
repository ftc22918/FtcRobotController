package org.firstinspires.ftc.teamcode.newrobot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

public class Initiate_Hardware {

    DcMotorEx initMotor(
            HardwareMap hwMap,
            String motorHardwareName,
            String motorDirection,
            String breakBehavior,
            boolean useEncoder
    ) {

        if (!"FORWARD".equalsIgnoreCase(motorDirection) && !"REVERSE".equalsIgnoreCase(motorDirection)) {
            throw new IllegalArgumentException("Only 'FORWARD' or 'REVERSE' are allowed for motor direction.");
        }
        DcMotorEx motor;
        motor = hwMap.get(DcMotorEx.class, motorHardwareName);
        motor.setPower(0);

//        Default Direction is FORWARD
        if (motorDirection.equalsIgnoreCase("REVERSE")) {
            motor.setDirection(DcMotorEx.Direction.REVERSE);
        }

//        Default ZeroPowerBehavior is FLOAT
        if (breakBehavior.equalsIgnoreCase("Break")) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

//        If useEncoder is set to true
        if (useEncoder) {
            motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }

        return motor;
    }

    DcMotorEx initTurretMotor(
            HardwareMap hwMap,
            String motorHardwareName,
            double kP,
            double kI,
            double kD,
            double F,
            double position
    ) {
        DcMotorEx motor;
        motor = hwMap.get(DcMotorEx.class, motorHardwareName);
        motor.setPower(0);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motor.setVelocityPIDFCoefficients(kP, kI, kD, F);
        motor.setPositionPIDFCoefficients(position);
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        return motor;
    }

//    ServoControllerEx initServo(HardwareMap hwMap, String motorHardwareName, String motorDirection) {
//
//    }
}
