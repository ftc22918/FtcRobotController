package org.firstinspires.ftc.teamcode.newrobot;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveTrain {

//    DcMotorEx initMotor(HardwareMap hwMap, String motorHardwareName, String motorDirection) {
//        if (!"FORWARD".equalsIgnoreCase(motorDirection) && !"REVERSE".equalsIgnoreCase(motorDirection)) {
//            throw new IllegalArgumentException("Only 'FORWARD' or 'REVERSE' are allowed for motor direction.");
//        }
//        DcMotorEx motor;
//        motor = hwMap.get(DcMotorEx.class, motorHardwareName);
//        motor.setPower(0);
//        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        if (motorDirection.equalsIgnoreCase("REVERSE")) {
//            motor.setDirection(DcMotorEx.Direction.REVERSE);
//        }
//        return motor;
//    }

    public void drive (DcMotorEx motorRightForward, DcMotorEx motorRightRear, DcMotorEx motorLeftForward, DcMotorEx motorLeftRear, double forward, double strafe, double rotate, boolean inDemoMode) {
        double powerRightFront = forward + strafe + rotate;
        double powerRightBack = forward - strafe + rotate;
        double powerLeftFront = forward - strafe - rotate;
        double powerLeftBack = forward + strafe - rotate;

        double maxPower;
        double maxSpeed;

        if (!inDemoMode) {
            maxPower = 0.8;
            maxSpeed = 0.8;
        } else {
            maxPower = 0.3;
            maxSpeed = 0.3;

        }

        maxPower = Math.max(maxPower, Math.abs(powerLeftFront));
        maxPower = Math.max(maxPower, Math.abs(powerLeftBack));
        maxPower = Math.max(maxPower, Math.abs(powerRightFront));
        maxPower = Math.max(maxPower, Math.abs(powerRightBack));

        motorLeftForward.setPower(maxSpeed * (powerLeftFront / maxPower));
        motorLeftRear.setPower(maxSpeed * (powerLeftBack / maxPower));
        motorRightForward.setPower(maxSpeed * (powerRightFront / maxPower));
        motorRightRear.setPower(maxSpeed * (powerRightBack / maxPower));
    }
}
