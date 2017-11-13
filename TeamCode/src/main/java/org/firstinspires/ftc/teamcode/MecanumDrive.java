package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Big Blue Robotics on 10/4/2017.
 */

public class MecanumDrive {
    private DcMotor leftFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightFront = null;
    private DcMotor rightRear = null;

    public MecanumDrive(HardwareMap hardwareMap){
        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        leftRear = hardwareMap.get(DcMotor.class, "left_rear");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        rightRear = hardwareMap.get(DcMotor.class, "right_rear");

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
    }

    public void move(double leftFrontPower, double rightFrontPower, double leftRearPower, double rightRearPower){
        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftRear.setPower(leftRearPower);
        rightRear.setPower(rightRearPower);
    }

    public void polarMove(double angle, double turn, double power){
        final double v1 = power * Math.cos(angle) + turn;
        final double v2 = power * Math.sin(angle) - turn;
        final double v3 = power * Math.sin(angle) + turn;
        final double v4 = power * Math.cos(angle) - turn;

        leftFront.setPower(v1);
        rightFront.setPower(v2);
        leftRear.setPower(v3);
        rightRear.setPower(v4);
    }
}
