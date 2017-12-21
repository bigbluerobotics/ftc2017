package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Big Blue Robotics on 10/4/2017.
 */

public class ClawControls {
    private double leftOpen = 0.185;
    private double leftClosed = 0.65;
    private double leftHalf = 0.5;

    private double leftTopOpen = 0.185;
    private double leftTopClosed = 0.65;
    private double leftTopHalf = 0.5;

    private double rightOpen = 0.95;
    private double rightClosed = 0.1;
    private double rightHalf = 0.25;

    private double rightTopOpen = 0.8;
    private double rightTopClosed = 0.2;
    private double rightTopHalf = 0.35;



    private Servo leftClaw;
    private Servo leftTopClaw;
    private Servo rightClaw;
    private Servo rightTopClaw;

    public ClawControls(HardwareMap hardwareMap){
        leftClaw = hardwareMap.get(Servo.class, "servo_left");
        leftTopClaw = hardwareMap.get(Servo.class, "servo_top_left");
        rightClaw = hardwareMap.get(Servo.class, "servo_right");
        rightTopClaw = hardwareMap.get(Servo.class, "servo_top_right");
    }

    public void close(){
        leftClaw.setPosition(leftClosed);
        leftTopClaw.setPosition(leftTopClosed);
        rightClaw.setPosition(rightClosed);
        rightTopClaw.setPosition(rightTopClosed);
    }

    public void open(){
        leftClaw.setPosition(leftOpen);
        leftTopClaw.setPosition(leftTopOpen);
        rightClaw.setPosition(rightOpen);
        rightTopClaw.setPosition(rightTopOpen);
    }

    public void halfOpen(){
        leftClaw.setPosition(leftHalf);
        leftTopClaw.setPosition(leftTopHalf);
        rightClaw.setPosition(rightHalf);
        rightTopClaw.setPosition(rightTopHalf);
    }

}
