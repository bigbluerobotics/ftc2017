package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Big Blue Robotics on 10/4/2017.
 */

public class ClawControls {
    private double leftOpen = 0.0;
    private double leftClosed = 0.6;
    private double rightOpen = 0.96;
    private double rightClosed = 0.36;

    private Servo leftClaw;
    private Servo rightClaw;

    public boolean isOpen = false;
    public ClawControls(HardwareMap hardwareMap){
        leftClaw = hardwareMap.get(Servo.class, "servo_left");
        rightClaw = hardwareMap.get(Servo.class, "servo_right");
    }

    public void close(){
        leftClaw.setPosition(leftClosed);
        rightClaw.setPosition(rightClosed);
        isOpen = false;
    }

    public void open(){
        leftClaw.setPosition(leftOpen);
        rightClaw.setPosition(rightOpen);
        isOpen = true;
    }

}
