package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Big Blue Robotics on 10/4/2017.
 */

public class RelicArm {
    private double open = 0.1;
    private double closed = 0.9;
    private double wristUp = 0.2;
    private double wristDown = 0.9;
    private double motorPower = 0.75;
    public boolean isHandOpen = false;
    public boolean isWristUp = false;

    private Servo hand;
    private Servo wrist;
    private DcMotor relicExtension;

    public RelicArm(HardwareMap hardwareMap){
        relicExtension = hardwareMap.get(DcMotor.class,"relicExtension");
        hand = hardwareMap.get(Servo.class,"clawHand");
        wrist = hardwareMap.get(Servo.class,"clawWrist");
    }
    public void push(double speed){
        relicExtension.setPower(motorPower*speed);
    }
    public void pull(double speed){
        relicExtension.setPower(-motorPower*speed);
    }
    public void stop(){
        relicExtension.setPower(0);
    }
    public void grab(){
        hand.setPosition(closed);
        isHandOpen = false;
    }
    public void ungrab(){
        hand.setPosition(open);
        isHandOpen = true;
    }
    public void wristUp(){
        wrist.setPosition(wristUp);
        isWristUp = true;
    }
    public void wristDown(){
        wrist.setPosition(wristDown);
        isWristUp = false;
    }

}
