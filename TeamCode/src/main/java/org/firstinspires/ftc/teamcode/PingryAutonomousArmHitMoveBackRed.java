package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Big Blue Robotics on 10/6/2017.
 */

@Autonomous(name="Autonomous Arm Hit Back Red")
public class PingryAutonomousArmHitMoveBackRed extends PingryAutonomousArmHitMoveBack {
    public PingryAutonomousArmHitMoveBackRed(){
        super();
        this.allianceColor = PingryAutonomous.RED;
    }
}
