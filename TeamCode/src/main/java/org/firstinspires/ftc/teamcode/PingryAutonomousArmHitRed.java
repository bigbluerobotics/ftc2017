package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Big Blue Robotics on 10/6/2017.
 */

@Autonomous(name="Autonomous Arm Hit Red")
public class PingryAutonomousArmHitRed extends PingryAutonomousArmHit {
    public PingryAutonomousArmHitRed(){
        super();
        this.allianceColor = PingryAutonomous.RED;
    }
}
