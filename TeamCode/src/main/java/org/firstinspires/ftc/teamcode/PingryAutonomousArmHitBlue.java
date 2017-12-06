package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Big Blue Robotics on 10/6/2017.
 */

@Autonomous(name="Autonomous Arm Hit Blue")
public class PingryAutonomousArmHitBlue extends PingryAutonomousArmHit {
    public PingryAutonomousArmHitBlue(){
        super();
        this.allianceColor = PingryAutonomous.BLUE;
    }
}
