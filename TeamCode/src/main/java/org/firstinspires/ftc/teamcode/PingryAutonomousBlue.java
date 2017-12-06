package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Big Blue Robotics on 10/6/2017.
 */

@Autonomous(name="Autonomous Blue")
public class PingryAutonomousBlue extends PingryAutonomous {
    public PingryAutonomousBlue(){
        super();
        this.allianceColor = PingryAutonomous.BLUE;
    }
}
