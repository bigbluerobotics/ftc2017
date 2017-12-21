package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Big Blue Robotics on 10/6/2017.
 */

@Autonomous(name="Autonomous Back Blue")
public class PingryAutonomousBackBlue extends PingryAutonomousBack {
    public PingryAutonomousBackBlue(){
        super();
        this.allianceColor = PingryAutonomousBack.BLUE;
    }
}
