package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Big Blue Robotics on 10/6/2017.
 */

@Autonomous(name="Autonomous Back Red")
public class PingryAutonomousBackRed extends PingryAutonomousBack {
    public PingryAutonomousBackRed(){
        super();
        this.allianceColor = PingryAutonomousBack.RED;
    }
}
