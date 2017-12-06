package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Axis;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by Big Blue Robotics on 10/6/2017.
 */

@Autonomous(name="DriveStraight")

public class DriveStraight extends LinearOpMode
{
    //private final int NAVX_DIM_I2C_PORT = 0; /* See the installation page for details on port numbering. */
    public static final boolean RED = true;
    public static final boolean BLUE = false;
    //public NavxMicroNavigationSensor navx = null;
    public ElapsedTime runtime = new ElapsedTime();
    public DcMotor winchDrive = null;
    public ClawControls clawControls = null;
    public MecanumDrive mecanumDrive = null;
    public RelicArm relicArm = null;
    public Servo jewelServo = null;
    public ColorSensor colorSensor = null;
    public boolean pressed = false;
    public boolean allianceColor = BLUE;
    public VuforiaLocalizer vuforia;
    public VuforiaTrackable relicTemplate;
    public char glyphPos = 'c';
    public void runOpMode() throws InterruptedException{
        colorSensor = hardwareMap.get(ColorSensor.class, "color_sensor");

        telemetry.addData("Status", "Starting...");
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        winchDrive = hardwareMap.get(DcMotor.class, "winch");

        clawControls = new ClawControls(hardwareMap);
        mecanumDrive = new MecanumDrive(hardwareMap);

        winchDrive.setDirection(DcMotor.Direction.FORWARD);
        jewelServo = hardwareMap.get(Servo.class, "jewel_servo");
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        colorSensor = hardwareMap.get(ColorSensor.class, "color_sensor");
        //navx = hardwareMap.get(NavxMicroNavigationSensor.class, "navX");
        clawControls.open();
        jewelServo.setPosition(0);
        waitForStart();
        mecanumDrive.move(-0.4,-0.4,-0.4,-0.4);
        twait(4000);
        mecanumDrive.move(0,0,0,0);
        twait(1000);
        mecanumDrive.move(0.1,0.1,0.1,0.1);
        twait(100);
        mecanumDrive.move(0,0,0,0);

    }

    public void twait(long millis) throws InterruptedException{
        Thread.sleep(millis);
    }

}
