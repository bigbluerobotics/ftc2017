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

@Autonomous(name="autonomous")

public class PingryAutonomous extends LinearOpMode
{
    //private final int NAVX_DIM_I2C_PORT = 0; /* See the installation page for details on port numbering. */
    public NavxMicroNavigationSensor navx = null;
    public ElapsedTime runtime = new ElapsedTime();
    public DcMotor winchDrive = null;
    public ClawControls clawControls = null;
    public MecanumDrive mecanumDrive = null;
    public RelicArm relicArm = null;
    public Servo jewelServo = null;
    public ColorSensor colorSensor = null;
    public boolean pressed = false;
    public boolean allianceColor;
    public static final boolean RED = true;
    public static final boolean BLUE = false;
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
        navx = hardwareMap.get(NavxMicroNavigationSensor.class, "navX");
        allianceColor = RED;

        /*Vuforia Code */

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AUx/Jtr/////AAAAGT/Wv/qzqE8Io1FwGTLz740qlNzPptr0US0IHzHFiciwzXK4addvdjXKjbRTPQjctOoOqR7ePutjttVjdpN723q6dVpCqV5te9sGoybNn78dC7TnzNbCXjCPqgTlWxpDsKx/Dy45z8xBKjKKMmTtNSCszpMVGl7ggM5RjDmzPU8vZxbhZAEHDeDdhk5jvxgLOYw208Y+vpUMRcSSU+57D8h6YKTFscxsJXz/xAwAwONxdI3wmySFbX7y/WjheD5bqkbNse6Plz6a1RoFcdHdzsW1W67auC8IQ41kC5LlhibT/61kKlBhCxQxEuLpbzO1UAASvysZsIpBBexP1ZRMqYQynJ7Qd4CG0dTNBwEArxHF";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        telemetry.addData(">", "Press Play to start");
        telemetry.update();

        waitForStart();
        relicTrackables.activate();
        detectPattern();
    }

    /**
     * Puts down arm, color senses the balls, spins to hit the correct ball off.
     * @throws InterruptedException
     */
    public void armHit() throws InterruptedException{
        colorSensor.enableLed(true);
        jewelServo.setPosition(0.2);
        twait(1500);
        boolean color = colorSensor.red() > colorSensor.blue();
        twait(250);
        if (color == allianceColor){
            mecanumDrive.move(0.6,-0.6,0.6,-0.6);
            twait(250);
            mecanumDrive.move(0,0,0,0);
            twait(100);
            mecanumDrive.move(-0.6,0.6,-0.6,0.6);
            twait(250);
            mecanumDrive.move(0,0,0,0);
        }else{
            mecanumDrive.move(-0.6,0.6,-0.6,0.6);
            twait(250);
            mecanumDrive.move(0,0,0,0);
            twait(100);
            mecanumDrive.move(0.6,-0.6,0.6,-0.6);
            twait(250);
            mecanumDrive.move(0,0,0,0);
        }
    }

    /**
     * uses Vuforia to detect the pattern, sets a class variable 'glyphPos' to l, r, or c depending on what it sees.
     */
    public void detectPattern() {
        RelicRecoveryVuMark vuMark = null;
        while (opModeIsActive()) {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                if (vuMark == RelicRecoveryVuMark.LEFT) {
                    glyphPos = 'l';
                    telemetry.addData("put", "left", vuMark);
                } else if (vuMark == RelicRecoveryVuMark.RIGHT){
                    glyphPos = 'r';
                    telemetry.addData("put", "right", vuMark);
                } else{
                    glyphPos = 'c';
                    telemetry.addData("put", "center", vuMark);
                }
                // POSE CODE BELOW \/ \/ \/

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */

                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
                telemetry.addData("Pose", format(pose));

                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */
                if (pose != null) {
                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    // Extract the X, Y, and Z components of the offset of the target relative to the robot
                    double tX = trans.get(0);
                    double tY = trans.get(1);
                    double tZ = trans.get(2);

                    // Extract the rotational components of the target relative to the robot
                    double rX = rot.firstAngle;
                    double rY = rot.secondAngle;
                    double rZ = rot.thirdAngle;

                }
                //break;
            } else {
                telemetry.addData("VuMark", "not visible");
            }
            telemetry.update();
        }
    }

    /**
     * Puts glyph into the tower based on what 'glyphPos' is.
     */
    public void putGlyph(){
        // navigate somehow? navx? vision?
        if (glyphPos == 'r'){ // put glyph on right

        }else if (glyphPos == 'l'){ // put glyph on left

        }else{ // put glyph in center

        }
        // put it in
    }
    public void twait(long millis) throws InterruptedException{
        Thread.sleep(millis);
    }
    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

}
