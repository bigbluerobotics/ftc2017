/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Axis;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Driving")
public class Drive1 extends OpMode
{
    public NavxMicroNavigationSensor navx = null;

    public AHRS navx_device;
    public String startDate;
    public ElapsedTime runtime = new ElapsedTime();
    public boolean calibration_complete = false;
    private final int NAVX_DIM_I2C_PORT = 2;

    private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;

    // Declare OpMode members.
    private DcMotor winchDrive = null;
    private ClawControls clawControls = null;
    private MecanumDrive mecanumDrive = null;
    private RelicArm relicArm = null;
    private Servo jewelServo = null;
    private ColorSensor colorSensor = null;
    private int offsetAngle = 0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Starting...");
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        winchDrive = hardwareMap.get(DcMotor.class, "winch");
        relicArm = new RelicArm(hardwareMap);
        clawControls = new ClawControls(hardwareMap);
        mecanumDrive = new MecanumDrive(hardwareMap);


        winchDrive.setDirection(DcMotor.Direction.FORWARD);
        jewelServo = hardwareMap.get(Servo.class, "jewel_servo");
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        colorSensor = hardwareMap.get(ColorSensor.class, "color_sensor");
        clawControls.open();
        telemetry.addData("position", jewelServo.getPosition());
        telemetry.update();
    }


    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        //Fine motor controls
        if(gamepad2.dpad_up){
            relicArm.wristUp();
        }else if(gamepad2.dpad_down){
            relicArm.wristDown();
        }

        if(gamepad2.dpad_left){
            relicArm.grab();
        }else if(gamepad2.dpad_right){
            relicArm.ungrab();
        }

        if(gamepad2.a){
            clawControls.open();
        }else if(gamepad2.y){
            clawControls.close();
        }else if(gamepad2.b){
            clawControls.halfOpen();
        }

        /*if (gamepad2.y) {
            if (relicArm.isWristUp && !wristButtonPressed){
                relicArm.wristDown();
            }else if (!relicArm.isWristUp && !wristButtonPressed) {
                relicArm.wristUp();
            }
            wristButtonPressed = true;
        }else {
            wristButtonPressed = false;
        }
        if (gamepad2.x) {
            if (relicArm.isHandOpen && !handButtonPressed){
                relicArm.grab();
            }else if (!relicArm.isHandOpen && !handButtonPressed) {
                relicArm.ungrab();
            }
            handButtonPressed = true;
        }else {
            handButtonPressed = false;
        }
        if (gamepad2.a) {
            if (clawControls.isOpen && !glyphButtonPressed){
                clawControls.close();
            }else if (!clawControls.isOpen && !glyphButtonPressed) {
                clawControls.open();
            }
            glyphButtonPressed = true;
        }else {
            glyphButtonPressed = false;
        }*/

        if (gamepad2.left_trigger > 0){
            winchDrive.setPower(gamepad2.left_trigger);
        }else if (gamepad2.right_trigger > 0){
            winchDrive.setPower(-gamepad2.right_trigger);
        }else{
            winchDrive.setPower(0);
        }

        if (gamepad2.left_bumper)
            relicArm.push(1);
        else if (gamepad2.right_bumper)
            relicArm.pull(1);
        else
            relicArm.stop();

        if(gamepad1.left_bumper){
            jewelServo.setPosition(0.85);
        }
        else if(gamepad1.right_bumper){
            jewelServo.setPosition(0.1);
        }

        if(gamepad1.y) {
            offsetAngle = 0;
        }else if(gamepad1.x){
            offsetAngle = 90;
        }


        if(gamepad1.dpad_up || gamepad1.dpad_left || gamepad1.dpad_down ||gamepad1.dpad_right){
            double direction = (gamepad1.dpad_up?0:0) + (gamepad1.dpad_left?90:0) + (gamepad1.dpad_down?180:0) + (gamepad1.dpad_right?270:0);
            int mag = (gamepad1.dpad_up?1:0) + (gamepad1.dpad_left?1:0) + (gamepad1.dpad_down?1:0) + (gamepad1.dpad_right?1:0);

            if (mag != 0) direction /= mag;
            direction -= 135;
            direction += offsetAngle;
            mecanumDrive.polarMove(direction/360*2*Math.PI, 0, 0.35);
        } else {
            double speed = 0.6;
            if(gamepad1.right_trigger > 0.5){
                speed += (1-speed)*(2*(gamepad1.right_trigger - 0.5));
            }
            double magnitude = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
            telemetry.addData("robot angle", robotAngle);
            robotAngle += offsetAngle / 180.0 * Math.PI;
            double rightX = -gamepad1.right_stick_x;
            mecanumDrive.polarMove(robotAngle, rightX, speed * magnitude);
        }

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("red", colorSensor.red());
        telemetry.addData("green", colorSensor.green());
        telemetry.addData("blue", colorSensor.blue());
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Position", "9");
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }

}