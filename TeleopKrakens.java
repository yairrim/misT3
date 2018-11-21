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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * This OpMode uses the common HardwareK9bot class to define the devices on the robot.
 * All device access is managed through the HardwareK9bot class. (See this class for device names)
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a basic Tank Drive Teleop for the K9 bot
 * It raises and lowers the arm using the Gampad Y and A buttons respectively.
 * It also opens and closes the claw slowly using the X and B buttons.
 *
 * Note: the configuration of the servos is such that
 * as the arm servo approaches 0, the arm position moves up (away from the floor).
 * Also, as the claw servo approaches 0, the claw opens up (drops the game element).
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Telop Krakens", group="Krakens")
public class TeleopKrakens extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareKrakens   robot           = new HardwareKrakens();              // Use a K9'shardware
    @Override
    public void runOpMode() {
        double LeftSpeed;
        double RightSpeed;
        boolean reverse =false;
        double cPower =0;
        double ePower;
        double udPower;
        double maxPower=0.5;
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */

        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Kraken get ready");    //
        telemetry.update();
        robot.MotorLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.MotorRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.ArmUpDown2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.ArmUpDown.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //setting the speed/power of the wheeels acording the y position in the left and right stick
            LeftSpeed = -gamepad1.left_stick_y;
            RightSpeed = -gamepad1.right_stick_y;

            cPower = gamepad2.left_trigger-gamepad2.right_trigger;
            ePower=gamepad2.left_stick_y;
            udPower=gamepad2.right_stick_y;
            if(gamepad1.dpad_down){
                reverse=true;
            }
            if(gamepad1.dpad_up){
                reverse=false;
            }
            if(reverse){
                telemetry.addData("mode","reverse");
                LeftSpeed=-LeftSpeed;
                RightSpeed=-RightSpeed;
            }
            else{
                telemetry.addData("mode","normal");
            }
            telemetry.addData("left",LeftSpeed);
            telemetry.addData("right",RightSpeed);
            telemetry.addData("left_trigger 2",gamepad2.left_trigger);
            telemetry.addData("right_trigger 2",gamepad2.right_trigger);
            telemetry.addData("lefy 2",gamepad2.left_stick_y);
            telemetry.addData("dpad_up 2",gamepad2.dpad_up);
            telemetry.addData("dpad_down 2",gamepad2.dpad_down);
            telemetry.addData("cPower",cPower);
            telemetry.update();
            if (gamepad2.dpad_up) {

                robot.ArmDistance.setPower(-0.7);
            }
            if (gamepad2.dpad_down) {

                robot.ArmDistance.setPower(0.7);
            }

            if(!gamepad2.dpad_down && !gamepad2.dpad_up) {

                robot.ArmDistance.setPower(0);
            }


            if(gamepad1.dpad_left){
                robot.IdDropper.setPosition(robot.IdDropper.getPosition()+1);
            }
            if(gamepad1.dpad_right) {
                robot.IdDropper.setPosition(robot.IdDropper.getPosition() - 1);
            }

            if(gamepad2.dpad_left){
                robot.blocker.setPosition(robot.IdDropper.getPosition()+1);
            }
            if(gamepad2.dpad_right) {
                robot.blocker.setPosition(robot.IdDropper.getPosition() - 1);
            }
            if(ePower<-0.3){
                ePower=-0.3;
            }
            if(ePower>0.3){
                ePower=0.3;
            }
            if(udPower<-0.3){
                udPower=-0.3;
            }
            if(udPower>0.3){
                udPower=0.3;
            }
            if(LeftSpeed<-maxPower){
                LeftSpeed=-0.5;
            }
            if(LeftSpeed>maxPower){
                LeftSpeed=0.5;
            }
            if(RightSpeed<-maxPower){
                RightSpeed=-0.5;
            }
            if(RightSpeed>maxPower){
                RightSpeed=0.5;
            }
            if(gamepad1.right_bumper){
                maxPower=maxPower+0.1;
            }
            if(gamepad1.left_bumper){
                maxPower=maxPower-0.1;
            }
            robot.c.setPower(cPower);
            robot.Elbow.setPower(ePower);
            robot.ArmUpDown.setPower(udPower);
            robot.ArmUpDown2.setPower(udPower);
            robot.AllWheelsPower(LeftSpeed,RightSpeed);
            robot.ArmMotor.setPower((gamepad2.right_stick_x));
            // Pause for 40 mS each cycle = update 25 times a second.
            sleep(40);
        }
    }
}
