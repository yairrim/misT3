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
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

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

@TeleOp(name="FuncTestKrakens", group="Krakens")
public class AutoFunctionTest extends LinearOpMode {

    HardwareKrakens robot = new HardwareKrakens();
    private ElapsedTime runtime = new ElapsedTime();
    public double ArmSpeed = 0.3;
    static final double HEADING_THRESHOLD = 1;
    static final double P_TURN_COEFF = 0.1;
    static final double P_DRIVE_COEFF = 0.15;
    @Override
    public void runOpMode() {

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
            runtime.reset();
            if(gamepad1.a){
                ArmUp(ArmSpeed);
            }
            if(gamepad1.b){
                ArmDown(ArmSpeed);
            }
            if(gamepad1.x){
                ArmDownFromStart(ArmSpeed);
            }
            if(gamepad1.dpad_left){
                MineralIn();
            }
            if(gamepad1.dpad_right){
                MineralOut();
            }
            if(gamepad1.left_stick_y>0) {
                gyroDrive(0.4, 80, 0);
            }
            if(gamepad1.left_stick_y<0) {
                gyroDrive(0.4, -80, 0);
            }
            if(gamepad1.left_stick_x>0){
                gyroTurn(0.4,90);
            }
            if(gamepad1.left_stick_x>0){
                gyroTurn(0.4,-90);
            }
            if (gamepad2.dpad_up){
                posC();
            }
            if (gamepad2.dpad_left){
                posL();
            }
            if (gamepad2.dpad_right){
                posR();
            }
        }
    }
    public void gyroDrive(double speed,
                          int distance,
                          double angle) {

        int newLeftTarget;
        int newRightTarget;
        int moveCounts;
        double max;
        double error;
        double steer;
        double leftSpeed = 0;
        double rightSpeed= 0;
        distance = distance * 25;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            robot.MotorRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.MotorLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            int startPosLeft=robot.MotorLeftFront.getCurrentPosition();
            newLeftTarget = startPosLeft + distance;
            newRightTarget = robot.MotorRightFront.getCurrentPosition() + distance;

            // Set Target and Turn On RUN_TO_POSITION
            robot.MotorLeftFront.setTargetPosition(newLeftTarget);
            robot.MotorRightFront.setTargetPosition(newRightTarget);

            robot.MotorRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.MotorLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.AllWheelsPower(speed, speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.MotorLeftFront.isBusy() &&
                            robot.MotorRightFront.isBusy())) {


                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);
                if(getGyroAngle()<angle-1){
                    leftSpeed=leftSpeed+0.1;
                    telemetry.addData("direction","left");

                }
                if(getGyroAngle()>angle+1){
                    rightSpeed=rightSpeed+0.1;
                    telemetry.addData("direction","right");
                }
                if(getGyroAngle()>angle-1 && getGyroAngle()<angle+1){
                    leftSpeed=speed;
                    rightSpeed=speed;
                    telemetry.addData("direction","forward");

                }
                telemetry.addData("gyroAngel",getGyroAngle());
                telemetry.addData("start position left ",startPosLeft);
                telemetry.addData("target left",newLeftTarget);
                telemetry.addData("correct position",robot.MotorLeftFront.getCurrentPosition());

                telemetry.update();

                robot.AllWheelsPower(leftSpeed, rightSpeed);


            }

            // Stop all motion;
            robot.AllWheelsPower(0, 0);

            // Turn off RUN_TO_POSITION
            robot.MotorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.MotorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Method to spin on central axis to point in a new direction.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the heading (angle)
     * 2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle Absolute Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn(double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && getGyroAngle()>angle-1 && getGyroAngle()<angle+1) {
            double leftSpeed=0;
            double rightSpeed=0;
            if(angle>0){
                leftSpeed=speed;
                rightSpeed=-speed;

            }
            if(angle<0){
                leftSpeed=-speed;
                rightSpeed=speed;

            }
            robot.AllWheelsPower(leftSpeed,rightSpeed);
            telemetry.update();
        }
        robot.AllWheelsPower(0,0);

    }

    /**
     * Method to obtain & hold a heading for a finite amount of time
     * Move will stop once the requested time has elapsed
     *
     * @param speed    Desired speed of turn.
     * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                 If a relative angle is required, add/subtract from current heading.
     * @param holdTime Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold(double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        robot.AllWheelsPower(0, 0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed  Desired speed of turn.
     * @param angle  Absolute Angle (in Degrees) relative to last gyro reset.
     *               0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *               If a relative angle is required, add/subtract from current heading.
     * @param PCoeff Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double error;
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        } else {
            steer = getSteer(error, PCoeff);
            rightSpeed = speed * steer;
            leftSpeed = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.AllWheelsPower(leftSpeed, rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);
        telemetry.addData(">", "Robot Heading = %f", getGyroAngle());

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     *
     * @param targetAngle Desired angle (relative to global reference established at last Gyro Reset).
     * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     * +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - getGyroAngle();
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     *
     * @param error  Error angle in robot relative degrees
     * @param PCoeff Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    public float getGyroAngle() {
        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return (AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
    }

    public void ArmDownFromStart(double speed) {
        robot.ArmUpDown2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.ArmUpDown2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int Target = robot.ArmUpDown2.getCurrentPosition() + 1700;
        robot.ArmUpDown2.setTargetPosition(Target);

        robot.ArmUpDown2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (robot.ArmUpDown2.isBusy()) {
            robot.ArmUpDown2.setPower(speed);
            robot.ArmUpDown.setPower(speed);
        }


    }

    public void ArmDown(double speed) {

        robot.ArmUpDown2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.ArmUpDown2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.ArmUpDown.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.ArmUpDown.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int Target2 = robot.ArmUpDown2.getCurrentPosition() + 770;
        int Target = robot.ArmUpDown.getCurrentPosition() + 770;

        robot.ArmUpDown2.setTargetPosition(Target2);
        robot.ArmUpDown.setTargetPosition(Target);

        robot.ArmUpDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.ArmUpDown2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (robot.ArmUpDown2.isBusy()) {
            robot.ArmUpDown2.setPower(speed);
            robot.ArmUpDown.setPower(speed);
        }
        robot.ArmUpDown2.setPower(0);
        robot.ArmUpDown.setPower(0);


    }

    public void ArmUp(double speed) {
        robot.ArmUpDown2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.ArmUpDown2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.ArmUpDown.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.ArmUpDown.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int Target2 = robot.ArmUpDown2.getCurrentPosition() - 770;
        int Target = robot.ArmUpDown.getCurrentPosition() - 770;

        robot.ArmUpDown2.setTargetPosition(Target2);
        robot.ArmUpDown.setTargetPosition(Target);

        robot.ArmUpDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.ArmUpDown2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (robot.ArmUpDown2.isBusy()&& robot.ArmUpDown.isBusy() &&opModeIsActive()) {
            robot.ArmUpDown2.setPower(speed);
            robot.ArmUpDown.setPower(speed);
        }
        robot.ArmUpDown2.setPower(0);
        robot.ArmUpDown.setPower(0);


    }

    public void ArmForWard(double speed) {
        robot.ArmUpDown2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.ArmUpDown2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int Target = robot.ArmUpDown2.getCurrentPosition() + 1487;
        robot.ArmUpDown2.setTargetPosition(Target);

        robot.ArmUpDown2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (robot.ArmUpDown2.isBusy()) {
            robot.ArmUpDown2.setPower(speed);
            robot.ArmUpDown.setPower(speed);
        }


    }

    public void ArmBackWard(double speed) {
        robot.ArmDistance.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.ArmDistance.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int Target = robot.ArmDistance.getCurrentPosition() - 1487;
        robot.ArmDistance.setTargetPosition(Target);

        robot.ArmDistance.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (robot.ArmDistance.isBusy()) {
            robot.ArmDistance.setPower(speed);
        }


    }

    public void MineralIn() {

        while (runtime.seconds() < 0.5) {
            telemetry.addData("runtime",runtime.seconds());
            telemetry.update();
            robot.c.setPower(1);
        }
        robot.c.setPower(0);
    }

    public void MineralOut() {

        while (runtime.seconds() < 0.5) {
            telemetry.addData("runtime",runtime.seconds());
            telemetry.update();
            robot.c.setPower(-1);
        }
        robot.c.setPower(0);
    }

    public void posL() {
        gyroTurn(0.5, -30);
        ArmDownFromStart(ArmSpeed);
        gyroDrive(0.5, 80, -30);
        gyroDrive(0.5, -80, -30);
        gyroTurn(0.5, 0);
        gyroDrive(0.5, 80, 0);

    }

    public void posR() {
        gyroTurn(0.5, 30);
        ArmDownFromStart(ArmSpeed);
        gyroDrive(0.5, 80, 30);
        gyroDrive(0.5, -80, 30);
        gyroTurn(0.5, 0);
        gyroDrive(0.5, 80, 0);

    }

    public void posC() {
        ArmDownFromStart(ArmSpeed);
        gyroDrive(0.5, 80, 0);
    }

}
