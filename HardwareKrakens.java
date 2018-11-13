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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


public class HardwareKrakens
{
    /* Public OpMode members. */
    public DcMotor  MotorLeftFront   = null;
    public DcMotor  MotorRightFront  = null;
    public Servo IdDropper =null;
    public DcMotor  ArmUpDown        = null;
    public DcMotor C2;

    public DcMotor  ArmDistance      = null;

    public DcMotor  ArmUpDown2      = null;
    public DcMotor collector = null;

    BNO055IMU imu;

    Orientation angles;
    Acceleration gravity;


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();
    private ElapsedTime     runtime = new ElapsedTime();

    /* Constructor */
    public HardwareKrakens() {}

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        //setting the parameters for the imu(gyro)
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        MotorLeftFront  = hwMap.get(DcMotor.class, "mlf");//port 3 rev 2
        MotorRightFront = hwMap.get(DcMotor.class, "mrf");//port 2 rev2
        ArmUpDown       =hwMap.get(DcMotor.class,"aud");//port 3 rev 1
        ArmDistance     = hwMap.get(DcMotor.class, "ad");//port 0 rev 2
        ArmUpDown2     = hwMap.get(DcMotor.class, "aud2");//port 0 rev 1
        C2=hwMap.get(DcMotor.class,"c2");//port 2 rev 1


        imu = hwMap.get(BNO055IMU.class, "imu");//gyro/imu in rev
        imu.initialize(parameters);

        MotorLeftFront.setDirection(DcMotor.Direction.REVERSE);
        MotorRightFront.setDirection(DcMotor.Direction.FORWARD);
        ArmUpDown.setDirection(DcMotor.Direction.FORWARD);
        ArmDistance.setDirection(DcMotor.Direction.FORWARD);
        ArmUpDown2.setDirection(DcMotor.Direction.REVERSE);

        collector.setDirection(DcMotor.Direction.FORWARD);




        // Set all motors to zero power
        AllWheelsPower(0,0);
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        MotorRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ArmUpDown.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ArmDistance.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ArmUpDown2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        collector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        IdDropper =hwMap.get(Servo.class,"id");

    }
    public void AllWheelsPower(double LeftMotorPower,double RightMotorPower){
        MotorLeftFront.setPower(LeftMotorPower);
        MotorRightFront.setPower(RightMotorPower);
    }

    public  void waiting(double seconds ){
        runtime.reset();

        while (runtime.seconds()<seconds){

        }
    }

}

