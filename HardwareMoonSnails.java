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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class HardwareMoonSnails
{
    /* Public OpMode members. */
    public DcMotor  MotorLeftFront   = null;
    public DcMotor  MotorLeftBack  = null;
    public DcMotor  MotorRightFront   = null;
    public DcMotor  MotorRightBack  = null;
    public DcMotor  HookUpDown = null;
    public DcMotor  Colocte = null;
    public double TimeToGetDown = 1.0;
    public double TimeToGoUp = 1.0;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();
    private ElapsedTime     runtime = new ElapsedTime();

    /* Constructor */
    public HardwareMoonSnails() {}

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        MotorLeftFront  = hwMap.get(DcMotor.class, "MLF");
        MotorLeftBack   = hwMap.get(DcMotor.class, "MLB");
        MotorRightFront = hwMap.get(DcMotor.class, "MRF");
        MotorRightBack  = hwMap.get(DcMotor.class, "MRB");
        HookUpDown = hwMap.get(DcMotor.class, "HUD");
        Colocte = hwMap.get(DcMotor.class, "C");

        MotorLeftFront.setDirection(DcMotor.Direction.FORWARD);
        MotorLeftBack.setDirection(DcMotor.Direction.FORWARD);
        MotorRightFront.setDirection(DcMotor.Direction.REVERSE);
        MotorRightBack.setDirection(DcMotor.Direction.REVERSE);
        HookUpDown.setDirection(DcMotor.Direction.FORWARD);
        Colocte.setDirection(DcMotor.Direction.FORWARD);


        // Set all motors to zero power
        AllWheelsPower(0,0);
        HookUpDown.setPower(0);
        Colocte.setPower(0);
        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        MotorRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorLeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        HookUpDown.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Colocte.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
    public void AllWheelsPower(double LeftMotorPower,double RightMotorPower){
        MotorLeftBack.setPower(LeftMotorPower);
        MotorLeftFront.setPower(LeftMotorPower);
        MotorRightBack.setPower(RightMotorPower);
        MotorRightFront.setPower(RightMotorPower);
    }

    public void MineralIn(){
        Colocte.setPower(0.5);
    }
    public void MineralOut(){
        Colocte.setPower(0.5);
    }

    public void climbDown(){
        runtime.reset();
        while ((runtime.seconds() < TimeToGetDown)){
            HookUpDown.setPower(1);
        }
        HookUpDown.setPower(0);
    }
    public void climbUp(){
        runtime.reset();
        while ((runtime.seconds() < TimeToGoUp)){
            HookUpDown.setPower(-1);
        }
        HookUpDown.setPower(0);
    }
}

