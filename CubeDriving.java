package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import java.util.List;
import java.util.Locale;

/**
 * Created by guinea on 10/5/17.
 * -------------------------------------------------------------------------------------
 * Copyright (c) 2018 FTC Team 5484 Enderbots
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
 * 
 * By downloading, copying, installing or using the software you agree to this license.
 * If you do not agree to this license, do not download, install,
 * copy or use the software.
 * -------------------------------------------------------------------------------------
 * This is a sample opmode that demonstrates the use of an OpenCVPipeline with FTC code.
 * When the x button is pressed on controller one, the camera is set to show areas of the image
 * where a certain color is, in this case, gold.
 *
 * Additionally, the centers of the bounding rectangles of the contours are sent to telemetry.
 */
@TeleOp(name="CubeDriving", group = "MoonSnails")
public class CubeDriving extends OpMode {
    private ExamplegoldVision goldVision;
    public DcMotor MotorLeftFront   = null;
    public DcMotor  MotorLeftBack  = null;
    public DcMotor  MotorRightFront   = null;
    public DcMotor  MotorRightBack  = null;
    public DcMotor  HookUpDown = null;
    public double TimeToGetDown = 1.0;
    public int i=0;
    public double midpixX =146;
    public double leftspeed=0.6;
    public double rightspeed=0.6;


    private ElapsedTime period  = new ElapsedTime();
    private ElapsedTime     runtime = new ElapsedTime();

    public void init() {
        MotorLeftFront  = hardwareMap.get(DcMotor.class, "MLF");
        MotorLeftBack   = hardwareMap.get(DcMotor.class, "MLB");
        MotorRightFront = hardwareMap.get(DcMotor.class, "MRF");
        MotorRightBack  = hardwareMap.get(DcMotor.class, "MRB");
        MotorLeftFront.setDirection(DcMotor.Direction.FORWARD);
        MotorLeftBack.setDirection(DcMotor.Direction.FORWARD);
        MotorRightFront.setDirection(DcMotor.Direction.REVERSE);
        MotorRightBack.setDirection(DcMotor.Direction.REVERSE);



        goldVision = new ExamplegoldVision();
        goldVision.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        goldVision.setShowCountours(false);
        // start the vision system
        goldVision.enable();
    }


    @Override
    public void loop() {}

    public void AllWheelsPower(double LeftMotorPower,double RightMotorPower){
        MotorLeftBack.setPower(LeftMotorPower);
        MotorLeftFront.setPower(LeftMotorPower);
        MotorRightBack.setPower(RightMotorPower);
        MotorRightFront.setPower(RightMotorPower);
    }
    public void CubeRub(){
        // update the settings of the vision pipeline
        goldVision.setShowCountours(true);
        // get a list of contours from the vision system
        List<MatOfPoint> contours = goldVision.getContours();
        while ( i < contours.size()) {
            AllWheelsPower(leftspeed,rightspeed);
            Rect boundingRect = Imgproc.boundingRect(contours.get(i));
            if(boundingRect.x>midpixX){
                leftspeed=0.7;
                rightspeed=0.6;
            }
            if (boundingRect.x<midpixX){
                rightspeed=0.7;
                leftspeed=0.6;
            }
            if (boundingRect.x==midpixX){
                rightspeed=0.6;
                leftspeed=0.6;
            }
            i++;
        }
    }



    public void stop() {
        // stop the vision system
        goldVision.disable();
    }
}
