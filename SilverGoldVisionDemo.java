package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
 * where a certain color is, in this case, blue.
 *
 * Additionally, the centers of the bounding rectangles of the Scontours are sent to telemetry.
 */
@TeleOp(name="Example: Blue Vision Demo")
public class SilverGoldVisionDemo extends OpMode {
    private silverVision silverVision;
    private goldVision goldVision;
    double goldlocition=0;
    double silverlocition=0;
    double locitioncount=1;

    @Override
    public void init() {
        silverVision = new silverVision();
        // can replace with ActivityViewDisplay.getInstance() for fullscreen
        silverVision.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        silverVision.setShowCountours(false);
        // start the vision system
        silverVision.enable();
        goldVision = new goldVision();
        // can replace with ActivityViewDisplay.getInstance() for fullscreen
        goldVision.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        goldVision.setShowCountours(false);
        // start the vision system
        goldVision.enable();
    }

    @Override
    public void loop() {
        // update the settings of the vision pipeline
        silverVision.setShowCountours(true);


        List<MatOfPoint> Scontours = silverVision.getContours();
        List<MatOfPoint> Gcontours = goldVision.getContours();
        for (int i = 0; i < Scontours.size(); i++) {
            // get the bounding rectangle of a single contour, we use it to get the x/y center
            // yes there's a mass center using Imgproc.moments but w/e
            Rect SboundingRect = Imgproc.boundingRect(Scontours.get(i));
            silverVision.setShowCountours(true);

            // get the bounding rectangle of a single contour, we use it to get the x/y center
            // yes there's a mass center using Imgproc.moments but w/e
            Rect GboundingRect = Imgproc.boundingRect(Gcontours.get(i));
            goldlocition=goldlocition+GboundingRect.x;
            silverlocition=silverlocition+SboundingRect.x;
            if (locitioncount>20){
                goldlocition=goldlocition/locitioncount;
                silverlocition=silverlocition/locitioncount;
            }
            while (locitioncount>20) {
                if (goldlocition + 20 > silverlocition && goldlocition - 20 < silverlocition) {
                    telemetry.addData("locition:", "mid");
                }
                if (goldlocition + 20 < silverlocition) {
                    telemetry.addData("locition:", "right");
                }
                if (goldlocition - 20 > silverlocition) {
                    telemetry.addData("locition:", "left");
                }
            }

            locitioncount=locitioncount+1;
        }

    }

    public void stop() {
        // stop the vision system
        silverVision.disable();
    }
}
