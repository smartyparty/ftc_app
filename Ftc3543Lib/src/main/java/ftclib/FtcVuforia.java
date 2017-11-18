/*
 * Copyright (c) 2016 Titan Robotics Club (http://www.titanrobotics.com)
 * Based on sample code by Robert Atkinson.
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
 */

package ftclib;

import android.graphics.Bitmap;

import com.vuforia.HINT;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

import hallib.HalVideoSource;

/**
 * This class makes using Vuforia a little easier by minimizing the number of calls to it. It only exposes the
 * minimum things you need to set for the FTC competition. If you want to do more complex stuff, you may want
 * to not use this and call Vuforia directly so you can customize other stuff.
 */
public class FtcVuforia implements HalVideoSource<Mat>
{
    /**
     * This class contains information required to make a trackable target. It has two constructors. One with all the
     * rotation/translation info for tracking the robot location on the field. If you don't need to track the robot's
     * location, then you can use the constructor with only the target name.
     */
    public static class Target
    {
        public final String name;
        public final float rotateX;
        public final float rotateY;
        public final float rotateZ;
        public final float translateX;
        public final float translateY;
        public final float translateZ;

        public Target(
                final String name, final float rotateX, final float rotateY, final float rotateZ,
                final float translateX, final float translateY, final float translateZ)
        {
            this.name = name;
            this.rotateX = rotateX;
            this.rotateY = rotateY;
            this.rotateZ = rotateZ;
            this.translateX = translateX;
            this.translateY = translateY;
            this.translateZ = translateZ;
        }   //Target

        public Target(final String name)
        {
            this(name, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        }   //Target
    }   //class Target

    public VuforiaLocalizer localizer;
    private VuforiaLocalizer.CameraDirection cameraDir;
    private VuforiaTrackables targetList = null;
    private int imageWidth = 0;
    private int imageHeight = 0;

    /**
     * Constructor: Create an instance of this object. It initializes Vuforia with the specified target images and
     * other parameters.
     *
     * @param licenseKey specifies the Vuforia license key.
     * @param cameraViewId specifies the camera view ID on the activity, -1 if none given.
     * @param cameraDir specifies which camera to use (front or back).
     * @param trackablesFile specifies the XML file that contains the target info, can be null.
     * @param numTargets specifies the number of simultaneous trackable targets.
     * @param cameraMonitorFeedback specifies the feedback image showing the orientation of the target.
     */
    public FtcVuforia(
            String licenseKey, int cameraViewId, VuforiaLocalizer.CameraDirection cameraDir,
            String trackablesFile, int numTargets,
            VuforiaLocalizer.Parameters.CameraMonitorFeedback cameraMonitorFeedback)
    {
        this.cameraDir = cameraDir;
        //
        // If no camera view ID, do not activate camera monitor view to save power.
        //
        VuforiaLocalizer.Parameters params =
                cameraViewId == -1? new VuforiaLocalizer.Parameters(): new VuforiaLocalizer.Parameters(cameraViewId);
        params.vuforiaLicenseKey = licenseKey;
        params.cameraDirection = cameraDir;
        params.cameraMonitorFeedback = cameraMonitorFeedback;
        localizer = ClassFactory.createVuforiaLocalizer(params);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, numTargets);
        if (trackablesFile != null)
        {
            targetList = localizer.loadTrackablesFromAsset(trackablesFile);
        }
    }   //FtcVuforia

    /**
     * Constructor: Create an instance of this object. It initializes Vuforia with the specified target images and
     * other parameters.
     *
     * @param licenseKey specifies the Vuforia license key.
     * @param cameraViewId specifies the camera view ID on the activity.
     * @param cameraDir specifies which camera to use (front or back).
     * @param trackablesFile specifies the XML file that contains the target info.
     * @param numTargets specifies the number of simultaneous trackable targets.
     */
    public FtcVuforia(
            String licenseKey, int cameraViewId, VuforiaLocalizer.CameraDirection cameraDir,
            String trackablesFile, int numTargets)
    {
        this(licenseKey, cameraViewId, cameraDir, trackablesFile, numTargets,
             VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES);
    }   //FtcVuforia

    /**
     * Constructor: Create an instance of this object. It initializes Vuforia with the specified target images and
     * other parameters.
     *
     * @param licenseKey specifies the Vuforia license key.
     * @param cameraViewId specifies the camera view ID on the activity.
     * @param cameraDir specifies which camera to use (front or back).
     */
    public FtcVuforia(String licenseKey, int cameraViewId, VuforiaLocalizer.CameraDirection cameraDir)
    {
        this(licenseKey, cameraViewId, cameraDir, null, 0, VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES);
    }   //FtcVuforia

    /**
     * This method enables/disables target tracking.
     *
     * @param enabled specifies true to enable target tracking, false otherwise.
     */
    public void setTrackingEnabled(boolean enabled)
    {
        if (targetList != null)
        {
            if (enabled)
            {
                targetList.activate();
            } else
            {
                targetList.deactivate();
            }
        }
    }   //setTrackingEnabled

    /**
     * This method sets the properties of the specified target.
     *
     * @param index specifies the target index in the XML file.
     * @param name specifies the target name.
     * @param locationOnField specifies the target location on the field, can be null if no robot tracking.
     * @param phoneLocationOnRobot specifies the phone location on the robot, can be null if no robot tracking.
     */
    public void setTargetInfo(int index, String name, OpenGLMatrix locationOnField, OpenGLMatrix phoneLocationOnRobot)
    {
        if (targetList != null)
        {
            VuforiaTrackable target = targetList.get(index);
            target.setName(name);

            if (locationOnField != null)
            {
                target.setLocation(locationOnField);
            }

            if (phoneLocationOnRobot != null)
            {
                ((VuforiaTrackableDefaultListener) target.getListener()).setPhoneInformation(
                        phoneLocationOnRobot, cameraDir);
            }
        }
    }   //setTargetInfo

    /**
     * This method sets the properties of the specified target.
     *
     * @param index specifies the target index in the XML file.
     * @param name specifies the target name.
     */
    public void setTargetInfo(int index, String name)
    {
        setTargetInfo(index, name, null, null);
    }   //setTargetInfo

    /**
     * This method sets tracking info for the targets described in the given target array.
     *
     * @param targets specifies the array of targets to set tracking info.
     * @param phoneLocationOnRobot specifies the location marix of the phone on the robot.
     */
    public void setTargets(Target[] targets, OpenGLMatrix phoneLocationOnRobot)
    {
        for (int i = 0; i < targets.length; i++)
        {
            OpenGLMatrix targetLocationOnField =
                    phoneLocationOnRobot == null?
                            null:
                            locationMatrix(
                                    targets[i].rotateX, targets[i].rotateY, targets[i].rotateZ,
                                    targets[i].translateX, targets[i].translateY, targets[i].translateZ);
            setTargetInfo(i, targets[i].name, targetLocationOnField, phoneLocationOnRobot);
        }
    }   //setTargets

    /**
     * This method creates a location matrix that can be used to relocate an object to its final location by rotating
     * and translating the object from the origin of the field. It is doing the operation in the order of the
     * parameters. In other words, it will first rotate the object on the X-axis, then rotate on the Y-axis, then
     * rotate on the Z-axis, then translate on the X-axis, then translate on the Y-axis and finally translate on the
     * Z-axis.
     *
     * @param rotateX specifies rotation on the X-axis.
     * @param rotateY specifies rotation on the Y-axis.
     * @param rotateZ specifies rotation on the Z-axis.
     * @param translateX specifies translation on the X-axis.
     * @param translateY specifies translation on the Y-axis.
     * @param translateZ specifies translation on the Z-axis.
     * @return returns the location matrix.
     */
    public OpenGLMatrix locationMatrix(
            float rotateX, float rotateY, float rotateZ, float translateX, float translateY, float translateZ)
    {
        return OpenGLMatrix.translation(translateX, translateY, translateZ)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, rotateX, rotateY, rotateZ));
    }   //locationMatrix

    /**
     * This method returns the list of trackable targets.
     *
     * @return list of trackable targets.
     */
    public VuforiaTrackables getTargetList()
    {
        return targetList;
    }   //getTargetList

    /**
     * This method returns the target object with the specified index in the target list.
     *
     * @param index specifies the target index in the list.
     * @return target.
     */
    public VuforiaTrackable getTarget(int index)
    {
        return targetList != null? targetList.get(index): null;
    }   //getTarget

    /**
     * This method returns the target object with the specified target name.
     *
     * @param name specifies the name of the target.
     * @return target.
     */
    public VuforiaTrackable getTarget(String name)
    {
        VuforiaTrackable target = null;

        if (targetList != null)
        {
            for (int i = 0; i < targetList.size(); i++)
            {
                target = targetList.get(i);
                if (name.equals(target.getName()))
                {
                    break;
                } else
                {
                    target = null;
                }
            }
        }

        return target;
    }   //getTarget

    /**
     * This method determines if the target is visible.
     *
     * @param target specifies the target object.
     * @return true if the target is in view, false otherwise.
     */
    public boolean isTargetVisible(VuforiaTrackable target)
    {
        VuforiaTrackableDefaultListener listener = (VuforiaTrackableDefaultListener)target.getListener();
        return listener.isVisible();
    }   //isTargetVisible

    /**
     * This method returns the position matrix of the specified target.
     *
     * @param target specifies the target to get the position matrix.
     * @return position matrix of the specified target.
     */
    public OpenGLMatrix getTargetPose(VuforiaTrackable target)
    {
        VuforiaTrackableDefaultListener listener = (VuforiaTrackableDefaultListener)target.getListener();
        return listener.getPose();
    }   //getTargetPose

    /**
     * This method determines the robot location by the given target.
     *
     * @param target specifies the target to be used to determine robot location.
     * @return robot location matrix.
     */
    public OpenGLMatrix getRobotLocation(VuforiaTrackable target)
    {
        VuforiaTrackableDefaultListener listener = (VuforiaTrackableDefaultListener)target.getListener();
        return listener.getRobotLocation();
    }   //getRobotLocation

    /**
     * This method configures Vuforia to capture video frames of the given format.
     *
     * @param imageWidth specifies the image width to capture.
     * @param imageHeight specifies the image height to capture.
     * @param queueCapacity specifies the frame queue capacity.
     */
    public void configVideoSource(int imageWidth, int imageHeight, int queueCapacity)
    {
        this.imageWidth = imageWidth;
        this.imageHeight = imageHeight;

        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        localizer.setFrameQueueCapacity(queueCapacity);
    }   //configVideoSource

    //
    // Implements HalVideoSource interface.
    //

    /**
     * This method gets a frame from the frame queue and returns the image that matches the format specified by the
     * configVideoSource method.
     *
     * @param frame specifies the frame object to hold image.
     * @return true if success, false otherwise.
     */
    @Override
    public boolean getFrame(Mat frame)
    {
        boolean success = false;

        try
        {
            VuforiaLocalizer.CloseableFrame closeableFrame = localizer.getFrameQueue().take();

            for (int i = 0; i < closeableFrame.getNumImages(); i++)
            {
                Image image = closeableFrame.getImage(i);
                if (image.getWidth() == imageWidth && image.getHeight() == imageHeight &&
                        image.getFormat() == PIXEL_FORMAT.RGB565)
                {
                    Bitmap bm = Bitmap.createBitmap(image.getWidth(), image.getHeight(), Bitmap.Config.RGB_565);
                    bm.copyPixelsFromBuffer(image.getPixels());
                    Utils.bitmapToMat(bm, frame);
                    break;
                }
            }

            closeableFrame.close();
            success = true;
        }
        catch (InterruptedException e)
        {
            e.printStackTrace();
        }

        return success;
    }   //getFrame

    /**
     * This method draws the given image frame to the display surface.
     *
     * @param frame specifies the image frame to be displayed.
     */
    @Override
    public void putFrame(Mat frame)
    {
        // TODO: figure out how to render frame back to Vuforia.
    }   //putFrame

}   //class FtcVuforia
