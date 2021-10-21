package org.firstinspires.ftc.teamcode.vision;
import android.content.Context;
import android.graphics.Bitmap;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.io.FileOutputStream;
import java.io.IOException;

public class Camera {

    public Bitmap bitmap;

    public VuforiaLocalizer vuforia;
    private VuforiaLocalizer.Parameters parameters;

    public boolean save;

    public Camera(HardwareMap hardwareMap, boolean save){
        initVuforia(hardwareMap,true);
        this.save = save;
        cycle();
    }

    public void initVuforia(HardwareMap hardwareMap, boolean cameraView) //False saves battery, true displays the screen
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        if (cameraView) {
            this.parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        } else {
            this.parameters = new VuforiaLocalizer.Parameters();

        }

        //HAZMAT KEY
        this.parameters.vuforiaLicenseKey = "AQO6Kaz/////AAABmdRW+ClM7EAjvcb5+/EslnlH1otFDS9Cw7xuHMuPZuOC4Xdupz9OruDx80AW4ed8KySJO0byAlMpDUDpf1U7KyBuU32dP77B1s4yFJUwpys74t/uDW5637aeb2JSv5OEgdkpbveoJAqAKwpVTrpiSqJtHcUrO86GTRs+8yax6cAOl/+vreCCtxTIIOJemO4d6lbmPsMogD6UChuk37/DyIn3PR63UpdeqUAd2os8p5rcot4X+52niYi3jOXww+ozM/+eOUJ7EXf5IGbWTXxwHkdNPHXRJhtvtcq6+Mhk+LSHG7cie8ZtQLX0RWtyPiosHdl6l6blv/svsuhCxaRDq+i8KMdVTnj/ud89VJecrM+X";

        this.parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.getInstance().createVuforia(parameters);

        com.vuforia.Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true); //enables RGB565 format for the image
        this.vuforia.setFrameQueueCapacity(1); //tells VuforiaLocalizer to only store one frame at a time
    }

    public int getColor(int x, int y){
        return this.bitmap.getPixel(x,y);
    }
    public int getRed(int x, int y){
        return Color.getRed(this.bitmap.getPixel(x,y));
    }
    public int getGreen(int x, int y){
        return Color.getGreen(this.bitmap.getPixel(x,y));
    }
    public int getBlue(int x, int y){
        return Color.getBlue(this.bitmap.getPixel(x,y));
    }

    public void cycle(){
        try
        {
            VuforiaLocalizer.CloseableFrame frame = this.vuforia.getFrameQueue().take();
            long numImages = frame.getNumImages();

            for (int i = 0; i < numImages; i++) {
                if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                    this.bitmap = imgToBitmap(frame.getImage(i));
                    break;
                }
            }
        }catch(InterruptedException ie)
        {
            this.bitmap = null;
        }
    }
    public Bitmap imgToBitmap(Image image){
        Bitmap img = Bitmap.createBitmap(image.getWidth(), image.getHeight(), Bitmap.Config.RGB_565);
        img.copyPixelsFromBuffer(image.getPixels());
        img = Bitmap.createBitmap(img,450,0,250,img.getHeight());
        return img;
    }

    public void save(Bitmap bmp, String name){
        if(save) {
            String filename = name + ".PNG";

            FileOutputStream out = null;
            try {
                out = new FileOutputStream(filename);
                bmp.compress(Bitmap.CompressFormat.PNG, 100, out); // bmp is your Bitmap instance
                // PNG is a lossless format, the compression factor (100) is ignored
            } catch (Exception e) {
                e.printStackTrace();
            } finally {
                try {
                    if (out != null) {
                        out.close();
                    }
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        }
    }

    public void save(String name){
        save(this.bitmap,name);
    }

    public void take_picture(){
        cycle();
        save("Image_Capture");
    }

    public static void download(Bitmap bmp, String name){
        String filename = "/sdcard/"+name + ".PNG";

        FileOutputStream out = null;
        try {
            out = new FileOutputStream(filename);
            bmp.compress(Bitmap.CompressFormat.PNG, 100, out); // bmp is your Bitmap instance
            // PNG is a lossless format, the compression factor (100) is ignored
        } catch (Exception e) {
            e.printStackTrace();
        } finally {
            try {
                if (out != null) {
                    out.close();
                }
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }

    public int[] bitmapToArray(Bitmap bmp){
        int[] o = new int[bmp.getWidth()*bmp.getHeight()];

        bmp.getPixels(o,0,bmp.getWidth(),0,0,bmp.getWidth(),bmp.getHeight());
        return o;
    }
}