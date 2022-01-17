package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.teamcode.vision.ClusterSort.*;

import java.util.ArrayList;


public class Sampler {

//    // If the percentage of pixels in the given image are lower than this, it will be assumed that 0 rings are present.
//    static final double RING_EXISTS_THRESHOLD=0.02;

    public static void main(String[] args) {

        long startTime = System.currentTimeMillis();

//        int none = sample("Images/1.jpg");

//        System.out.println(none);

        System.out.println("The entire process took " + (System.currentTimeMillis()-startTime) + " millis");

    }

    public Camera camera;

    public Sampler(Camera camera) {
        this.camera=camera;
    }

    public final int TERMINATE_LIMIT=15;

    public String sample() {

        System.out.println(System.currentTimeMillis());

        int[] rgb = camera.bitmapToArray(camera.bitmap);

        int width=camera.bitmap.getWidth();
        int height=camera.bitmap.getHeight();

        return "Hi! First pixel: " + rgb[0];
    }


    static int getRed  (int c){return ((c >> 16) & 0xff);}
    static int getGreen(int c){return ((c >>  8) & 0xff);}
    static int getBlue (int c){return ((c      ) & 0xff);}

    private static int colorToRGB(int alpha, int red, int green, int blue) {
        int newPixel = 0;
        newPixel += alpha;
        newPixel = newPixel << 8;
        newPixel += red;
        newPixel = newPixel << 8;
        newPixel += green;
        newPixel = newPixel << 8;
        newPixel += blue;

        return newPixel;
    }

}
