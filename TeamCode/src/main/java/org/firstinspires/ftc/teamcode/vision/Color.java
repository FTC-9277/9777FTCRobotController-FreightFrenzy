package org.firstinspires.ftc.teamcode.vision;

public class Color {

    //175y 220w

    //y150 w200 - OCT19
    public static final int yellow_threshold = 175;
    public static final int white_threshold = 200;

    public static int getRed(int colorInt){return ((colorInt >> 16) & 0xff);}
    public static int getGreen(int colorInt){return ((colorInt >>  8) & 0xff);}
    public static int getBlue (int colorInt){return ((colorInt      ) & 0xff);}

    //is a pixel yellow
    public static boolean isYellow(int c){
        return isYellow(getRed(c),getGreen(c),getBlue(c));
    }
    public static boolean isYellow(int r, int g, int b){
        return (255-b > yellow_threshold) && !(255-g > yellow_threshold) && !(255-r > yellow_threshold);
    }

    //is a pixel white
    public static boolean isWhite(int c){
        return isWhite(getRed(c),getGreen(c),getBlue(c));
    }
    public static boolean isWhite(int r, int g, int b){
        return ((r > white_threshold) && (g > white_threshold) && (b > white_threshold));
    }
}