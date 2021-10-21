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

    public double[] sample() {

        System.out.println(System.currentTimeMillis());

        int[] rgb = camera.bitmapToArray(camera.bitmap);

        int width=camera.bitmap.getWidth();
        int height=camera.bitmap.getHeight();

        Coordinate center = findCenterOfYellow(getYellowCoordsFromPixelData(rgb,width,height));

//            markImage(image,center);

        System.out.println(center);

        // This count represents the number of non-yellow pixels found in the search. If this number gets to be 5 or more, the search will terminate and assume that all the yellow pixels have been found
        int terminateCount=0;

        // This count is how many pixels in the positive x direction the code has searched so far. It will increase by 1 every loop
        int xCount=0;

        // The maximum number of pixels is 1/5 of the image's width
        while(terminateCount<TERMINATE_LIMIT && xCount<=(width/5)) {
            xCount+=1;
            if(!isCoordYellow((int)Math.round(center.x())+xCount,(int)Math.round(center.y()),rgb,width,height)) {
                // If the given pixel is not yellow, add one to the terminate count
                terminateCount++;
            } else {
                // If it is yellow, make the terminate count go down one only if it is above 0
                if(terminateCount>0) {
                    terminateCount--;
                }
            }
        }

        xCount -= terminateCount;

        // Once you have gotten here, the total x length of the yellow spot

        int totalWidth = (xCount*2)+1;


        // Now calculate this but for the y, twice. Once for positive y, once for negative y


        // This count represents the number of non-yellow pixels found in the search. If this number gets to be TERMINATE_LIMIT or more, the search will terminate and assume that all the yellow pixels have been found
        terminateCount=0;

        // This count is how many pixels in the positive y direction the code has searched so far. It will increase by 1 every loop
        int yCount=0;

        // The maximum number of pixels is 1/5 of the image's height
        while(terminateCount<TERMINATE_LIMIT && yCount<=(height/5)) {
            yCount+=1;
            if(!isCoordYellow((int)Math.round(center.x()),(int)Math.round(center.y()+yCount),rgb,width,height)) {
                // If the given pixel is not yellow, add one to the terminate count
                terminateCount++;
            } else {
                // If it is yellow, make the terminate count go down one only if it is above 0
                if(terminateCount>0) {
                    terminateCount--;
                }
            }
        }

        yCount -= terminateCount;

        // Once you have gotten here, the total y height of the yellow spot

        int totalHeightUp = (yCount*2)+1;

        // This count represents the number of non-yellow pixels found in the search. If this number gets to be TERMINATE_LIMIT or more, the search will terminate and assume that all the yellow pixels have been found
        terminateCount=0;

        // This count is how many pixels in the NEGATIVE y direction the code has searched so far. It will increase by 1 every loop
        yCount=0;

        // The maximum number of pixels is 1/5 of the image's height
        while(terminateCount<TERMINATE_LIMIT && yCount<=(height/5)) {
            yCount-=1;
            if(!isCoordYellow((int)Math.round(center.x()),(int)Math.round(center.y()+yCount),rgb,width,height)) {
                // If the given pixel is not yellow, add one to the terminate count
                terminateCount++;
            } else {
                // If it is yellow, make the terminate count go down one only if it is above 0
                if(terminateCount>0) {
                    terminateCount--;
                }
            }
        }

        yCount -= terminateCount;

        // Once you have gotten here, the total y height of the yellow spot

        int totalHeightDown = (yCount*2)+1;
        totalHeightDown = -totalHeightDown;

        System.out.println("Total width of yellow spot: " + totalWidth);
        System.out.println("Total height of yellow spot (positive): " + totalHeightUp);
        System.out.println("Total height of yellow spot (negative): " + totalHeightDown);


        // Now use this information to determine how many rings are there and return

        double[] arr = new double[8];

//        if (totalWidth <= 5 || (totalHeightDown <= 5 && totalHeightUp <= 5)) {
//            arr[0] = 0;
//        } else if ((totalHeightUp <= totalWidth*0.3) || totalHeightDown <= totalWidth*0.3) {
//            arr[0] = 1;
//        } else if (totalHeightUp > totalWidth*0.3 || totalHeightDown > totalWidth*0.3) {
//            arr[0] = 4;
//        } else {
//            arr[0] = 0;
//        }

        if(totalHeightUp+totalHeightDown>=200) {
            arr[0]=4;
        } else if(totalHeightUp+totalHeightDown>=85) {
            arr[0]=1;
        } else {
            arr[0]=0;
        }

        arr[1] = totalWidth;
        arr[2] = totalHeightUp;
        arr[3] = totalHeightDown;

        arr[4] = width;
        arr[5] = height;

        arr[6] = center.x();
        arr[7] = center.y();

        return arr;

    }


    static int getRed  (int c){return ((c >> 16) & 0xff);}
    static int getGreen(int c){return ((c >>  8) & 0xff);}
    static int getBlue (int c){return ((c      ) & 0xff);}

    /*

    Trials: 5000
    Score: 1.8508705207966893
    Values: [60, 249, 30, 216, 2, 69]
    Empty percent: 0.035638913165574874, Full percent: 0.8865094339622641

    Trials: 15000
    Score: 1.897130869894691
    Values: [63, 251, 0, 240, 0, 64]
    Empty percent: 0.02889114268392542, Full percent: 0.9260220125786164

    Trials: 30000
    Score: 1.8346512590302835
    Values: [35, 241, 4, 213, 0, 83]
    Empty percent: 0.10387075354833292, Full percent: 0.9385220125786163

     */

    static boolean isYellow(int c) {
//        return (getRed(c) >= 130) && (getGreen(c) >= 45 && getGreen(c) <= 170) && (getBlue(c) <= 120);
        //61, 159, 217, 231, 3, 220
        return (getRed(c) >= 100 && getRed(c) <= 255) && (getGreen(c) >= 0 && getGreen(c) <= 240) && (getBlue(c) >= 0 && getBlue(c) <= 70);
    }


    /*
        Returns the RGB integer array from the given image file name
     */
//    static int[] getRGBFromImage(BufferedImage image) throws IOException {
//        return image.getRGB(0,0,image.getWidth(),image.getHeight(),null,0,image.getWidth());
//    }

//    static BufferedImage getImage(String file) throws IOException {
//
////        //Create file for the source
//        File input = new File(file);
////
//        ImageIO.setUseCache(false);
////
////        //Read the file to a BufferedImage
//        BufferedImage image = ImageIO.read(input);
//
////        image = toBufferedImage(image.getScaledInstance(300,255,0));
//
////        return image.getSubimage(image.getWidth()/5,image.getHeight()/3,image.getWidth()/2,(int)Math.round(image.getHeight()/1.5));
//
//        return image;
//    }

//    private static BufferedImage toBufferedImage(Image img)
//    {
//        if (img instanceof BufferedImage)
//        {
//            return (BufferedImage) img;
//        }
//
//        // Create a buffered image with transparency
//        BufferedImage bimage = new BufferedImage(img.getWidth(null), img.getHeight(null), BufferedImage.TYPE_INT_ARGB);
//
//        // Draw the image on to the buffered image
//        Graphics2D bGr = bimage.createGraphics();
//        bGr.drawImage(img, 0, 0, null);
//        bGr.dispose();
//
//        // Return the buffered image
//        return bimage;
//    }


    static Coordinate[] getYellowCoordsFromPixelData(int[] pixels, int width, int height) {

        ArrayList<Coordinate> coords = new ArrayList<Coordinate>();

        for(int i=0; i<pixels.length; i++) {
            if(isYellow(pixels[i])) {
                coords.add(getCoordsFromPixel(i,pixels,width,height));
            }
        }

        Coordinate[] finalCoords = new Coordinate[coords.size()];

        for(int i=0; i<coords.size(); i++) {
            finalCoords[i] = coords.get(i);
        }

        return finalCoords;

    }


    // Returns if the specified x and y coordinate in a given image is yellow or not
    static boolean isCoordYellow(int x, int y, int[] pixels, int width, int height) {
        return (isYellow(pixels[getPixelFromCoords(x,y,pixels,width,height)]));
    }

    // Returns the x and y coordinates of the pixel k in a given image
    static Coordinate getCoordsFromPixel(int k, int[] pixels, int width, int height) {
        // The y is the rows that have come before
        int y = height - (int) Math.ceil(((double) k / width));

        // The x coordinate is i minus all the rows that have come before
        int x = width - ((width * (height - y)) - k);

        // Special case
        if (k == 0) {
            x = 0;
            y = height - 1;
        }

        return new Coordinate(x,y);
    }

    // Returns the index in the pixels array provided that is associated with the given x and y
    static int getPixelFromCoords(int x, int y, int[] pixels, int width, int height){
        int pixel = ((height-y)*width)+(x);

        // Error avoidance
        if(pixel<0 || pixel>=pixels.length) {
            return 0;
        }

        return pixel;
    }


    static double findPercentOfYellow(int[] pixels) {
        int count=0;
        for(int i=0; i<pixels.length; i++) {
            if(isYellow(pixels[i])) {
                count++;
            }
        }
        System.out.println(((double)count)/pixels.length);
        return ((double)count)/pixels.length;
    }


    /*
        This method will use the ClusterSort algorithm to find the cluster of pixels that exist amongst the given data. It will return the center of the cluster as a Coordinate
     */
    static Coordinate findCenterOfYellow(Coordinate[] coords) {
        ClusterSort cluster = new ClusterSort();

        //Use the given coords and look for 1 cluster so k=1
//        ClusterDataset data = cluster.cluster(coords,1);

        Coordinate center = cluster.findCenterOfMassOfCoordinates(coords);

        return center;
    }


//    // Replaces all the classified Yellow pixels in the given image with a teal color and saves it to the converted folder as a BMP
//    private static void markImage(BufferedImage image, Coordinate center) throws IOException {
//        int[] rgb = getRGBFromImage(image);
//        for(int i=3; i<image.getHeight()*image.getWidth()-6; i++) {
//            if(isYellow(rgb[i])) {
//                int height = image.getHeight();
//                int width = image.getWidth();
//
//                // The y is the rows that have come before
//                int y = height - (int) Math.ceil(((double) i / width));
//
//                // The x coordinate is i minus all the rows that have come before
//                int x = width - ((width * (height - y)) - i);
//
//                // Special case
//                if (i == 0) {
//                    x = 0;
//                    y = height - 1;
//                }
//
//                y = height - y - 1;
//
//                x = x - 1;
//                image.setRGB(x, y, colorToRGB(100, 0, 255, 255));
//            }
//        }
//
//        image.setRGB((int)center.x()-1, image.getHeight()-(int)center.y()-1, colorToRGB(100,255,0,0));
//
//        ImageIO.write(image,"bmp", new File("Images/converted/image.bmp"));
//    }



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
