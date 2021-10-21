package org.firstinspires.ftc.teamcode.vision.ClusterSort;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

public class ClusterSort {

    private Coordinate[] dataPoints;

    private final int WAIT_TIME = 250;
    private final int MAX_TICKS = 50;

    /**
     *
     * @param coords The coordinates to detect the clusters of
     * @param k The number of clusters to detect
     *
     * @return A HashMap dictionary where the key is the cluster number and the value is an array of the Coordinates that are in that cluster
     */
    public ClusterDataset cluster(Coordinate[] coords, int k) {
        if(coords.length==0||k==0) { return null; }

        this.dataPoints=coords;

        HashMap<Integer,Coordinate[]> assigned = new HashMap<>();

        //Determine scale of data
        Scale scale = new Scale(coords);

        //Create cluster array
        Coordinate[] clusters = new Coordinate[k];

        //Place Initial Clusters
        print("Placing initial clusters...");
        if(k==1) {
            print("Only one cluster needed, placing it at the center...");
            clusters[0] = scale.getCenter();
        } else {
            //If there is more than one cluster, lay them out in a circular pattern around the center OF THE SCALE, where the diameter is half of the width

            //Find the angle for each of the cluster positions (in radians)
            double[] angles = new double[k];
            double multiplier = (2*(Math.PI))/k;

            for(int i = 0; i<k; i++) {
                angles[i]=i*multiplier;

                //Add small offset
                angles[i]+=1;
            }

            double radius = scale.getWidth()/2/2;

            print(String.valueOf(scale.getWidth()));

            //Add cartesian coordinates
            for(int i=0;i<k; i++) {
                clusters[i] = new Coordinate((radius*Math.cos(angles[i]))+scale.getCenter().x(),(radius*Math.sin(angles[i]))+scale.getCenter().y());
            }

        }

        print("Placed all initial clusters");

        print("----------------------------------------------");

        HashMap<Integer,Integer> old = new HashMap<>();
        for(int tick=0;tick<MAX_TICKS;tick++) {

            long startTime = System.currentTimeMillis();

            print("Beginning round " + tick + " of training...\n");

            //Assign each data point to a cluster
            HashMap<Integer, Integer> coordAssignments = assignCoordinatesToNearestCluster(coords, clusters);

            //Rearrange the data so it's easier to find the center of mass of
            //Fill the assigned dictionary values with empty Coordinate arrays
            print("Rearranging data...");
            for (int i = 0; i < k; i++) {
                assigned.put(i, new Coordinate[0]);
            }

            //Add the Coordinates into the assigned array
            print("Inserting coordinates...");
            for (int i = 0; i < coords.length; i++) {
                assigned.put(coordAssignments.get(i), addArrays(assigned.get(coordAssignments.get(i)), new Coordinate[]{coords[i]}));
            }

            //Using the coordinates in the assigned array, find the center of mass of each cluster of data and move the clusters there
            for (int i = 0; i < k; i++) {
                print("Calculating center of mass for cluster " + i + "...");
                clusters[i] = findCenterOfMassOfCoordinates(assigned.get(i));
            }

            print("\nCompleted round " + tick + " of training in " + (System.currentTimeMillis()-startTime) + " ms");
            print("----------------------------------------------");

            //Check if it should go again
            if(old.equals(coordAssignments)) {
                print("No more Coordinates are switching Clusters anymore. The training is complete.");
                tick=MAX_TICKS;
            } else {
                old=coordAssignments;
            }
        }


        print("\nCreating final ClusterDataset object...");

        print("\nDone!\n");

        //Make ClusterDataset and return
        return new ClusterDataset(assigned,clusters);

    }

    private static HashMap<Integer,Integer> assignCoordinatesToNearestCluster(Coordinate[] coords, Coordinate[] clusters) {
        //For this HashMap, the key is the index of the Coordinate in the coords array and the value is the cluster
        HashMap<Integer, Integer> coordAssignments = new HashMap<>();
        for(int i=0; i<coords.length; i++) {
            //The cluster number the coordinate will be assigned to
            int assignment = 0;
            for(int j=0;j<clusters.length;j++) {
                if(Coordinate.distanceBetween(coords[i],clusters[j])<=Coordinate.distanceBetween(coords[i],clusters[assignment])) {
                    assignment=j;
                }
            }
            coordAssignments.put(i,assignment);
        }

        return coordAssignments;
    }

    public static Coordinate findCenterOfMassOfCoordinates(Coordinate[] coords) {

        double totalX=0.0,totalY=0.0;

        ArrayList<Coordinate> filtered = new ArrayList<Coordinate>();

        for(Coordinate coord : coords) {
            if(isCoordSurrounded(coord,coords)) {
                filtered.add(coord);
            }
        }

        for(Coordinate coord : filtered) {
            totalX+=coord.x();
            totalY+=coord.y();
        }

        return new Coordinate(totalX/filtered.size(),totalY/filtered.size());
    }

    // Returns whether the coordinate given has coordinates on all four sides of it as well
    private static boolean isCoordSurrounded(Coordinate coord, Coordinate[] coords) {
        List list = Arrays.asList(coords);
        int count=0;
        for(int i=0; i<coords.length; i++) {
            if(coords[i].x()==coord.x()+1||coords[i].x()==coord.x()-1||coords[i].y()==coord.y()+1||coords[i].y()==coord.y()-1) {
                count++;
            }
            if(count>=3) {
                return true;
            }
        }
        return false;
    }

    private static Coordinate[] addArrays(Coordinate[] arr1, Coordinate[] arr2) {
        Coordinate[] newArr = new Coordinate[arr1.length+arr2.length];
        for(int i=0; i<arr1.length;i++) {
            newArr[i]=arr1[i];
        }
        for(int i=arr1.length; i<arr1.length+arr2.length;i++) {
            newArr[i] = arr2[i-arr1.length];
        }
        return newArr;
    }

    private static void print(String str) {
        System.out.println(str);
    }

    private void wait(int millis) {
        try {
            Thread.sleep(millis);
        } catch(InterruptedException e) {
            e.printStackTrace();
        }
    }

    /**
     *
     * @param dataset The dataset containing the training data for the clusterss
     * @param coord The coordinate you would like to classify
     * @return An array of percent probabilities that the coordinate is part of the cluster, in the order in which the clusters are presented in the ClusterDataset
     */
    public double[] predict(ClusterDataset dataset, Coordinate coord) {
        print("Predicting which cluster " + coord + " would reside in from a previously generated dataset...");

        double[] probabilities = new double[dataset.getClusters().length];

        //Find the max distance
        double max=Integer.MIN_VALUE;
        for(int i=0;i<dataset.getData().size();i++) {
            probabilities[i]=Coordinate.distanceBetween(dataset.getClusters()[i],coord);
            System.out.println(probabilities[i]);
            if(max<Coordinate.distanceBetween(dataset.getClusters()[i],coord)) {
                max = Coordinate.distanceBetween(dataset.getClusters()[i],coord);
            }
        }

        print(String.valueOf(max));

        //Divide all the other distances by the max to get the percent distance they are away from it and invert the value
        for(int i=0;i<probabilities.length;i++) {
            probabilities[i]/=max;
            probabilities[i] = Math.abs(probabilities[i]-1);

        }

        //Add up all the probabilities to get the percent of each
        double total=0.0;
        for(double p : probabilities) {
            total+=p;
        }

        //Get the percent
        for(int i=0;i<probabilities.length;i++) {
            probabilities[i] = probabilities[i]/total;
        }

        return probabilities;
    }

}

