package org.firstinspires.ftc.teamcode.vision.ClusterSort;

import java.util.HashMap;

public class ClusterDataset {

    private HashMap<Integer,Coordinate[]> data;
    private Coordinate[] clusters;

    public ClusterDataset(HashMap<Integer,Coordinate[]> data, Coordinate[] clusters) {
        this.data=data;
        this.clusters=clusters;
    }

    public HashMap<Integer,Coordinate[]> getData() {
        return this.data;
    }

    public Coordinate[] getClusters() {
        return this.clusters;
    }

    public String toString() {
        String str = "";

        int i=0;
        while(data.containsKey(i)) {
            str+=i+": ";
            for(int j=0;j<data.get(i).length;j++) {
                str+= data.get(i)[j];
                if(j!=data.get(i).length-1) {
                    str+=", ";
                }
            }
            str+="\n";
            i++;
        }

        str+= "Cluster Positions: ";

        for(int j=0;j<clusters.length;j++) {
            str+= j + ": " + clusters[j];
            if(j!=clusters.length-1) {
                str+= ", ";
            }
        }

        return str;
    }

}
