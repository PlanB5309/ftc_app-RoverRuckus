package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.Comparator;

public class LeftSort implements Comparator<Recognition> {
    public int compare(Recognition a, Recognition b){
        return(int)( a.getLeft() - b.getLeft());
    }

}
