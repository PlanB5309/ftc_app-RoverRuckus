package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.Comparator;

public class BottomSort implements Comparator<Recognition> {
    public int compare(Recognition a, Recognition b){
        return(int)( a.getBottom() - b.getBottom());
    }

}
