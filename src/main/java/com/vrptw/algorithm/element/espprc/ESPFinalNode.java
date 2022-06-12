package com.vrptw.algorithm.element.espprc;

import com.vrptw.instance.Node;
import lombok.Data;

import java.util.ArrayList;

@Data
public class ESPFinalNode extends Node {
	public ArrayList<Integer> path;	// Best solution found at any time of the exploration
	public double pathTime;	// Best solution time
	public double pathLoad;	// Best solution load
	public double pathCost;	// Best solution cost
	public double pathDist;	// Best solution distance

	public ESPFinalNode() {
		
	}
	
	public ESPFinalNode(Node node) {
		super(node);
		pathTime      = 0;
		pathLoad      = 0;
		pathCost      = 0;
		pathDist      = 0;
		path = new ArrayList<Integer>();
	}
	public void resetFinalSol() {
		pathTime      = 0;
		pathLoad      = 0;
		pathCost      = 0;
		pathDist      = 0;
	}

}
