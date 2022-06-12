package com.vrptw.algorithm.element.espprc;

import com.vrptw.instance.Node;

import java.util.ArrayList;
import java.util.HashMap;

public class ESPNode extends Node {
	
	public boolean isFirstTimeVisited;      // boolean that indicates if the node is visited for first time
	public  boolean visited;			    // Binary indicator for detecting cycles in the bounding stage
	public  boolean[] visitedMT;		    // Binary indicator for detecting cycles one for each thread
	public  double bestCost;		        // Best cost found for node at each iteration of the bounding stage
	public ArrayList<ESPEdge> outgoingEdges;// Array of edges
	public HashMap<Integer, Double> boundsMap;// Bounds matrix
	
	public ESPNode(Node node) {
		super(node);
		visited    = false;
		isFirstTimeVisited = true;
		bestCost   = Integer.MAX_VALUE;
		outgoingEdges = new ArrayList<ESPEdge>();
		boundsMap = new HashMap<Integer, Double>();
	}
}
