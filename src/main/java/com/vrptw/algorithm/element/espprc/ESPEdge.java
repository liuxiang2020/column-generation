package com.vrptw.algorithm.element.espprc;

import lombok.Data;

@Data
public class ESPEdge implements Comparable{
	private int id;
	private ESPNode startNode;
	private ESPNode endNode;
	public double time;
	public double cost;
	public double load;
	public double dist;
	
	public ESPEdge() {
		id         = 0;
		time       = 0;
		cost       = 0;
		load       = 0;
		dist       = 0;
	}
	
	public ESPEdge(ESPEdge e) {
		id         = e.id;
		startNode  = e.startNode;
		endNode    = e.endNode;
		time       = e.time;
		cost       = e.cost;
		load       = e.load;
		dist       = e.dist;
	}
	
	public ESPEdge(int id, ESPNode x, ESPNode y) {
		this.id    = id;
		startNode  = x;
		endNode    = y;
	}

	@Override
	public int compareTo(Object obj) {
		// TODO Auto-generated method stub
		double objCost = ((ESPEdge)obj).cost;

		return this.cost > objCost? 1 : -1;
	}

}
