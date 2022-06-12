package com.vrptw.algorithm.element;

import java.util.ArrayList;


import com.vrptw.instance.VrptwInstance;
import ilog.concert.IloNumVar;
import lombok.Data;

@Data
public class Path {
	private ArrayList<Integer> path;
	private int id;
	private double travalTime;
	public IloNumVar y;
	
	public Path() {
		path       = new ArrayList<Integer>();
		id         = 0;
		travalTime = 0;
	}
	
	public Path(Path p) {
		path       = new ArrayList<Integer>();
		id         = p.getId();
		travalTime = p.getTravalTime();
		y          = p.y;
		path.addAll(p.getPath());
	}

	public int isContain(int customer) {
		return path.contains(customer)? 1 : 0;
	}

	public void calcTravalTime(VrptwInstance vrptwInstance) {
		this.travalTime = 0;
		if(path.size() > 2){
			for(int i = 0; i < path.size()-1; i++) {
				this.travalTime+=vrptwInstance.getTravelTime(path.get(i), path.get(i+1));
			}
		}
	}
	
	/**
	 * Prints the path
	 */
	public String toString() {
		if(path.size() <= 2) {
			return "";
		}
		StringBuffer print = new StringBuffer();
		
		print.append("Path[" + String.format("%-3d", id) + ", " +String.format("%-3d", path.size()-2) + "] = ");
		for (int i = 0; i < path.size()-1; ++i) {
			print.append(String.format("%-3d", path.get(i))+" -> ");
		}
		print.append(String.format("%-3d",path.get(path.size()-1)));
		print.append("\n");
		return print.toString();
	}

}
