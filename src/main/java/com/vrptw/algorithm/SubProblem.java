package com.vrptw.algorithm;

import com.vrptw.algorithm.element.Path;
import com.vrptw.algorithm.element.espprc.ESPPRC;
import com.vrptw.instance.VrptwInstance;


import java.util.Map;

public class SubProblem {
	
	public double reducedCost;
	public Map<Integer,Double> rmpDualValues;
	public VrptwInstance vrptwInstance;
	private Path optPath;

	public SubProblem() {
		
	}
	
	public SubProblem(VrptwInstance vrptwInstance) {
		this.vrptwInstance = vrptwInstance;
	}
	
	public void solve() throws InterruptedException {
		//simpleHeuristic();
		espprcPulseAlgo();
	}
	
	public void espprcPulseAlgo() throws InterruptedException {
		ESPPRC espprc = new ESPPRC(vrptwInstance);
		PulseAlgorithm pulseAlgo = new PulseAlgorithm(espprc);
		optPath = pulseAlgo.runPulseAlgo(rmpDualValues);
		//借用一个变量转移reducedCost，不用重复计算
		reducedCost = optPath.getTravalTime();
		optPath.calcTravalTime(vrptwInstance);
	}

	public void updateDualValues(Map<Integer,Double> dualValues) {
		this.rmpDualValues = dualValues;
	}
	
	public Path getPath() {
		return this.optPath;
	}
}

