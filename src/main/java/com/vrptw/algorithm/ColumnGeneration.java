package com.vrptw.algorithm;

import com.vrptw.algorithm.element.Path;
import com.vrptw.instance.VrptwInstance;
import com.vrptw.timer.Timer;
import lombok.extern.slf4j.Slf4j;

@Slf4j
public class ColumnGeneration {
	
	public VrptwInstance vrptwInstance;
	public RestrictedMasterProblem rstMasterProblem;
	public SubProblem subProblem;
	public Timer watch;

	public ColumnGeneration() {
		
	}
	
	public ColumnGeneration(VrptwInstance vrptwInstance) {
		this.vrptwInstance = vrptwInstance;
		rstMasterProblem = new RestrictedMasterProblem(this.vrptwInstance);
		subProblem = new SubProblem(this.vrptwInstance);
		watch = new Timer();
	}
	
	
	public void runColumnGeneration() throws InterruptedException {
		int iterationCounter = 0;
		int maxNonImprove = 0;
				
		do {
			iterationCounter++;
			maxNonImprove++;
			log.info("第----------------------------{}次迭代------------------------------", iterationCounter);
			rstMasterProblem.solveRelaxation();
			subProblem.updateDualValues(rstMasterProblem.getDualValues());
			watch.start();
			subProblem.solve();
			watch.stop();
			// 子问题的值小于一定范围，才增加新列
			if(subProblem.reducedCost <  this.vrptwInstance.parameters.zero_reduced_cost_AbortColGen) {
				//Path sbPath = new Path(subProblem.getPath());
				Path sbPath = subProblem.getPath();
				sbPath.setId(rstMasterProblem.getPaths().size()+1);
				rstMasterProblem.addNewColumn(sbPath);
				rstMasterProblem.getPaths().add(sbPath);
				maxNonImprove = 0;
				displayIteration(iterationCounter,true);
			}else {
				displayIteration(iterationCounter,false);
			}
			
		} while (maxNonImprove < this.vrptwInstance.parameters.getNoImproveIterations());
		
		rstMasterProblem.solveMIP();
	}
	
	private void displayIteration(int iter, boolean isImprove) {
		if ((iter)%20 == 0 || iter==1) {
			System.out.println();
			System.out.print("Iteration");
			System.out.print("   SB Time");
			System.out.print("     nPaths");
			System.out.print("      MP lb");
			System.out.print("          SB lb");
			System.out.print("     Improve");
			System.out.println();
		}
		System.out.format("%9.0f", (double)iter);
		System.out.format("%9.1f", watch.getSecond());
		System.out.format("%9.0f", (double)rstMasterProblem.getPaths().size());
		System.out.format("      %12.4f", rstMasterProblem.objectiveValue);//master lower bound
		System.out.format("%12.4f", subProblem.reducedCost);//sb lower bound
		if(isImprove) {
			System.out.format("%8s", "YES");//sb lower bound
		}else {
			System.out.format("%8s", "NO");//sb lower bound
		}
		System.out.println();
	}
}
