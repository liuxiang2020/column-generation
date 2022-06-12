package com.vrptw.main;


import com.vrptw.algorithm.ColumnGeneration;
import com.vrptw.instance.VrptwInstance;
import com.vrptw.parameters.Parameters;

public class CGMainRun {

	public static void main(String[] args) throws InterruptedException {
		// TODO Auto-generated method stub
		String fileName = "Solomon/C101.txt";
		if(args.length>0){
			fileName = args[0];
		}
		Parameters parm = new Parameters(fileName);
		parm.updateParameters(args);

		System.out.println("<<<<<<<<<<< Column Generation For VRPTW, instance : "+parm.getInstanceName() +" >>>>>>>>>>>");
		System.out.println(parm.toString());

		VrptwInstance vrptwIns = new VrptwInstance(parm);
		vrptwIns.readInstanceFromFile(parm.getInputFileName());
		vrptwIns.printInstanceInfo();

		ColumnGeneration colGenMain = new ColumnGeneration(vrptwIns);
		
		double st = System.currentTimeMillis();
		colGenMain.runColumnGeneration();
		double et = System.currentTimeMillis();

		colGenMain.rstMasterProblem.writeSolution(parm.getOutputFileName(), (et-st)/1000);
		
		System.out.println("Time >>> "+(et-st)/1000 + " s");

	}

}
