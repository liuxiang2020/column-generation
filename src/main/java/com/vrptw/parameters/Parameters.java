package com.vrptw.parameters;

import java.util.Random;


import com.vrptw.algorithm.RestrictedMasterProblem;
import ilog.concert.IloException;
import ilog.cplex.IloCplex;
import lombok.Data;

@Data
public class Parameters {
	private String inputFileName;
	private String outputFileName;
	private String instanceName;
	private double precision;
	private int noImproveIterations;
	private int startClient;
	private int randomSeed;
	private String currDir = System.getProperty("user.dir")+"/src/main/resources/";;
	private Random random;
	private double alpha;
	private double beta;
	
	///////column generation
	public boolean abort;
	public double zero_reduced_cost;
	public double zero_reduced_cost_AbortColGen;

	///////espprc
	public int numThreads;		// Number of threads
	public int boundStep;		// Step size for the bounding procedure

	
	public Parameters(String fileName) {
		//defult output
		outputFileName    	= currDir + "result/solution.csv";
		inputFileName       = currDir + fileName;
		//defult configure
		precision         	= 1E-4;
		noImproveIterations = 1;
		startClient       	= 0;
		randomSeed		  	= 10086;
		random = new Random(); 
		random.setSeed(randomSeed);
		alpha = 1;
		beta = 1;
		
		abort = false;
		zero_reduced_cost = -0.0001;
		zero_reduced_cost_AbortColGen = -0.005;
		
		numThreads  = 20;
		boundStep   = 4;

	}
	
	public void updateParameters(String[] args)
	{
		if(args.length % 2 == 0){
			for(int i = 0; i < args.length; i += 2){
				switch (args[i]) {
					case "-in":
						inputFileName= args[i+1];
						String []tS1 = args[i+1].split("\\\\");
						String []tS2 = tS1[tS1.length-1].split("\\.");
						instanceName = tS2[0];
						outputFileName = currDir + "\\output\\" + instanceName+"_solution.txt";
						break;
					case "-out":
						outputFileName = args[i+1];
						if(args[i+1].charAt(args[i+1].length()-1) != '\\') {
							outputFileName = outputFileName+"\\"+instanceName+"_solution.txt";
						}
						else {
							outputFileName = outputFileName+instanceName+"_solution.txt";
						}						
						break;
					case "-precision":
						precision = Double.parseDouble(args[i+1]);
						break;
					case "-iteration":
						noImproveIterations = Integer.parseInt(args[i+1]);
						break;
					case "-startClient":
						startClient = Integer.parseInt(args[i+1]);
						break;
					case "-randomSeed":
						randomSeed = Integer.parseInt(args[i+1]);
						this.random.setSeed(randomSeed);
						break;
					case "-alpha":
						alpha = Double.parseDouble(args[i+1]);
						break;
					case "-beta":
						beta = Double.parseDouble(args[i+1]);
						break;
					default: {
						System.out.println("Unknown type of argument: " + args[i]);
						System.exit(-1);
					}
				}
			}
		}else {
			System.out.println("Parameters are not in correct format");
			System.exit(-1);
		}
	}
	
	public String toString(){
		String print = "\n" + "--- Parameters: -------------------------------------" +
				"\n" + "| Input File Name       = " + inputFileName +
				"\n" + "| Output File Name      = " + outputFileName +
				"\n" + "| Precision             = " + precision +
				"\n" + "| Iterations            = " + noImproveIterations +
				"\n" + "| Start Client          = " + startClient +
				"\n" + "| Random Seed           = " + randomSeed +
				"\n" + "| Alpha                 = " + alpha +
				"\n" + "| Beta                  = " + beta +
				"\n" + "------------------------------------------------------";
		return print;
	}
	
	public void cplexConfigure(RestrictedMasterProblem masterproblem) {
		try {
			// branch and bound
			masterproblem.cplex.setParam(IloCplex.Param.MIP.Strategy.NodeSelect, 1);
			masterproblem.cplex.setParam(IloCplex.Param.MIP.Strategy.Branch,1);
			//masterproblem.cplex.setParam(IloCplex.Param.Preprocessing.Presolve, true);
			// display options
			masterproblem.cplex.setParam(IloCplex.Param.MIP.Display, 2);
			masterproblem.cplex.setParam(IloCplex.Param.Tune.Display, 1);
			masterproblem.cplex.setParam(IloCplex.Param.Simplex.Display, 0);
		}catch (IloException e) {
			System.err.println("Concert exception caught: " + e);
		}
	}
	
}
