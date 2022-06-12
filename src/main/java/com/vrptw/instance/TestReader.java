package com.vrptw.instance;


import com.vrptw.parameters.Parameters;

public class TestReader {

	public static void main(String[] args) {
		// TODO Auto-generated method stub
		String fileName = "C101.txt";
		Parameters parm = new Parameters(fileName);
		parm.updateParameters(args);
		
		VrptwInstance vrptwIns = new VrptwInstance();
		vrptwIns.readInstanceFromFile(parm.getInputFileName());
		
		System.out.println(parm.toString());
		
		System.out.println(vrptwIns.customersToString());
		System.out.println(vrptwIns.distancesToString());

		
	}

}
