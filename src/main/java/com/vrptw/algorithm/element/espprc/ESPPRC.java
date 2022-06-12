package com.vrptw.algorithm.element.espprc;

import com.vrptw.instance.Node;
import com.vrptw.instance.VrptwInstance;
import lombok.Data;

import java.util.HashMap;

@Data
public class ESPPRC {
	
	public VrptwInstance vrptwInstance;
	private HashMap<Integer, ESPNode> indexEspNodeMap;// Array of nodes
	private int nodesNr;
	private ESPFinalNode espFinalNode;		// The final node overrides the class node and is different because it stops the recursion
	// 路径最大装载量
	private double capacity;
	private ESPNode espStartNode;
	public double[][] costMatrix;
	public double[][] distanceMatrix;
	
	public ESPPRC() {
		
	}
	// Class constructor
	public ESPPRC(VrptwInstance vrpIns) {
		this.vrptwInstance   = vrpIns;
		this.nodesNr         = vrpIns.getCustomersNr()+1;
		this.indexEspNodeMap = new HashMap<Integer, ESPNode>();
		this.espFinalNode    = new ESPFinalNode(vrpIns.getNodes().get(vrpIns.getDepot().getNumber()));
//		this.espStartNode    = new ESPNode(vrpIns.getNodes().get(vrpIns.getDepot().getNumber()));

		this.capacity        = vrpIns.getVehiclesCapacity();
		this.distanceMatrix = vrpIns.getDistances();
		this.costMatrix = new double[nodesNr][nodesNr];
		initESPPRCDate();
	}
	
	public void initESPPRCDate() {
		int arc = 0;
		//add esp nodes
		for(Node val : vrptwInstance.getNodes().values()) {
			ESPNode espno = new ESPNode(val);
			espno.visitedMT = new boolean[vrptwInstance.getParameters().numThreads];
			indexEspNodeMap.put(espno.getNumber(), espno);
		}
		
		//add esp edges
		for(ESPNode espi : getIndexEspNodeMap().values()) {
			for(ESPNode espj : getIndexEspNodeMap().values()) {
				if( (espi.getNumber() != espj.getNumber()) && 
					(espi.getStartTw()+espi.getServiceDuration() + 
					vrptwInstance.getTravelTime(espi.getNumber(), espj.getNumber()) < espj.getEndTw()) ) {
					ESPEdge espe = new ESPEdge(arc, espi, espj);
					espe.dist = vrptwInstance.getTravelTime(espi.getNumber(), espj.getNumber());
					espe.load = espj.getDemand();
					espe.time = espi.getServiceDuration() + espe.dist;
					espe.cost = espe.dist;
					//cost will be update later ... 
					espi.outgoingEdges.add(espe);
					
					costMatrix[espe.getStartNode().getNumber()][espe.getEndNode().getNumber()] = espe.dist;
					
					arc++;
				}
			}
		}
		this.espStartNode = indexEspNodeMap.get(vrptwInstance.getDepot().getNumber());
		this.espStartNode.visitedMT = new boolean[vrptwInstance.getParameters().numThreads];
	}
	
	public int getNodesNr() {
		return nodesNr;
	}
	public void setNodesNr(int nodesNr) {
		this.nodesNr = nodesNr;
	}
	public HashMap<Integer, ESPNode> getIndexEspNodeMap() {
		return indexEspNodeMap;
	}
	public void setIndexEspNodeMap(HashMap<Integer, ESPNode> indexEspNodeMap) {
		this.indexEspNodeMap = indexEspNodeMap;
	}
	public ESPFinalNode getEspFinalNode() {
		return espFinalNode;
	}
	public void setEspFinalNode(ESPFinalNode espFinalNode) {
		this.espFinalNode = espFinalNode;
	}
	public double getCapacity() {
		return capacity;
	}
	public void setCapacity(double capacity) {
		this.capacity = capacity;
	}
	public ESPNode getEspStartNode() {
		return espStartNode;
	}
	public void setEspStartNode(ESPNode espStartNode) {
		this.espStartNode = espStartNode;
	}

}
