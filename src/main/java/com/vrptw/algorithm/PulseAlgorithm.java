package com.vrptw.algorithm;

import com.vrptw.algorithm.element.Path;
import com.vrptw.algorithm.element.espprc.ESPEdge;
import com.vrptw.algorithm.element.espprc.ESPNode;
import com.vrptw.algorithm.element.espprc.ESPPRC;
import lombok.extern.slf4j.Slf4j;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Map;

@Slf4j
public class PulseAlgorithm {
	public ESPPRC espprcIns;
	private double primalBound;		// Primal bound updated through the execution of the algorithm
	private double naiveDualBound;	// Naive dual bound 
	private double overallBestCost;	// Overall best cost found at each iteration of the bounding stage
	private double timeIncumbent;		// Time incumbent for the bounding stage
	private int timeStep;
	
	//algo setting
	private int threadNr;  // thread:线程
	private Thread[] threads;		// Threads

	public PulseAlgorithm(){
		
	}
	
	public PulseAlgorithm(ESPPRC espprc){
		espprcIns            = espprc;
		primalBound          = 0;
		naiveDualBound       = Integer.MAX_VALUE;
		overallBestCost      = 0;
		timeStep             = espprc.vrptwInstance.getParameters().boundStep;
		this.threadNr        = espprc.vrptwInstance.getParameters().numThreads;
		this.threads         = new Thread[threadNr];
		
		for(int i = 0; i <threadNr;++i ) {
			this.threads[i] = new Thread(); //线程
		}
	}

	/**
	 * 每次计算前都应先更新ESPPRC中的cost，该问题求出最终的结果是reduced cost
	 * @param rmpDualValues 限制主问题的对偶变量
	 */
	public void updateESPPRCCost(Map<Integer,Double> rmpDualValues) {
		for(int customerIndex : rmpDualValues.keySet()) {
			// sum_(j\in J)x_ij = y_i
			for(ESPEdge outgoingEdge : espprcIns.getIndexEspNodeMap().get(customerIndex).outgoingEdges) {
				outgoingEdge.cost = outgoingEdge.dist - rmpDualValues.get(customerIndex); // 在这个案例中距离就是成本
				espprcIns.costMatrix[outgoingEdge.getStartNode().getNumber()][outgoingEdge.getEndNode().getNumber()] = outgoingEdge.cost;
			}
		}
	}

	/**
	 * 计算主程序
	 * step 1 : 初始化：1）更新边的成本；2）初始化累计距离，容量，时间
	 * step 2 : boundingScheme
	 * step 3 : pulse
	 * step 4: 整理结果
	 *
	 * @param rmpDualValues 限制主问题的对偶变量值
	 * @return
	 * @throws InterruptedException
	 */
	public Path runPulseAlgo(Map<Integer,Double> rmpDualValues) throws InterruptedException {
		//first update
		log.info("--------------------------step 1: 初始化：1）更新边的成本；2）初始化累计距离，容量，时间--------------------------");
		updateESPPRCCost(rmpDualValues);
		espprcIns.getEspFinalNode().resetFinalSol();

		// 初始化timeIncumbent的值：需要为timeStep的整数倍且不小于espprcIns.vrptwInstance.getDepot().getEndTw()
		this.timeIncumbent = espprcIns.vrptwInstance.getDepot().getEndTw();
		this.timeIncumbent += timeStep;
		this.timeIncumbent -= this.timeIncumbent % timeStep;
		log.info("this.timeIncumbent = "+this.timeIncumbent);

		log.info("--------------------------step 2: 使用动态规划原理计算每个节点到终点的最小成本-----------------------------------");
		this.boundingScheme();
		
		log.info("--------------------------step 3: 计算从起点到终点的最小成本线路-----------------------------------------------");
		this.timeIncumbent += this.timeStep; 	// Set time incumbent to the last value solved
		this.primalBound=0;			            // Reset the primal bound
		this.pulseMT(espprcIns.getEspStartNode(), 0, 0, 0, new ArrayList<Integer>(),0,0);

		log.info("-----------------------------------step 4: 打印并整理结果----------------------------------------------------");
		this.printSolution();
		Path optPath = new Path();
		optPath.getPath().addAll(espprcIns.getEspFinalNode().path);
		optPath.setTravalTime(espprcIns.getEspFinalNode().pathCost);
		return optPath;
	}

	/**
	 * 基于动态规划，计算每个节点到终点每个时间片段的最小费用，返回结果为矩阵[node][timeStep]
	 */
	public void boundingScheme() {
		calNaiveDualBound();
		// Lower time (resource) limit to stop the bounding procedure. For 100-series we used 50 and for 200-series we used 100;
		int lowerTimeLimit = 100;
		int timeIndex = 0;
		
		while(timeIncumbent >= lowerTimeLimit){
			timeIndex = (int) Math.ceil(timeIncumbent/timeStep);
			for(ESPNode espNode : this.espprcIns.getIndexEspNodeMap().values()) {
				pulseBound(espNode, 0, timeIncumbent, 0 , new ArrayList<Integer>(), espNode,0);
				espNode.boundsMap.put(timeIndex, espNode.bestCost);
				//System.out.println("boundsMatrix["+espno.getNumber()+"]["+timeIndex+"] = "+espno.bestCost);
			}

			this.overallBestCost = this.primalBound;		// Store the best cost found over all the nodes
			timeIncumbent -= timeStep;						// Update the time incumbent
		}
	}

	/**
	 * 计算单位时间消耗的最小费用
	 */
	public void  calNaiveDualBound() {
		log.info("finds the best arc regarding the cost/time ratio(naiveDualBound)");
		this.naiveDualBound = Integer.MAX_VALUE;
		for(ESPNode espNode : this.espprcIns.getIndexEspNodeMap().values()) {
			for(ESPEdge ogEdge : espNode.outgoingEdges) {
				if((ogEdge.time != 0) && (ogEdge.cost / ogEdge.time < this.naiveDualBound)) {
					this.naiveDualBound = ogEdge.cost / ogEdge.time;
				}
			}
		}
		log.info("naiveDualBound = " + naiveDualBound);
	}
	
	
	/**
	 * 使用回溯原理（深度优先）
	 * Pulse function for the bounding stage
	 * @param pathLoad current path load
	 * @param pathTime current path time
	 * @param pathCost current path cost
	 * @param path current path
	 * @param rootEspNode current root node 当前路径根节点
	 * @param pathDist current path distance
	 */
	@SuppressWarnings("unchecked")
	public void pulseBound(ESPNode currentEspNode, double pathLoad, double pathTime, double pathCost, ArrayList<Integer> path, ESPNode rootEspNode, double pathDist) {
		if(currentEspNode.isFirstTimeVisited){
			currentEspNode.isFirstTimeVisited =false;
			Collections.sort(currentEspNode.outgoingEdges); // 根据边的 cost 排序
		}
		
		// If the node is reached before the lower time window wait until the beginning of the time window
		if(pathTime < currentEspNode.getStartTw()){
			pathTime = currentEspNode.getStartTw();
		}

		// Try to prune pulses with the pruning strategies: cycles, infeasibility, bounds, and rollback
		/*重点看判断函数： 1)未访问；2)满足时间窗；3)成本比rootNode的成本低；4）不需要回退*/
		if(!currentEspNode.visited
				&& pathTime <= currentEspNode.getEndTw()
				&& (pathCost+calcBoundPhaseI(currentEspNode, pathTime, rootEspNode)) < rootEspNode.bestCost
				&& !rollback(currentEspNode, path, pathCost, pathTime)){

			currentEspNode.visited = true;   // 防止成环
			path.add(currentEspNode.getNumber());

			for (ESPEdge espEdge : currentEspNode.outgoingEdges) {
				ESPNode arcHead = espEdge.getEndNode();
				double newPathTime = (pathTime + espEdge.time);
				double newPathCost = (pathCost + espEdge.cost);
				double newPathLoad = (pathLoad + espEdge.load);
				double newPathDist = (pathDist + espEdge.dist);

				if (newPathTime <= arcHead.getEndTw() && newPathLoad <= espprcIns.vrptwInstance.getVehiclesCapacity()) {
					if(arcHead.getNumber() == espprcIns.getEspFinalNode().getNumber()){
						pulseBoundFinal(newPathLoad, newPathTime, newPathCost, path, rootEspNode ,newPathDist);
					} else{
						pulseBound(arcHead, newPathLoad, newPathTime, newPathCost, path, rootEspNode ,newPathDist);
					}
				}
			}
			// 还原到加入当前节点的状态
			path.remove((path.size() - 1));
			currentEspNode.visited = false;
		}
	}

	/** Multithread pulse function 
	 * @param pLoad current load
	 * @param pTime current time
	 * @param pCost current cost
	 * @param path current partial path
	 * @param pDist current distance
	 * @param thread current thread 
	 * @throws InterruptedException
	 */
	@SuppressWarnings("unchecked")
	public void pulseMT(ESPNode node, double pLoad, double pTime, double pCost, ArrayList<Integer> path, double pDist, int thread) throws InterruptedException {

		if(node.isFirstTimeVisited){
			node.isFirstTimeVisited =false;
			Collections.sort(node.outgoingEdges);
		}

		if(pTime < node.getStartTw()){
			pTime = node.getStartTw();
		}
		// Try to prune pulses with the pruning strategies
		if((!node.visitedMT[thread]
				&& (pCost+ calcBoundPhaseII(node, pTime)) < primalBound
				&& !rollback(node, path,pCost,pTime))){

			node.visitedMT[thread] = true;
			path.add(node.getNumber());

			// Propagate the pulse through all the outgoing arcs
			for(ESPEdge espEdge : node.outgoingEdges){
				ESPNode arcHead = espEdge.getEndNode();
				double newPathTime=(pTime+espEdge.time);
				double newPathCost=(pCost+espEdge.cost);
				double newPathLoad=(pLoad+espEdge.load);
				double newPathDist=(pDist+espEdge.dist);
				
				//System.out.println("newPathTime ="+newPathTime+" newPathLoad = "+newPathLoad);
				if( newPathTime <= arcHead.getEndTw() && newPathLoad <= espprcIns.getCapacity()){
					if (arcHead.getNumber() == espprcIns.getEspFinalNode().getNumber()) {
						pulseMTFinal(newPathLoad,newPathTime,newPathCost, path, newPathDist,thread);
					}else{
						// If not in the start node continue the exploration on the current thread,
						// If standing in the start node, wait for the next available thread to trigger the exploration
						if(node.getNumber() != espprcIns.getEspStartNode().getNumber()){
							pulseMT(arcHead, newPathLoad,newPathTime,newPathCost, path, newPathDist, thread);
						} else {
							initTread(arcHead, newPathLoad, newPathTime, newPathCost, path, newPathDist);
						}
					}
				}
			}

			// Wait for all active threads to finish
			if(node.getNumber() == espprcIns.getEspStartNode().getNumber()){
				for (int k = 1; k < this.threads.length; k++) {
					this.threads[k].join();
				}
			}
			// Remove the explored node from the path
			path.remove((path.size()-1));
			node.visitedMT[thread] = false ;
		}
	}
	
	public class PulseTask implements Runnable{
		private double pathLoad;
		private double pathTime;
		private double pathCost;
		private ArrayList<Integer> path;
		private double pathDist;
		private int thread;
		private ESPNode head;

		public PulseTask(ESPNode head, double pathLoad, double pathTime, double pathCost, ArrayList<Integer> path, double pathDist, int thread) {
			this.pathLoad = pathLoad;
			this.pathTime = pathTime;
			this.pathCost = pathCost;
			this.path   = new ArrayList<>();
			this.pathDist = pathDist;
			this.thread = thread;
			this.head   = head;
			this.path.addAll(path);
		}
		@Override
		public void run() {
			try {
				pulseMT(head, pathLoad, pathTime, pathCost, path, pathDist, thread);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
	}


	/** Rollback pruning strategy
	 * 判断从a--c是不是比a--b--c更便宜
	 *          b
	 *        /  \
	 *       /     \
	 * ----a--------c(currentNode)
	 * 由于运输距离和运输时间满足三角不等式，所以检测的内容更少？
	 * @param path current partial path
	 * @param pathCost current cost
	 * @param pathTime current time
	 * @return
	 */
	private boolean rollback(ESPNode currentNode, ArrayList<Integer> path, double pathCost, double pathTime) {

		if(path.size() <= 1){
			return false;
		}
		//倒数第一个节点
		int prevNode = path.get(path.size()-1);
		//倒数第二个节点
		int directNode = path.get(path.size()-2);

		double directCost = pathCost - espprcIns.costMatrix[prevNode][currentNode.getNumber()]-espprcIns.costMatrix[directNode][prevNode]+espprcIns.costMatrix[directNode][currentNode.getNumber()];

		return directCost <= pathCost;
	}


	/**
	 * 计算从currentEspNode到终点的最多可节约成本，这个功能主要用于剪枝
	 * This method calculates a lower bound given a time consumption at a given node
	 * If the time consumed is less than the last time incumbent solved and the node id is larger than the current pathRootEspNode node being explored
	 * it means that there is no lower bound available and we must use the naive bound
	* @param pathTime current time
	* @param pathRootEspNode current pathRootEspNode node
	* @return
	*/
	private double calcBoundPhaseI(ESPNode currentEspNode, double pathTime, ESPNode pathRootEspNode) {
		// todo 不理解为什么需要currentEspNode.getNumber() >= pathRootEspNode.getNumber()
		if(pathTime < this.timeIncumbent + this.timeStep && currentEspNode.getNumber() >= pathRootEspNode.getNumber()){
			// 我觉得this.overallBestCost应该改为currentEspNode.boundsMap.get( (this.timeIncumbent+this.timeStep)/ this.timeStep )
			return (this.timeIncumbent+this.timeStep - pathTime)*this.naiveDualBound + this.overallBestCost;
		} else {
			int index = ((int) Math.floor(pathTime / this.timeStep));
			return currentEspNode.boundsMap.get(index);
		}
	}

	/** This method calculates a lower bound given a time consumption at a given node
	 *
	 * If the time consumed is less than the current time incumbent it means that there is no lower bound available and we must use the naive bound
	 *
	* @param pathTime current time
	* @return
	*/
	private double calcBoundPhaseII(ESPNode node, double pathTime) {
		if(pathTime < this.timeIncumbent){
			return (this.timeIncumbent-pathTime)*this.naiveDualBound+this.overallBestCost;
		} else {
			int Index=((int) Math.floor(pathTime/this.timeStep));
			return node.boundsMap.get(Index);
		}
	}

	/**
	 * Override for the bounding procedure
	 * @param pathLoad
	 * @param pathTime
	 * @param pathCost
	 * @param path
	 * @param pathRootEspNode
	 * @param pathDist
	 */
	public void pulseBoundFinal(double pathLoad, double pathTime, double pathCost, ArrayList<Integer> path, ESPNode pathRootEspNode, double pathDist) {
		// If the path is feasible update values for the bounding matrix and primal bound
		if (pathLoad <= espprcIns.getCapacity() && pathTime <= espprcIns.getEspFinalNode().getEndTw()) {
			if (pathCost < pathRootEspNode.bestCost) {
				pathRootEspNode.bestCost = pathCost;
				if (pathCost < primalBound) {
					primalBound = pathCost;
				}
			}
		}
	}

	/**
	 * Final node for the pulse procedure
	 * @param pLoad
	 * @param pTime
	 * @param pCost
	 * @param path
	 * @param pDist
	 * @param thread
	 */
	public void pulseMTFinal(double pLoad, double pTime, double pCost, ArrayList<Integer> path, double pDist, int thread) {
		//System.out.println("final: pLoad = "+pLoad+" pTime = "+pTime+" pCost = "+pCost);

		if (pLoad <= espprcIns.getCapacity() && (pTime) <= espprcIns.getEspFinalNode().getEndTw()) {
			if (pCost <= primalBound) {
				primalBound = pCost;
				espprcIns.getEspFinalNode().pathTime = pTime;
				espprcIns.getEspFinalNode().pathCost = pCost;
				espprcIns.getEspFinalNode().pathLoad = pLoad;
				espprcIns.getEspFinalNode().pathDist = pDist;
				espprcIns.getEspFinalNode().setPath(new ArrayList<Integer>(path));
				//espprcIns.getEspFinalNode().path.add(espprcIns.getEspFinalNode().getNumber());
			}
		}
	}

	public void printSolution(){
		System.out.println("************ESPPRC OPTIMAL SOLUTION *****************\n");
		System.out.println("Optimal cost: "+espprcIns.getEspFinalNode().pathCost);
		System.out.println("Optimal time: "+espprcIns.getEspFinalNode().pathTime);
		System.out.println("Optimal Load: "+espprcIns.getEspFinalNode().pathLoad);
		System.out.println();
		System.out.println("Optimal path: ");
		System.out.println(espprcIns.getEspFinalNode().path);
		System.out.println("******************ESPPRC END************************\n");
	}

	/**
	 * 初始化多线程
	 * @param arcHead
	 * @param newPathLoad
	 * @param newPathTime
	 * @param newPathCost
	 * @param path
	 * @param newPathDist
	 * @throws InterruptedException
	 */
	public void initTread(ESPNode arcHead, double newPathLoad, double newPathTime, double newPathCost, ArrayList<Integer> path, double newPathDist) throws InterruptedException {
		boolean stopLooking = false;
		for (int j = 1; j < this.threads.length; j++) {
			if(!this.threads[j].isAlive()){
				this.threads[j] = new Thread(new PulseTask(arcHead, newPathLoad, newPathTime, newPathCost, path, newPathDist, j));
				this.threads[j].start();
				stopLooking = true;
				//j = 1000;
				break;
			}
		}
		if (!stopLooking) {
			this.threads[1].join();
			this.threads[1] = new Thread(new PulseTask(arcHead, newPathLoad, newPathTime, newPathCost, path, newPathDist, 1));
			this.threads[1].start();
		}
	}
}
