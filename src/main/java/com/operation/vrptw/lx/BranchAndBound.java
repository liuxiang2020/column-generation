package com.operation.vrptw.lx;


import org.checkerframework.checker.units.qual.A;

import java.io.IOException;
import java.util.ArrayList;

public class BranchAndBound {
    /**
    * 分支定界算法类
    */
    //下界
    public double lowerBound;
    //上界
    public double upperBound;

    public BranchAndBound(double lowerBound, double upperBound) {
        this.lowerBound = lowerBound;
        this.upperBound = upperBound;
    }

    public BranchAndBound() {
        this.lowerBound = -1E10;
        this.upperBound = 1E10;
    }

    static class TreeBB{
        TreeBB father;
        TreeBB leftSon;
        int branchFrom;
        int branchTo;
        int branchValue;
        double lowestValue;
        boolean  topLevel;

        public TreeBB(TreeBB father, TreeBB leftSon, int branchFrom, int branchTo, int branchValue, double lowestValue,
                      boolean topLevel) {
            // todo 确认这样写对不对
            this(father, leftSon, branchFrom, branchTo, branchValue, topLevel);
            this.lowestValue = lowestValue;
        }
        public TreeBB(TreeBB father, TreeBB leftSon, int branchFrom, int branchTo, int branchValue, boolean topLevel) {
            this.father = father;
            this.leftSon = leftSon;
            this.branchFrom = branchFrom;
            this.branchTo = branchTo;
            this.branchValue = branchValue;
            this.topLevel = topLevel;
        }
    }

    public void edgesBasedOnBranching(VRPParam userParam, TreeBB branchTree, boolean recur){
        /**
         * 这是一个递归函数，终止条件为branchTree.father != null
         * TreeBB branchTree 分支树
         * recur: 递归/循环
         */
        // 到达根节点时停止
        if(branchTree.father != null){
            // 禁止使用扯个分支
            if(branchTree.branchValue == 0){
                userParam.dist[branchTree.branchFrom][branchTree.branchTo] = userParam.bigM;
            }else{
                // 选择了这条边，则除了from--to这条边，其他所有起点为from的边都被禁止
                if(branchTree.branchFrom != 0){
                    for(int i=0; i < branchTree.branchTo; i++)
                        userParam.dist[branchTree.branchFrom][i] = userParam.bigM;
                    for(int i=branchTree.branchTo+1; i< userParam.totalNode; i++)
                        userParam.dist[branchTree.branchFrom][i] = userParam.bigM;
                }
                // 选择了这条边，则除了from--to这条边，其他所有终点为to的边都被禁止
                if(branchTree.branchFrom != 0){
                    for(int i=0; i < branchTree.branchFrom; i++)
                        userParam.dist[i][branchTree.branchTo] = userParam.bigM;
                    for(int i=branchTree.branchFrom+1; i<userParam.totalNode; i++)
                        userParam.dist[i][branchTree.branchTo] = userParam.bigM;
                }
                // 禁止反向链接
                userParam.dist[branchTree.branchTo][branchTree.branchFrom] = userParam.bigM;
            }
            // 递归调用直到搜索到根节点，为什么要这样？
            if(recur)
                edgesBasedOnBranching(userParam, branchTree.father, recur);
        }
    }
    //BB branch and bound的意思
    public boolean BBNode(VRPParam userParam, ArrayList<Route> routeList, TreeBB branching,
                                       ArrayList<Route> bestRouteList, int depth) throws IOException{
        /**
         * userParam: all the parameters provided by the users (cities, roads...)
         * routeList: all (but we could decide to keep only a subset) the routes considered up to now (to initialize the Column generation process)
         * branching: BB branching context information for the current node to process (branching edge var, branching value, branching from...)
         * 输出：bestRoutes: best solution encountered
         */

        try{
            if((upperBound-lowerBound)/upperBound<userParam.gap)
                return true;
            //初始化
            if(branching==null){
                //第一次调用：设置根节点
                TreeBB newNode = new TreeBB(null, null, -1, -1, -1, true);
                branching = newNode;
            }
            // 展示部分信息
            if (branching.branchValue < 1)
                System.out.println("Edge from " + branching.branchFrom + " to " + branching.branchTo + ": forbid");
            else
                System.out.println("Edge from " + branching.branchFrom + " to " + branching.branchTo + ": set");

            int mb=1024*1024;
            Runtime runtime = Runtime.getRuntime();
            System.out.print("Java Memory=> Total:" + (runtime.totalMemory() / mb)
                    + " Max:" + (runtime.maxMemory() / mb) + " Used:"
                    + ((runtime.totalMemory() - runtime.freeMemory()) / mb) + " Free: "
                    + runtime.freeMemory() / mb);

            // 使用列生成算法计算该分支的的节点
            ColumnGeneration CG = new ColumnGeneration();
            double CGobj = CG.computeColGen(userParam, routeList);
            if ((CGobj > 2 * userParam.maxLength) || (CGobj < -1e-6)) {
                // can only be true when the routeList in the solution include forbidden edges
                // (can happen when the BB set branching values)
                System.out.println("RELAX INFEASIBLE | Lower bound: " + lowerBound
                        + " | Upper bound: " + upperBound +
                        " | Gap: " + ((upperBound - lowerBound) / upperBound) +
                        " | BB Depth: " + depth + " | " + routeList.size() + " routes");
                // stop this branch
                return true;
            }
            branching.lowestValue = CGobj;
            // 更新下界
            if((branching.father!=null) && (branching.father.leftSon != null) && branching.father.topLevel){
                // all nodes above and on the left have been processed=> we can compute a new lowerBound
                lowerBound = Math.min(branching.lowestValue, branching.father.leftSon.lowestValue);
                branching.topLevel = true;
            }else if(branching.father == null)
                lowerBound = CGobj;
            // 确认使用>=还是>
            if(branching.lowestValue >= upperBound){
                CG = null;
                System.out.println("CUT | Lower bound: " + lowerBound
                        + " | Upper bound: " + upperBound +
                        " | Gap: " + ((upperBound - lowerBound) / upperBound) +
                        " | BB Depth: " + depth + " | Local CG cost: " + CGobj + " | "
                        + routeList.size() + " routes");
                // cut this useless branch
                return true;
            }else{
                //check the (integer) feasibility. Otherwise search for a branching variable

                // transform the path variable (of the CG model) into edges variables
                for (int i = 0; i < userParam.totalNode; i++)
                    java.util.Arrays.fill(userParam.edges[i], 0.0);
                for(Route route: routeList){
                    // we consider only the routes in the current local solution
                    if(route.getQ()>1e-6){
                        // get back the sequence of cities (path for this route)
                        ArrayList<Integer> path = route.getPath();
                        int preCity = 0;
                        for (int city : path) {
                            userParam.edges[preCity][city] = route.getQ();
                            preCity = city;
                        }
                    }
                }
                boolean feasible = true;
                int bestEdge1 = -1;
                int bestEdge2 = -1;
                double  bestObj = -1.0;// 改名字
                int bestVal = 0;
                // 找到值为分数的edge
                for (int i = 0; i < userParam.totalNode; i++) {
                    for (int j = 0; j < userParam.totalNode; j++) {
                        double coef = userParam.edges[i][j];
                        if ((coef > 1e-6) && ((coef < 0.9999999999) || (coef > 1.0000000001))) {
                            // this route has a fractional coefficient in the solution => should we branch on this one?
                            feasible = false;
                            // what if we impose this route in the solution? Q=1
                            // keep the ref of the edge which should lead to the largest change
                            double change = Math.min(coef, Math.abs(1.0 - coef));
                            change *= routeList.get(i).getCost();
                            if (change > bestObj) {
                                bestEdge1 = i;
                                bestEdge2 = j;
                                bestObj = change;  //????
                                bestVal = (Math.abs(1.0 - coef) > coef) ? 0 : 1;
                            }
                        }
                    }
                }
                CG = null;
                if(feasible){
                    // todo怎么确保lowestValue对应的解是整数解
                    if(branching.lowestValue < upperBound){
                        upperBound = branching.lowestValue;
                        bestRouteList.clear();
                        for(Route route: routeList){
                            if(route.getQ()>1e-6){
                                Route optim = new Route();
                                optim.setCost(route.getCost());
                                optim.path = route.getPath();
                                optim.setQ(route.getQ());
                                bestRouteList.add(optim);
                            }
                        }
                        System.out.println("OPT | Lower bound: " + lowerBound
                                + " | Upper bound: " + upperBound +
                                " | Gap: " + ((upperBound - lowerBound) / upperBound) +
                                " | BB Depth: " + depth + " | Local CG cost: " + CGobj + " | "
                                + routeList.size() + " routes");
                        System.out.flush();
                    }else
                        System.out.println("FEAS | Lower bound: " + lowerBound
                                + " | Upper bound: " + upperBound +
                                " | Gap: " + ((upperBound - lowerBound) / upperBound) +
                                " | BB Depth: " + depth + " | Local CG cost: " + CGobj + " | "
                                + routeList.size() + " routes");
                    return feasible;
                }else{
                    System.out.println("INTEG INFEAS | Lower bound: " + lowerBound
                            + " | Upper bound: " + upperBound +
                            " | Gap: " + ((upperBound - lowerBound) / upperBound) +
                            " | BB Depth: " + depth + " | Local CG cost: " + CGobj + " | "
                            + routeList.size() + " routes");
                    System.out.flush();

                    // 往下搜索
                    // first branch -> set edges[bestEdge1][bestEdge2]=0 record the branching information in a tree list
                    // 未知topLevel应该设置为什么值
                    TreeBB newNode1 =new TreeBB(branching, null, bestEdge1, bestEdge2, bestVal, -1E10, true);
                    // branching on edges[bestEdge1][bestEdge2]=0
                    edgesBasedOnBranching(userParam, newNode1, false);

                    // the initial lp for the CG contains all the routes of the previous solution less the routes containing this arc
                    ArrayList<Route> nodeRouteList = new ArrayList<>();
                    // 1)将路径长度大于等于3, 2）路径中不包含 bestEdge1 --> bestEdge2取出来
                    for(Route route: routeList){
                        //路径长度等于3的路径肯定满足要求
                        boolean accept =true;
                        if(route.path.size()>3){
                            int preCity = 0;
                            for(int j=1; j<route.path.size(); j++){
                                int city = route.path.get(j);
                                if(preCity==bestEdge1 && city==bestEdge2){
                                    accept = false;
                                    break;
                                }
                                preCity = city;
                            }
                        }
                        if(accept)
                            nodeRouteList.add(route);
                    }
                    boolean ok = BBNode(userParam, nodeRouteList, newNode1, bestRouteList, depth+1);
                    // 释放内存
                    nodeRouteList = null;
                    if(!ok)
                        return false;
                    branching.leftSon = newNode1;

                    // second branch -> set edges[bestEdge1][bestEdge2]=1
                    // record the branching information in a tree list
                    TreeBB newNode2 = new TreeBB(branching, null, bestEdge1, bestEdge2, 1 - bestVal,  -1E10, true);
                    // branching on edges[bestEdge1][bestEdge2]=1
                    //
                    // second branching=>need to reinitialize the dist matrix
                    for (int i = 0; i < userParam.totalNode; i++)
                        System.arraycopy(userParam.distBase[i], 0, userParam.dist[i], 0, userParam.totalNode);
                    edgesBasedOnBranching(userParam, newNode2, true);
                    // the initial lp for the CG contains all the routes of the previous solution less the routes incompatible with this arc
                    ArrayList<Route> nodeRoutes2 = new ArrayList<>();
                    for (Route route : routeList) {
                        ArrayList<Integer> path = route.getPath();
                        boolean accept = true;
                        if (path.size() > 3) {
                            // we must keep trivial routes Depot-City-Depot in the set to ensure feasibility of the CG
                            int prevCity = 0;
                            for (int i = 1; accept && (i < path.size()); i++) {
                                int city = path.get(i);
                                if (userParam.dist[prevCity][city] >= userParam.bigM - 1E-6)
                                    accept = false;
                                prevCity = city;
                            }
                        }
                        if (accept)
                            nodeRoutes2.add(route);
                    }
                    ok = BBNode(userParam, nodeRoutes2, newNode2, bestRouteList, depth + 1);
                    nodeRoutes2 = null;
                    // update lowest feasible value of this node
                    branching.lowestValue = Math.min(newNode1.lowestValue, newNode2.lowestValue);
                    return ok;
                }
            }

        } catch (IOException e) {
            System.err.println("Error: " + e);
        }
        return false;
    }
}
