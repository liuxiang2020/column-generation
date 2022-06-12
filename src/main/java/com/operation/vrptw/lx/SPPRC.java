package com.operation.vrptw.lx;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.TreeSet;
// shortest path with resource constraints
// inspired by Irnish and Desaulniers, "SHORTEST PATH PROBLEMS WITH RESOURCE CONSTRAINTS"
// for educational demonstration only - (nearly) no code optimization
//
// four main lists will be used:
// labelList: array (ArraList) => one dimensional unbounded vector
//		 list of all labels created along the feasible paths (i.e. paths satisfying the resource constraints)
//
// U: sorted list (TreeSet) => one dimensional unbounded vector
//		sorted list containing the indices of the unprocessed labels (paths that can be extended to obtain a longer feasible path)
//
// P: sorted list (TreeSet) => one dimensional unbounded vector
//		sorted list containing the indices of the processed labels ending at the depot with a negative cost
//
// city2labels: matrix (array of ArrayList) => nbclients x unbounded
//		for each city, the list of (indices of the) labels attached to this city/vertex
//		before processing a label at vertex i, we compare pairwise all labels at the same vertex to remove the dominated ones


// 当子问题为SPPRC时，使用labeling algorithm求解
public class SPPRC {
    VRPParam userParam;
    public ArrayList<Label> labelList;

    //标签类
    static class Label {
        // 当前节点
        public int city;
        // 当前路径下的前一个label的index
        public int indexPreLabel;
        // 当前路径到达该点的总成本
        public double cost;
        // 该路径到达该点花费的时间
        public float ttime;
        // 该路径到达该点，已经满足的需求
        public double demand;
        //是否被其他路线dominated
        public boolean dominated;
        //当前路径是否访问了各节点
        public boolean[] vertexVisitied;

        public Label(int city, int indexPreLabel, double cost, float ttime, double demand, boolean dominated, boolean[] vertexVistied) {
            this.city = city;
            this.indexPreLabel = indexPreLabel;
            this.cost = cost;
            this.ttime = ttime;
            this.demand = demand;
            this.dominated = dominated;
            this.vertexVisitied = vertexVistied;
        }
    }
    //dominance准则，在这个类中未使用
    class LabelComparater implements Comparator<Integer> {

        public int compare(Integer a, Integer b){
            Label A = labelList.get(a);
            Label B = labelList.get(b);

            if(A.cost-B.cost < -1e-7){
                return -1;
            }else if(A.cost-B.cost>1e-7){
                return 1;
            }else{
                if(A.city == B.city){
                    if(A.ttime-B.ttime<-1e-7){
                        return -1;
                    }else if(A.ttime-B.ttime > 1e-7){
                        return 1;
                    }else if(A.demand - B.demand<-1e-7){
                        return -1;
                    }else if(A.demand - B.demand > 1e-7){
                        return 1;
                    }else{
                        // 至此，各元素均相等了
                        int i=0;
                        while(i < userParam.nbClients + 2){
                            if(A.vertexVisitied[i] != B.vertexVisitied[i]){
                                if (A.vertexVisitied[i]){
                                    return -1;
                                }else{
                                    return 1;
                                }
                            }
                            i++;
                        }
                        return 0;
                    }
                }else if(A.city > B.city){
                    return 1;
                }else{
                    return -1;
                }
            }
        }
    }

    public void labelingAlgorithm(VRPParam userParamArg, ArrayList<Route> routes, int nbRoute){
        /**
         * 主函数：使用labeling算法求解SPPRC问题
         * userParamArg:问题中的数据
         * routes：路径
         * nbRoute: 输出的最大路径数量
         * nb:number
         */

        this.userParam = userParamArg;
        int nbVertex = userParamArg.nbClients +2;
        // 未处理的标签，已经处理的标签
        TreeSet<Integer> U = new TreeSet<>(new LabelComparater());
        // processed labels list
        TreeSet<Integer> P = new TreeSet<>(new LabelComparater());
        //初始化labels
        labelList = new ArrayList<>(2*nbRoute);
        //当前label是否已经访问了该节点
        boolean[] vertexVisited = new boolean[nbVertex];
        vertexVisited[0] = true;
        for (int i = 1; i < nbVertex; i++) {
            vertexVisited[i] = false;
        }
        // 第一个标签是从起点开始
        labelList.add(new Label(0, -1, 0.0, 0, 0, false, vertexVisited));
        U.add(0);

        // for each city, an array with the index of the corresponding labels (for dominance)
        int[] checkDom = new int[nbVertex];
        //每个城市对应的label序号
        ArrayList<Integer>[] city2labels = new ArrayList[nbVertex];
        for(int i=0; i<nbVertex; i++){
            city2labels[i] = new ArrayList<Integer>();
            checkDom[i] = 0;
        }
        city2labels[0].add(0);
        int currentSolutionNum = 0;
        int maxSolutionNum = 2*nbRoute;

        // 生成label主程序
        while((U.size()>0) && (currentSolutionNum < maxSolutionNum) ){
            // second term if we want to limit to the first solutions encountered to speed up the SPPRC (perhaps not the BP)
            // remark: we'll keep only nbRoute, but we compute 2xnbroute!  It makes a huge difference=>we'll keep the most negative ones
            // this is something to analyze further!  how many solutions to keep and which ones?

            // 从U中取出一个标签判断与其他标签的dominated关系，并将其从U中删除process one label => get the index AND remove it from U
            Integer currentIndex = U.pollFirst();
            Label currentLabel = labelList.get(currentIndex);
            int cityIndex = currentLabel.city;
            // check for dominance，code not fully optimized:
            boolean pathDominated;
            // 记录被删除的label
            ArrayList<Integer> cleaning = new ArrayList<>();

            // 判断两个index之间的dominance关系
            // 由于在上一轮循环中，已经判断了checkDom[cityIndex]个标签，两两之间的dominance关系，
            // 因此这里只需要判断在上一轮循环中增加的标签与所有标签之间的dominance关系（只需要比较new与(old, new)之间的关系），其实还可以进一步优化
            // 1）dominated是否具有传递性：回答否：如以最大化为例 A(2, 4), B(3,2)， C(4, 3); A和B互不dominated,A和C互不dominated，然而C dominated B
            // todo 2）理论上两个label是否可能完全一样：回答，如果允许存在圈，也不可能吧
            for(int i=checkDom[cityIndex]; i<city2labels[cityIndex].size(); i++){
                int l1 = city2labels[cityIndex].get(i);
                Label label1 = labelList.get(l1);
                for(int j=0; j<i; j++){
                    int l2 = city2labels[cityIndex].get(j);
                    Label label2 = labelList.get(l2);
                    // 只有两个都不被其他labeldominated，才有必要继续比较，因为每个city只需要保留非dominated的标签
                    // 可能发生这种情况，因为cleaning中的标签是在双重循环结束后，才从city2labels删除的。
                    if(!(label1.dominated || label2.dominated)){

                        //判断label1是否 dominated label2
                        pathDominated = true;

                        // 一旦pathDominated为false就跳出来
                        // 为true： label1通过的节点是label的子集，
                        // 1)路径label1未通过vertexVisitied[k], 无论label2是否通过
                        // 2)路径label1通过vertexVisitied[k]， label2也通过了vertexVisitied[k],
                        // 为false，退出，label1通过了vertexVisitied[k]，且label2未通过vertexVisitied[k]
                        for(int k=1; pathDominated && (k < nbVertex); k++){  //0为depot
                            pathDominated = (!label1.vertexVisitied[k] || label2.vertexVisitied[k]);
                        }
                        if(pathDominated && (label1.cost <= label2.cost) && (label1.ttime <= label2.ttime) && (label1.demand <= label2.demand)){
                            labelList.get(l2).dominated = true;
                            U.remove((Integer) l2);
                            cleaning.add(l2);
                            System.out.print(" ###Remove"+l2);
                        }
                        //判断label2是否 dominated label1
                        pathDominated = true; // todo 这一行可能是多余的，需要做实验
                        for(int k=1; pathDominated && (k < nbVertex); k++){
                            pathDominated = (!label2.vertexVisitied[k] || label1.vertexVisitied[k]);
                        }
                        if(pathDominated && (label2.cost <= label1.cost) && (label2.ttime <= label1.ttime) && (label2.demand <= label1.demand)){
                            labelList.get(l1).dominated = true;
                            U.remove((Integer) l1);
                            cleaning.add(l1);
                            System.out.print(" ###Remove"+l1);
                            break;
                        }
                    }
                }
            }
            //从city2labels删除明确被dominated的label, 并初始化cleaning
            for(Integer c: cleaning){
                city2labels[cityIndex].remove(c);
            }
            // 更新city2labels[cityIndex]中已经检查过的标签的数量，以加快速度，见注释157-158；下一步骤将往更新city2labels中增加一些label
            checkDom[cityIndex] = city2labels[cityIndex].size();

            // expand REF：生成新的标签，只有当当前label不被其他标签dominated，扩展它才是有意义的
            if(!currentLabel.dominated){
                System.out.println("Label "+currentLabel.city+" preLabel="+currentLabel.indexPreLabel+"cost= "+currentLabel.cost+"ttimie="+currentLabel.ttime+"dominated="+currentLabel.dominated);
                // 如果拓展到了终点
                if(cityIndex == nbVertex-1){
                    // 注意此处的cost是cost-\pi*xx
                    if(currentLabel.cost < -1e-7){
                        P.add(currentIndex);
                        currentSolutionNum = 0;
                        // todo 为什么不更新新增的完整路径，前面有些地方可能会更新一些完整路径的dominated属性，那么是否也可以删除呢？
                        for(Integer complitedlLabel: P){
                            if(!labelList.get(complitedlLabel).dominated){
                                currentSolutionNum += 1;
                            }
                        }
                    }
                //如果没有拓展到终点
                }else{
                    for(int i=0; i < nbVertex; i++){
                        if((!currentLabel.vertexVisitied[i]) && userParam.dist[cityIndex][i] < userParam.bigM-1e-6){
                            //更新到达时间：tt(到达下一节点的时间)=当前节点到达时间+服务时长+运输时长
                            float tt = (float)(currentLabel.ttime+userParam.serviceTimeLength[cityIndex]+userParam.ttime[currentIndex][i]);
                            tt = tt<userParam.openTime[i] ? tt : userParam.openTime[i];
                            //更新载重量
                            double request1 = currentLabel.demand + userParam.request[i];

                            if((tt<=userParam.closeTime[i]) && request1<=userParam.capacity){
                                int idx = labelList.size();
                                boolean[] newVertexVisited = new boolean[nbVertex];
                                System.arraycopy(currentLabel.vertexVisitied, 0, newVertexVisited, 0, nbVertex);
                                newVertexVisited[i] = true;
                                //加速策略: 第三个技巧 - Feillet 2004 as mentioned in Laporte's paper
                                // 有些节点没必要继续访问了
                                for(int j=1; j<nbVertex-1; j++){
                                    if(!newVertexVisited[i]){
                                        float tt2 = (float)(tt + userParam.serviceTimeLength[i]+userParam.ttime[i][j]);
                                        double request2 = request1 + userParam.request[j];
                                        if((tt2>userParam.closeTime[j]) || (request2>userParam.capacity)){
                                            newVertexVisited[j] = true;
                                        }
                                    }
                                }
                                double cost = currentLabel.cost+userParam.cost[cityIndex][i];
                                // first label: start from depot (client 0)
                                labelList.add(new Label(i, currentIndex, cost, tt, request1, false, newVertexVisited));
                                //完美，存在这样的情况，路径经过的节点一样，只是顺序不一样，但是成本，到达时间，满足需求量等均一样，这个时候可以只保留另一个
                                if(!U.add((Integer) idx)){
                                    labelList.get(idx).dominated = true;
                                }else{
                                    city2labels[i].add(idx);
                                }
                            }
                        }
                    }
                }
            }
        }
        //结果解析filtering(过滤): 筛选出指定数量的从起点到终点的最好的路线
        int i=0;
        Integer labelIndex;
        while((i < nbRoute) && ( (labelIndex = P.pollFirst()) != null)){
            Label label = labelList.get(labelIndex);
            //todo 是否应该先判断label.city是否为末节点，检查上一部分是否能确保route的终点是末节点
            if(!label.dominated){
                if(/*(i < nbRoute / 2) ||*/ (label.cost < -1e-4)){
                    // System.out.println(s.cost);
//                    if(label.cost > 0) {
//                        System.out.println("warning >>>>>>>>>>>>>>>>>>>>");
//                    }
                    Route newRoute = new Route();
                    newRoute.setCost(label.cost);
                    newRoute.addCity(label.city);
                    int preLabelIndex = label.indexPreLabel;
                    while(preLabelIndex>=0){
                        newRoute.addCity(labelList.get(preLabelIndex).city);
                        preLabelIndex = labelList.get(preLabelIndex).indexPreLabel;
                    }
                    newRoute.switchPath();
                    routes.add(newRoute);
                    i++;
                }
            }
        }
    }

    public static void main(String[] args){
        // 主函数
        // 构造数据
        // 运算
        // 结果判断

    }
}
