package com.operation.vrptw.lx;

import java.io.IOException;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;

import com.sun.scenario.effect.impl.sw.sse.SSEBlend_SRC_OUTPeer;
import ilog.concert.*;
import ilog.cplex.*;

import javax.xml.stream.FactoryConfigurationError;

public class ColumnGeneration {

    static class IloNumVarArray{
        int _num = 0;
        IloNumVar[] _array = new IloNumVar[32];
        void add(IloNumVar ivar){
            if (_num >= _array.length){
                IloNumVar[] array = new IloNumVar[2*_array.length];
                System.arraycopy(_array, 0, array, 0, _num);
                _array = array;
            }
            _array[_num++] = ivar;
        }
        IloNumVar getElement(int index){return _array[index];}
        int getSize(){return _num;}
    }

    public double computeColGen(VRPParam userParam, ArrayList<Route> routeList) throws IOException{

        try{
            //这个程序，试图在每次调用时都新建一个环境，实际上可以只在程序开始时新建一个cplex环境，每次加入新的列后，只在原环境中的系数矩阵中增加列即可
            //cplex环境初始化
            IloCplex cplex = new IloCplex();
            IloObjective objFunc = cplex.addMinimize();
            //for each vertex/client, one constraint(chapter 3.23)
            // IloRange定义约束的上下界
            //对每个客户都要建立一个约束
            IloRange[] constraints = new IloRange[userParam.nbClients];
            for(int i=0; i<userParam.nbClients; i++){
                constraints[i] = cplex.addRange(1.0, Double.MAX_VALUE, "client_"+i);
            }
            // 定义变量 y_p表示路径p是否选择
            IloNumVarArray y = new IloNumVarArray();

            // 构造初始模型
            if(routeList.size()>0){
                for(Route r: routeList){
                    int v, city;
                    double cost = 0.0;
                    int preCity = 0;
                    //更新路径的成本
                    for (int i=1; i<r.getPath().size(); i++){
                        city = r.getPath().get(i);
                        cost += userParam.dist[preCity][city];
                        preCity = city;
                    }
                    r.setCost(cost);
                    // 按列建模：目标函数和约束逐个添加新增列的系数
                    IloColumn column = cplex.column(objFunc, r.getCost());
                    // path的第一个元素是虚拟起点，最后一个元素是虚拟终点
                    for(int i=1; i<r.getPath().size()-1;i++){
                        v = r.getPath().get(i)-1;
                        column = column.and(cplex.column(constraints[v], 1.0));
                    }
                    // todo 创建变量y_1  绑定的列，下界，     上界，              如何设置为0-1变量
                    y.add(cplex.numVar(column, 0.0, Double.MAX_VALUE));
                }
            }

            // 创建初始列，以确保解可行
            if(routeList.size() < userParam.nbClients){
                for(int i=0; i< userParam.nbClients; i++){
                    double cost = userParam.dist[0][i+1]+userParam.dist[i+1][userParam.nbClients+1];
                    IloColumn column = cplex.column(objFunc, cost);
                    column = column.and(cplex.column(constraints[i], 1.0));
                    y.add(cplex.numVar(column, 0.0, Double.MAX_VALUE));
                    Route newRoute = new Route();
                    int[] path = {0, i+1,userParam.nbClients+1};
                    newRoute.addCityFromPath(path);
                    newRoute.setCost(cost);
                    routeList.add(newRoute);
                }
            }

            //输出模型
            cplex.exportModel("columnGeneration.lp");

            //设置求解参数
            cplex.setParam(IloCplex.IntParam.RootAlg, IloCplex.Algorithm.Primal);
            cplex.setOut(null);
            // seconds: 2h=7200
            cplex.setParam(IloCplex.DoubleParam.TimeLimit, 60);

            //循环
            DecimalFormat df = new DecimalFormat("#0000.00");
            boolean oncemore = true;
            List<Double> objRecord = new ArrayList<>();
            int iter = 1;
            // 循环添加列
            while (oncemore){
                oncemore = false;
                // solve the current RMP
                // 求解松弛限制主问题
                if(!cplex.solve()){
                    System.out.println("列生成算法，松弛主问题不可行");
                    return 1E10;
                }
                //记录每一代迭代的解
                objRecord.add(cplex.getObjValue());
                System.out.println(cplex.getStatus());
                if(iter % 100 ==0){
                    cplex.exportModel(String.format("model_%d.lp",routeList.size()));
                    System.out.printf("CG Iter %d, 模型中的我路径数量为%d, 当前成本为%.2f%n", iter, routeList.size(), cplex.getObjValue());
                    System.out.flush();
                }

                // 判断是否收敛

                // todo 为什么如此更新，更新每个条线路的价格
                double[] pi = cplex.getDuals(constraints);
                for(int i=1; i< userParam.nbClients+1; i++)
                    for(int j=1; j<userParam.totalNode; j++)
                        userParam.cost[i][j] = userParam.dist[i][j] - pi[i-1];

                //使用标号算法求解最短问题，获得一些列
                SPPRC subProblem = new SPPRC();
                ArrayList<Route> spprcRouteList = new ArrayList<>();
                int nbRoute = userParam.nbClients;

                subProblem.labelingAlgorithm(userParam, spprcRouteList, nbRoute);
                subProblem = null;

                //将列加入到限制主问题中
                if(spprcRouteList.size()>0){
                    for(Route route: spprcRouteList){
                        ArrayList<Integer> path = route.getPath();
                        int prevCity = path.get(1);
                        double cost = userParam.dist[0][prevCity];
                        IloColumn column = cplex.column(constraints[prevCity-1], 1.0);
                        for(int i=2; i<path.size()-1; i++){
                            int city = path.get(i);
                            cost += userParam.dist[prevCity][city];
                            prevCity = city;
                            column = column.and(cplex.column(constraints[city-1], 1.0));
                        }
                        cost += userParam.dist[prevCity][userParam.nbClients+1];
                        column = column.and(cplex.column(objFunc, cost));
                        y.add(cplex.numVar(column, 0, Double.MAX_VALUE, "p_"+routeList.size()));
                        route.setCost(cost);
                        routeList.add(route);
                    }
                    oncemore = true;
                }
                iter += 1;
            }
            System.out.println();
            double obj;
            for(int i=0; i<y.getSize(); i++)
                routeList.get(i).setQ(cplex.getValue(y.getElement(i)));
            obj = cplex.getObjValue();
            cplex.end();

            //绘图

            return obj;
        }catch(IloException e){
            System.err.println("Connect exception caught '" + e + "' caught");
        }
        return 1E10;
    }

    // 构造初始模型
    public void initialModel(VRPParam userParam, ArrayList<Route> routeList, IloCplex cplex,
                             IloObjective objFunc, IloRange[] constraints, IloNumVarArray y) throws IloException {

        for(Route r: routeList){
            int v, city;
            double cost = 0.0;
            int preCity = 0;
            //更新路径的成本
            for (int i=1; i<r.getPath().size(); i++){
                city = r.getPath().get(i);
                cost += userParam.dist[preCity][city];
                preCity = city;
            }
            r.setCost(cost);
            // 按列建模：目标函数和约束逐个添加新增列的系数
            IloColumn column = cplex.column(objFunc, r.getCost());
            // path的第一个元素是虚拟起点，最后一个元素是虚拟终点
            for(int i=1; i<r.getPath().size()-1;i++){
                v = r.getPath().get(i)-1;
                column = column.and(cplex.column(constraints[v], 1.0));
            }
            // todo 创建变量y_1  绑定的列，下界，     上界，              如何设置为0-1变量
            y.add(cplex.numVar(column, 0.0, Double.MAX_VALUE));
        }
        // 创建初始列，以确保解可行
        if(routeList.size() < userParam.nbClients){
            for(int i=0; i< userParam.nbClients; i++){
                double cost = userParam.dist[0][i+1]+userParam.dist[i+1][userParam.nbClients+1];
                IloColumn column = cplex.column(objFunc, cost);
                column = column.and(cplex.column(constraints[i], 1.0));
                y.add(cplex.numVar(column, 0.0, Double.MAX_VALUE));
                Route newRoute = new Route();
                int[] path = {0, i+1,userParam.nbClients+1};
                newRoute.addCityFromPath(path);
                newRoute.setCost(cost);
                routeList.add(newRoute);
            }
        }

    }

    public void addColumns(VRPParam userParam, ArrayList<Route> routeList, IloCplex cplex,
                           IloObjective objFunc, IloRange[] constraints, IloNumVarArray y) throws IloException {

        for(Route r: routeList){
            int v, city;
            double cost = 0.0;
            int preCity = 0;
            //更新路径的成本
            for (int i=1; i<r.getPath().size(); i++){
                city = r.getPath().get(i);
                cost += userParam.dist[preCity][city];
                preCity = city;
            }
            r.setCost(cost);
            // 按列建模：目标函数和约束逐个添加新增列的系数
            IloColumn column = cplex.column(objFunc, r.getCost());
            // path的第一个元素是虚拟起点，最后一个元素是虚拟终点
            for(int i=1; i<r.getPath().size()-1;i++){
                v = r.getPath().get(i)-1;
                column = column.and(cplex.column(constraints[v], 1.0));
            }
            // todo 创建变量y_1  绑定的列，下界，     上界，              如何设置为0-1变量
            y.add(cplex.numVar(column, 0.0, Double.MAX_VALUE));
        }
    }

}
