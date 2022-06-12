package com.chb;

import java.util.ArrayList;

public class Node implements Comparable {
    Data data;
    int d;                    // 节点所在分支定界树的深度
    double node_cost;         //目标值object
    double[][][] lp_x;        //记录lp中决策变量x的值
    int[][] node_x;           //0表示弧可以访问，1表示必须访问，-1表示不能访问
    int[][][] node_x_map;     //node_xij=1时,node_x_map_ijk=1表示必须访问，node_x_map_ijk=0表示不能访问
    ArrayList<ArrayList<Integer>> node_routes;        //定义车辆路径链表
    ArrayList<ArrayList<Double>> node_servetimes;    //定义花费时间链表

    public Node(Data data) {
        super();
        this.data = data;
        node_cost = Data.big_num;
        lp_x = new double[Data.pointnum][Data.pointnum][Data.carnum];
        node_x_map = new int[Data.pointnum][Data.pointnum][Data.carnum];
        node_x = new int[Data.pointnum][Data.pointnum];
        node_routes = new ArrayList<ArrayList<Integer>>();
        node_servetimes = new ArrayList<ArrayList<Double>>();
    }

    //复制node
    @SuppressWarnings("unchecked")
    public Node note_copy() {
        Node new_node = new Node(data);
        new_node.d = d;
        new_node.node_cost = node_cost;
        for (int i = 0; i < lp_x.length; i++) {
            for (int j = 0; j < lp_x[i].length; j++) {
                new_node.lp_x[i][j] = lp_x[i][j].clone();
            }
        }
        for (int i = 0; i < node_x.length; i++) {
            new_node.node_x[i] = node_x[i].clone();
        }
        for (int i = 0; i < node_x_map.length; i++) {
            for (int j = 0; j < node_x_map[i].length; j++) {
                new_node.node_x_map[i][j] = node_x_map[i][j].clone();
            }
        }
        for (int i = 0; i < node_routes.size(); i++) {
            new_node.node_routes.add((ArrayList<Integer>) node_routes.get(i).clone());
        }
        for (int i = 0; i < node_servetimes.size(); i++) {
            new_node.node_servetimes.add((ArrayList<Double>) node_servetimes.get(i).clone());
        }
        return new_node;
    }

    public int compareTo(Object o) {
        Node node = (Node) o;
        if (node_cost < node.node_cost)
            return -1;
        else if (node_cost == node.node_cost)
            return 0;
        else
            return 1;
    }
}

