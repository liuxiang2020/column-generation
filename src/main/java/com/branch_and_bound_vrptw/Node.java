package com.branch_and_bound_vrptw;

import java.util.ArrayList;

@SuppressWarnings("rawtypes")
public class Node implements Comparable{
	Data data;
	int d;
	double node_cost;				//Ŀ��ֵobject
	double[][][]lp_x;//��¼lp��
	int[][][] node_x_map;//node_xij=1ʱ,node_x_mapijk=1��ʾ������ʣ�node_x_mapijk=0��ʾ���ܷ���
	int[][] node_x;//0��ʾ�����Է��ʣ�1��ʾ������ʣ�-1��ʾ���ܷ���
	ArrayList<ArrayList<Integer>> node_routes;		//���峵��·������
	ArrayList<ArrayList<Double>> node_servetimes;	//���廨��ʱ������
	
	public Node(Data data) {
		super();
		this.data = data;
		node_cost = data.big_num;
		lp_x = new double [data.vertex_num][data.vertex_num][data.veh_num];
		node_x_map = new int[data.vertex_num][data.vertex_num][data.veh_num];
		node_x = new int[data.vertex_num][data.vertex_num];
		node_routes = new ArrayList<ArrayList<Integer>>();
		node_servetimes = new ArrayList<ArrayList<Double>>();
	}
	//����node
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
	
	public int compareTo(Object o){
		Node node = (Node) o;
		if(node_cost < node.node_cost)
			return -1;
		else if(node_cost == node.node_cost)
			return 0;
		else
			return 1;
	}
}
