package com.branch_and_bound_vrptw;

import java.util.ArrayList;

import ilog.concert.IloException;

//�๦�ܣ���Ŀ������ж�(��ֱ����������)
class Check{
	double epsilon = 0.0001;
	Data data = new Data();
	ArrayList<ArrayList<Integer>> routes = new ArrayList<>();
	ArrayList<ArrayList<Double>> servetimes = new ArrayList<>();
	public Check(BaB_Vrptw lp) {
		super();
		this.data = lp.data;
		this.routes = lp.routes;
		this.servetimes = lp.servetimes;
	}
	//�������ܣ��Ƚ��������Ĵ�С
	public int double_compare(double v1,double v2) {
		if (v1 < v2 - epsilon) {
			return -1;
		}
		if (v1 > v2 + epsilon) {
			return 1;
		}
		return 0;
	}
	//�������ܣ���Ŀ������ж�
	public void fesible() throws IloException {
		//���������������ж�
		if (routes.size() > data.veh_num) {
			System.out.println("error: vecnum!!!");
			System.exit(0);
		}
		//�����غɿ������ж�
		for (int k = 0; k < routes.size(); k++) {
			ArrayList<Integer> route = routes.get(k);
			double capasity = 0;
			for (int i = 0; i < route.size(); i++) {
				capasity += data.demands[route.get(i)];
			}
			if (capasity > data.cap) {
				System.out.println("error: cap!!!");
				System.exit(0);
			}
		}
		//ʱ�䴰���������������ж�
		for (int k = 0; k < routes.size(); k++) {
			ArrayList<Integer> route = routes.get(k);
			ArrayList<Double> servertime = servetimes.get(k);
			double capasity = 0;
			for (int i = 0; i < route.size()-1; i++) {
				int origin = route.get(i);
				int destination = route.get(i+1);
				double si = servertime.get(i);
				double sj = servertime.get(i+1);
				if (si < data.a[origin] && si >  data.b[origin]) {
					System.out.println("error: servertime!");
					System.exit(0);
				}
		if (double_compare(si + data.dist[origin][destination],data.b[destination]) > 0) {
					System.out.println(origin + ": [" + data.a[origin] 
							+ ","+data.b[origin]+"]"+ " "+ si);
					System.out.println(destination + ": [" +
							data.a[destination] + ","+data.b[destination]+"]"+ " "+ sj);
					System.out.println(data.dist[origin][destination]);
					System.out.println(destination + ":" );
					System.out.println("error: forward servertime!");
					System.exit(0);
				}
			if (double_compare(sj - data.dist[origin][destination],data.a[origin]) < 0) {
					System.out.println(origin + ": [" + data.a[origin]
							+ ","+data.b[origin]+"]"+ " "+ si);
					System.out.println(destination + ": [" + data.a[destination] 
							+ ","+data.b[destination]+"]"+ " "+ sj);
					System.out.println(data.dist[origin][destination]);
					System.out.println(destination + ":" );
					System.out.println("error: backward servertime!");
					System.exit(0);
				}
			}
			if (capasity > data.cap) {
				System.out.println("error: cap!!!");
				System.exit(0);
			}
		}
	}
}