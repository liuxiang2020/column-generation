package com.branch_and_bound_vrptw;

import java.io.BufferedReader;
import java.io.FileReader;
import java.util.Scanner;

//�������
class Data{
	int vertex_num;			//���е㼯��n�������������ĺͿͻ��㣬��β��0��n��Ϊ�������ģ�
	double E;	      		//��������ʱ�䴰��ʼʱ��
	double	L;	     		//��������ʱ�䴰����ʱ��
	int veh_num;    		//������
	double cap;     		//�����غ�
	int[][] vertexs;		//���е������x,y
	int[] demands;			//������
	int[] vehicles;			//�������
	double[] a;				//ʱ�䴰��ʼʱ�䡾a[i],b[i]��
	double[] b;				//ʱ�䴰����ʱ�䡾a[i],b[i]��
	double[] s;				//�ͻ���ķ���ʱ��
	int[][] arcs;			//arcs[i][j]��ʾi��j��Ļ�
	double[][] dist;		//��������������ǹ�ϵ,���þ����ʾ���� C[i][j]=dist[i][j]
	double gap= 1e-6;
	double big_num = 100000;
	//�ض�С��3.26434-->3.2
	public double double_truncate(double v){
		int iv = (int) v;
		if(iv+1 - v <= gap)
			return iv+1;
		double dv = (v - iv) * 10;
		int idv = (int) dv;
		double rv = iv + idv / 10.0;
		return rv;
	}	
	public Data() {
		super();
	}
	//�������ܣ���txt�ļ��ж�ȡ���ݲ���ʼ������
	public void Read_data(String path,Data data,int vertexnum) throws Exception{
		String line = null;
		String[] substr = null;
		Scanner cin = new Scanner(new BufferedReader(new FileReader(path)));  //��ȡ�ļ�
		for(int i =0; i < 4;i++){
			line = cin.nextLine();  //��ȡһ��
		}
		line = cin.nextLine();
		line.trim(); //���ص����ַ��������һ��������ɾ����ʼ�ͽ�β�Ŀո�
		substr = line.split(("\\s+")); //�Կո�Ϊ��־���ַ������
		//��ʼ������
		data.vertex_num = vertexnum;
		data.veh_num = Integer.parseInt(substr[1]); 
		data.cap = Integer.parseInt(substr[2]);
		data.vertexs =new int[data.vertex_num][2];				//���е������x,y
		data.demands = new int[data.vertex_num];					//������
		data.vehicles = new int[data.veh_num];					//�������
		data.a = new double[data.vertex_num];						//ʱ�䴰��ʼʱ��
		data.b = new double[data.vertex_num];						//ʱ�䴰����ʱ��
		data.s = new double[data.vertex_num];						//����ʱ��
		data.arcs = new int[data.vertex_num][data.vertex_num];
		//�������,�������ǹ�ϵ,�þ����ʾcost
		data.dist = new double[data.vertex_num][data.vertex_num];
		for(int i =0; i < 4;i++){
			line = cin.nextLine();
		}
		//��ȡvetexnum-1������
		for (int i = 0; i < data.vertex_num - 1; i++) {
			line = cin.nextLine();
			line.trim();
			substr = line.split("\\s+");
			data.vertexs[i][0] = Integer.parseInt(substr[2]);
			data.vertexs[i][1] = Integer.parseInt(substr[3]);
			data.demands[i] = Integer.parseInt(substr[4]);
			data.a[i] = Integer.parseInt(substr[5]);
			data.b[i] = Integer.parseInt(substr[6]);
			data.s[i] = Integer.parseInt(substr[7]);
		}
		cin.close();//�ر���
		//��ʼ���������Ĳ���
		data.vertexs[data.vertex_num-1] = data.vertexs[0];
		data.demands[data.vertex_num-1] = 0;
		data.a[data.vertex_num-1] = data.a[0];
		data.b[data.vertex_num-1] = data.b[0];
		data.E = data.a[0];
		data.L = data.b[0];
		data.s[data.vertex_num-1] = 0;		
		double min1 = 1e15;
		double min2 = 1e15;
		//��������ʼ��
		for (int i = 0; i < data.vertex_num; i++) {
			for (int j = 0; j < data.vertex_num; j++) {
				if (i == j) {
					data.dist[i][j] = 0;
					continue;
				}
				data.dist[i][j] = 
					Math.sqrt((data.vertexs[i][0]-data.vertexs[j][0])
							*(data.vertexs[i][0]-data.vertexs[j][0])+
					(data.vertexs[i][1]-data.vertexs[j][1])
					*(data.vertexs[i][1]-data.vertexs[j][1]));
				data.dist[i][j]=data.double_truncate(data.dist[i][j]);
			}
		}
		data.dist[0][data.vertex_num-1] = 0;
		data.dist[data.vertex_num-1][0] = 0;
		//��������������ǹ�ϵ
		for (int  k = 0; k < data.vertex_num; k++) {
			for (int i = 0; i < data.vertex_num; i++) {
				for (int j = 0; j < data.vertex_num; j++) {
					if (data.dist[i][j] > data.dist[i][k] + data.dist[k][j]) {
						data.dist[i][j] = data.dist[i][k] + data.dist[k][j];
					}
				}
			}
		}
		//��ʼ��Ϊ��ȫͼ
		for (int i = 0; i < data.vertex_num; i++) {
			for (int j = 0; j < data.vertex_num; j++) {
				if (i != j) {
					data.arcs[i][j] = 1;
				}
				else {
					data.arcs[i][j] = 0;
				}
			}
		}
		//��ȥ������ʱ�䴰������Լ���ı�
		for (int i = 0; i < data.vertex_num; i++) {
			for (int j = 0; j < data.vertex_num; j++) {
				if (i == j) {
					continue;
				}
				if (data.a[i]+data.s[i]+data.dist[i][j]>data.b[j] ||
						data.demands[i]+data.demands[j]>data.cap) {
					data.arcs[i][j] = 0;
				}
				if (data.a[0]+data.s[i]+data.dist[0][i]+data.dist[i][data.vertex_num-1]>
				data.b[data.vertex_num-1]) {
					System.out.println("the calculating example is false");
					
				}
			}
		}
		for (int i = 1; i < data.vertex_num-1; i++) {
			if (data.b[i] - data.dist[0][i] < min1) {
				min1 = data.b[i] - data.dist[0][i];
			}
			if (data.a[i] + data.s[i] + data.dist[i][data.vertex_num-1] < min2) {
				min2 = data.a[i] + data.s[i] + data.dist[i][data.vertex_num-1];
			}
		}
		if (data.E > min1 || data.L < min2) {
			System.out.println("Duration false!");
			System.exit(0);//��ֹ����
		}
		//��ʼ����������0��n+1����Ĳ���
		data.arcs[data.vertex_num-1][0] = 0;
		data.arcs[0][data.vertex_num-1] = 1;
		for (int i = 1; i < data.vertex_num-1; i++) {
			data.arcs[data.vertex_num-1][i] = 0;
		}
		for (int i = 1; i < data.vertex_num-1; i++) {
			data.arcs[i][0] = 0;
		}
	}
}