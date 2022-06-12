package com.branch_and_bound_vrptw;
import java.util.ArrayList;
import java.util.PriorityQueue;


import ilog.concert.IloException;
import ilog.concert.IloNumExpr;
import ilog.concert.IloNumVar;
import ilog.concert.IloNumVarType;
import ilog.cplex.IloCplex;
/**
 * @author�� huangnan
 * @School: HuaZhong University of science and technology
 * @����˵�������벻ͬ���ļ�ǰҪ�ֶ��޸�vetexnum����������ֵΪ���е����,������������0��n+1��
 * ����������ȡ��Solomon��������
 *
 */
//�๦�ܣ�����ģ�Ͳ����
public class BaB_Vrptw {
	Data data;					//������Data�Ķ���
	Node node1;
	Node node2;
	int deep;//���
	public PriorityQueue<Node> queue;//��֧����
	Node best_note;//��ǰ��÷�֧
	double cur_best;//��ý�
	int []record_arc;//��¼��Ҫ��֧�Ľڵ�
	double x_gap;//��С����
	IloCplex model;				//����cplex�ڲ���Ķ���		
	public IloNumVar[][][] x;	//x[i][j][k]��ʾ��arcs[i][j]������k����
	public IloNumVar[][] w;		//�����������е��ʱ�����
	double cost;				//Ŀ��ֵobject
	double[][][] x_map;//cplex����x
	ArrayList<ArrayList<Integer>> routes;		//���峵��·������
	ArrayList<ArrayList<Double>> servetimes;	//���廨��ʱ������
	
	public BaB_Vrptw(Data data) {
		this.data = data;
		x_gap = data.gap;
		routes = new ArrayList<>();		//���峵��·������
		servetimes = new ArrayList<>();	//���廨��ʱ������
		//��ʼ������·���ͻ���ʱ������������Ϊ������k
		for (int k = 0; k < data.veh_num; k++) {
			ArrayList<Integer> r = new ArrayList<>();	
			ArrayList<Double> t = new ArrayList<>();	
			routes.add(r);							
			servetimes.add(t);							
		}
		x_map = new double[data.vertex_num][data.vertex_num][data.veh_num];
	}
	public void clear_lp() {
		data=null;
		routes.clear();
		servetimes.clear();
		x_map=null;
	}
	//����lp�⵽node
	@SuppressWarnings("unchecked")
	public void copy_lp_to_node(BaB_Vrptw lp, Node node) {
		node.node_routes.clear();
		node.node_servetimes.clear();
		node.node_cost = lp.cost;
		for (int i = 0; i < lp.x_map.length; i++) {
			for (int j = 0; j < lp.x_map[i].length; j++) {
				node.lp_x[i][j] = lp.x_map[i][j].clone();
			}
		}
		for (int i = 0; i < lp.routes.size(); i++) {
			node.node_routes.add((ArrayList<Integer>) lp.routes.get(i).clone());
		}
		for (int i = 0; i < lp.servetimes.size(); i++) {
			node.node_servetimes.add((ArrayList<Double>) lp.servetimes.get(i).clone());
		}
	}
	//�������ܣ�����VRPTW��ѧģ�ͽ���VRPTW��cplexģ��
	//����ģ��
	private void build_model() throws IloException {
		//model
		model = new IloCplex();
		model.setOut(null);
//		model.setParam(IloCplex.DoubleParam.EpOpt, 1e-9);
//		model.setParam(IloCplex.DoubleParam.EpGap, 1e-9);
		//variables
		x = new IloNumVar[data.vertex_num][data.vertex_num][data.veh_num];
		w = new IloNumVar[data.vertex_num][data.veh_num];				//�������ʵ��ʱ��
		//����cplex����x��w���������ͼ�ȡֵ��Χ
		for (int i = 0; i < data.vertex_num; i++) {
			for (int k = 0; k < data.veh_num; k++) {
		w[i][k] = model.numVar(0, 1e15, IloNumVarType.Float, "w" + i + "," + k);
			}
			for (int j = 0; j < data.vertex_num; j++) {
				if (data.arcs[i][j]==0) {
					x[i][j] = null;
				}
				else{
					//Xijk,��ʽ(10)-(11)
					for (int k = 0; k < data.veh_num; k++) {
	x[i][j][k] = model.numVar(0, 1, IloNumVarType.Float, "x" + i + "," + j + "," + k);
					}
				}
			}
		}
		//����Ŀ�꺯��
		//��ʽ(1)
		IloNumExpr obj = model.numExpr();
		for(int i = 0; i < data.vertex_num; i++){
			for(int j = 0; j < data.vertex_num; j++){
				if (data.arcs[i][j]==0) {
					continue;
				}
				for(int k = 0; k < data.veh_num; k++){
					obj = model.sum(obj, model.prod(data.dist[i][j], x[i][j][k]));
				}
			}
		}
		model.addMinimize(obj);
		//����Լ��
		//��ʽ(2)
		for(int i= 1; i < data.vertex_num-1;i++){
			IloNumExpr expr1 = model.numExpr();
			for (int k = 0; k < data.veh_num; k++) {
				for (int j = 1; j < data.vertex_num; j++) {
					if (data.arcs[i][j]==1) {
						expr1 = model.sum(expr1, x[i][j][k]);
					}
				}
			}
			model.addEq(expr1, 1);
		}
		//��ʽ(3)
		for (int k = 0; k < data.veh_num; k++) {
			IloNumExpr expr2 = model.numExpr();
			for (int j = 1; j < data.vertex_num; j++) {
				if (data.arcs[0][j]==1) {
					expr2 = model.sum(expr2, x[0][j][k]);
				}
			}
			model.addEq(expr2, 1);
		}
		//��ʽ(4)
		for (int k = 0; k < data.veh_num; k++) {
			for (int j = 1; j < data.vertex_num-1; j++) {
				IloNumExpr expr3 = model.numExpr();
				IloNumExpr subExpr1 = model.numExpr();
				IloNumExpr subExpr2 = model.numExpr();
				for (int i = 0; i < data.vertex_num; i++) {
					if (data.arcs[i][j]==1) {
						subExpr1 = model.sum(subExpr1,x[i][j][k]);
					}
					if (data.arcs[j][i]==1) {
						subExpr2 = model.sum(subExpr2,x[j][i][k]);
					}
				}
				expr3 = model.sum(subExpr1,model.prod(-1, subExpr2));
				model.addEq(expr3, 0);
			}
		}
		//��ʽ(5)
		for (int k = 0; k < data.veh_num; k++) {
			IloNumExpr expr4 = model.numExpr();
			for (int i = 0; i < data.vertex_num-1; i++) {
				if (data.arcs[i][data.vertex_num-1]==1) {
					expr4 = model.sum(expr4,x[i][data.vertex_num-1][k]);
				}
			}
			model.addEq(expr4, 1);
		}
		//��ʽ(6)
		double M = 1e5;
		for (int k = 0; k < data.veh_num; k++) {
			for (int i = 0; i < data.vertex_num; i++) {
				for (int j = 0; j < data.vertex_num; j++) {
					if (data.arcs[i][j] == 1) {
						IloNumExpr expr5 = model.numExpr();
						IloNumExpr expr6 = model.numExpr();
						expr5 = model.sum(w[i][k], data.s[i]+data.dist[i][j]);
						expr5 = model.sum(expr5,model.prod(-1, w[j][k]));
						expr6 = model.prod(M,model.sum(1,model.prod(-1, x[i][j][k])));
						model.addLe(expr5, expr6);
					}
				}
			}
		}
		//��ʽ(7)
		for (int k = 0; k < data.veh_num; k++) {
			for (int i = 1; i < data.vertex_num-1; i++) {
				IloNumExpr expr7 = model.numExpr();
				for (int j = 0; j < data.vertex_num; j++) {
					if (data.arcs[i][j] == 1) {
						expr7 = model.sum(expr7,x[i][j][k]);
					}
				}
				model.addLe(model.prod(data.a[i], expr7), w[i][k]);
				model.addLe(w[i][k], model.prod(data.b[i], expr7));
			}
		}
		//��ʽ(8)
		for (int k = 0; k < data.veh_num; k++) {
			model.addLe(data.E, w[0][k]);
			model.addLe(data.E, w[data.vertex_num-1][k]);
			model.addLe(w[0][k], data.L);
			model.addLe(w[data.vertex_num-1][k], data.L);
		}
		//��ʽ(9)
		for (int k = 0; k < data.veh_num; k++) {
			IloNumExpr expr8 = model.numExpr();
			for (int i = 1; i < data.vertex_num-1; i++) {
				IloNumExpr expr9 = model.numExpr();
				for (int j = 0; j < data.vertex_num; j++) {
					if (data.arcs[i][j] == 1) {
						expr9=model.sum(expr9,x[i][j][k]);
					}
				}
				expr8 = model.sum(expr8,model.prod(data.demands[i],expr9));
			}
			model.addLe(expr8, data.cap);
		}
	}
	//�������ܣ���ģ�ͣ������ɳ���·���͵õ�Ŀ��ֵ
	//��ȡcplex��
	public void get_value() throws IloException {
		routes.clear();
		servetimes.clear();
		cost = 0;
//		//��ʼ������·���ͻ���ʱ������������Ϊ������k
		for (int k = 0; k < data.veh_num; k++) {
			ArrayList<Integer> r = new ArrayList<>();	
			ArrayList<Double> t = new ArrayList<>();	
			routes.add(r);							
			servetimes.add(t);							
		}
		for (int i = 0; i < data.vertex_num; i++) {
			for (int j = 0; j < data.vertex_num; j++) {
				for (int k = 0; k < data.veh_num; k++) {
					x_map[i][j][k] = 0.0;
				}
				if (data.arcs[i][j]>0.5) {
					for (int k = 0; k < data.veh_num; k++) {
						x_map[i][j][k]=model.getValue(x[i][j][k]);
					}
				}
			}
		}
		//ģ�Ϳɽ⣬���ɳ���·��
		for(int k = 0; k < data.veh_num; k++){
			boolean terminate = true;
			int i = 0;
			routes.get(k).add(0);		
			servetimes.get(k).add(0.0);
			while(terminate){
				for (int j = 0; j < data.vertex_num; j++) {
					if (doubleCompare(x_map[i][j][k], 0)==1) {
						routes.get(k).add(j);
						servetimes.get(k).add(model.getValue(w[j][k]));
						i = j;
						break;
					}
				}
				if (i == data.vertex_num-1) {
					terminate = false;
				}
			}
		}
		cost = model.getObjValue();
	}
	//branch and bound����
	public void branch_and_bound(BaB_Vrptw lp) throws IloException {
		cur_best = 3000;//�����Ͻ�
		deep=0;
		record_arc = new int[3];
		node1 = new Node(data);
		best_note = null;
		queue = new PriorityQueue<Node>();
		//��ʼ�⣨�Ƿ��⣩
		for (int i = 0; i < lp.routes.size(); i++) {
			ArrayList<Integer> r = lp.routes.get(i);
			System.out.println();
			for (int j = 0; j < r.size(); j++) {
				System.out.print(r.get(j)+" ");
			}
		}
		lp.copy_lp_to_node(lp, node1);
//		node1.node_cost = lp.cost;
//		node1.lp_x = lp.x_map.clone();
//		node1.node_routes =lp.routes;
//		node1.node_servetimes = lp.servetimes;
		node2 = node1.note_copy();
		deep=0;
		node1.d=deep;
		queue.add(node1);
		//branch and bound����
		while (!queue.isEmpty()) {
			Node node = queue.poll();
			//ĳ֧���Ž���ڵ�ǰ��ÿ��н⣬ɾ��
			if (doubleCompare(node.node_cost, cur_best)>0) {
				continue;
			}else {
				record_arc = lp.find_arc(node.lp_x);
				//ĳ֧�ĺϷ���,0,1��ϵĽ�,��ǰ��֧��ý�
				if (record_arc[0]==-1) {
					//�ȵ�ǰ��ý�ã����µ�ǰ��
					if (doubleCompare(node.node_cost, cur_best)==-1) {
						lp.cur_best = node.node_cost;
						System.out.println(node.d+"  cur_best:"+cur_best);
						lp.best_note = node;
					}
					continue;
				}else {//���Է�֧
					node1 = lp.branch_left_arc(lp, node, record_arc);//��֧
					node2 = lp.branch_right_arc(lp, node, record_arc);//��֧
					if (node1!=null && doubleCompare(node1.node_cost, cur_best)<=0) {
						queue.add(node1);
					}
					if (node2!=null && doubleCompare(node2.node_cost, cur_best)<=0) {
						queue.add(node2);
					}
				}
			}
		}
	}
	//��֧����
	public void set_bound(Node node) throws IloException {
		for (int i = 0; i < data.vertex_num; i++) {
			for (int j = 0; j < data.vertex_num; j++) {
				if (data.arcs[i][j]>0.5) {
					if (node.node_x[i][j]==0) {
						for (int k = 0; k < data.veh_num; k++) {
							x[i][j][k].setLB(0.0);
							x[i][j][k].setUB(1.0);
						}
					}else if (node.node_x[i][j]==-1) {
						for (int k = 0; k < data.veh_num; k++) {
							x[i][j][k].setLB(0.0);
							x[i][j][k].setUB(0.0);
						}
					}else {
						for (int k = 0; k < data.veh_num; k++) {
							if (node.node_x_map[i][j][k]==1) {
								x[i][j][k].setLB(1.0);
								x[i][j][k].setUB(1.0);
							}else {
								x[i][j][k].setLB(0.0);
								x[i][j][k].setUB(0.0);
							}
						}
					}
				}
			}
		}
	}
	public void set_bound1(Node node) throws IloException {
		for (int i = 0; i < data.vertex_num; i++) {
			for (int j = 0; j < data.vertex_num; j++) {
				if (data.arcs[i][j]>0.5) {
					for (int k = 0; k < data.veh_num; k++) {
						if (node.node_x_map[i][j][k]==0) {
							x[i][j][k].setLB(0.0);
							x[i][j][k].setUB(1.0);
						}else if (node.node_x_map[i][j][k]==-1) {
							x[i][j][k].setLB(0.0);
							x[i][j][k].setUB(0.0);
						}else {
							x[i][j][k].setLB(1.0);
							x[i][j][k].setUB(1.0);
						}
					}
				}
			}
		}
	}
	//������֧
	public Node branch_left_arc(BaB_Vrptw lp,Node father_node,int[] record) throws IloException {
		if (record[0] == -1) {
			return null;
		}
		Node new_node = new Node(data);
		new_node = father_node.note_copy();
		new_node.node_x[record[0]][record[1]] = -1;
		for (int k = 0; k < data.veh_num; k++) {
			new_node.node_x_map[record[0]][record[1]][k]=0;
		}
//		new_node.node_x_map[record[0]][record[1]][record[2]]=-1;
		//������֧
		lp.set_bound(new_node);
		
		if (lp.model.solve()) {
			lp.get_value();
			deep++;
			new_node.d=deep;
			lp.copy_lp_to_node(lp, new_node);
			System.out.println(new_node.d+" "+lp.cost);
		}else {
			new_node.node_cost = data.big_num;
		}
		return new_node;
}
	//������֧
	public Node branch_right_arc(BaB_Vrptw lp,Node father_node,int[] record) throws IloException {
		if (record[0] == -1) {
			return null;
		}
		Node new_node = new Node(data);
		new_node = father_node.note_copy();
		new_node.node_x[record[0]][record[1]] = 1;
//		new_node.node_x_map[record[0]][record[1]][record[2]]=1;
		for (int k = 0; k < data.veh_num; k++) {
			if (k==record[2]) {
				new_node.node_x_map[record[0]][record[1]][k]=1;
			}else {
				new_node.node_x_map[record[0]][record[1]][k]=0;
			}
		}
		//������֧
		lp.set_bound(new_node);
		if (lp.model.solve()) {
			lp.get_value();
			deep++;
			new_node.d=deep;
			System.out.println(new_node.d+" right: "+lp.cost);
			lp.copy_lp_to_node(lp, new_node);
		}else {
			new_node.node_cost = data.big_num;
		}
		return new_node;
	}
	//�ҵ���Ҫ��֧��֧��λ��
	public int[] find_arc1(double[][][] x) {
		int record[] = new int[3];//��¼��֧����
		double cur_dif = 0;
		double min_dif = Double.MAX_VALUE;
		//�ҳ���ӽ�0.5�Ļ�
		for (int i = 1; i <data.vertex_num-1; i++) {
			for (int j = 1; j < data.vertex_num-1; j++) {
				if (data.arcs[i][j]>0.5) {
					for (int k = 0; k <data.veh_num; k++) {
						//���û�ֵΪ0��1�������
						if (is_one_zero(x[i][j][k])) {
							continue;
						}
						cur_dif = get_dif(x[i][j][k]);
						if (doubleCompare(cur_dif, min_dif)==-1) {
							record[0] = i;
							record[1] = j;
							record[2] = k;
							min_dif = cur_dif;
						}
					}
				}
			}
		}
		//depot
		if (doubleCompare(min_dif, Double.MAX_VALUE)==0) {
			for (int i = 1; i < data.vertex_num-1; i++) {
				if (data.arcs[0][i]>0.5) {
					for (int k = 0; k < data.veh_num; k++) {
						if (is_fractional(x[0][i][k])) {
							cur_dif = get_dif(x[0][i][k]);
							if (doubleCompare(cur_dif, min_dif)==-1) {
								record[0] = 0;
								record[1] = i;
								record[2] = k;
								min_dif = cur_dif;
							}
						}
					}
				}
				if (data.arcs[i][data.vertex_num-1]>0.5) {
					for (int k = 0; k < data.veh_num; k++) {
						if (is_fractional(x[i][data.vertex_num-1][k])) {
							cur_dif = get_dif(x[i][data.vertex_num-1][k]);
							if (doubleCompare(cur_dif, min_dif)==-1) {
								record[0] = i;
								record[1] = data.vertex_num-1;
								record[2] = k;
								min_dif = cur_dif;
							}
						}
					}
				}
			}
		}
		if (doubleCompare(min_dif, data.big_num)==1) {
			record[0] = -1;
			record[1] = -1;
			record[2] = -1;
		}
		return record;
	}
//	�ҵ�Ҫ��֧�Ļ�
	public int[] find_arc(double[][][] x) {
		int record[] = new int[3];//��¼��֧����
		for (int i = 0; i <data.vertex_num; i++) {
			for (int j = 0; j < data.vertex_num; j++) {
				if (data.arcs[i][j]>0.5) {
					for (int k = 0; k <data.veh_num; k++) {
						//���û�ֵΪ0��1�������
						if (is_one_zero(x[i][j][k])) {
							continue;
						}
//						cur_dif = get_dif(x[i][j][k]);
						record[0] = i;
						record[1] = j;
						record[2] = k;
						return record;
					}
				}
			}
		}
		record[0] = -1;
		record[1] = -1;
		record[2] = -1;
		return record;
	}
	//�Ƚ�����double��ֵ�Ĵ�С
	public int doubleCompare(double a, double b){
		if(a - b > x_gap)
			return 1;
		if(b - a > x_gap)
			return -1;		
		return 0;
	} 
	//�ж��Ƿ�Ϊ0��1֮���С��
	public boolean is_fractional(double v){
		if( v > (int) v + x_gap && v < (int) v + 1 - x_gap)
			return true;
		else
			return false;
	}
	//�ж��Ƿ�Ϊ0����1
	public boolean is_one_zero(double temp) {
		if (doubleCompare(temp, 0)==0 || doubleCompare(temp, 1)==0) {
			return true;
		}else {
			return false;
		}
	}
	//��ȡ��0.5�ľ���
	public double get_dif(double temp) {
		double v = (int)temp+0.5;
		if (v>temp) {
			return v-temp;
		} else {
			return temp-v;
		}
	}
	public BaB_Vrptw init(BaB_Vrptw lp) throws IloException {
		lp.build_model();
		if (lp.model.solve()) {
			lp.get_value();
			int aa=0;
			for (int i = 0; i < lp.routes.size(); i++) {
				if (lp.routes.get(i).size()==2) {
					aa++;
				}
			}
			System.out.println(aa);
			if (aa==0) {
				data.veh_num -=1;
				lp.model.clearModel();
				lp = new BaB_Vrptw(data);
				return init(lp);
			}else {
				data.veh_num -=aa;
				lp.model.clearModel();
				lp = new BaB_Vrptw(data);
				return init(lp);
			}
		}else {
			data.veh_num +=1;
			System.out.println("vehicle number: "+data.veh_num);
			lp.model.clearModel();
			lp = new BaB_Vrptw(data);
			lp.build_model();
			if (lp.model.solve()) {
				lp.get_value();
				return lp;
			}else {
				System.out.println("error init");
				return null;
			}
		}
	}
	public static void main(String[] args) throws Exception {
		Data data = new Data();
		int vetexnum = 102;//���е����������0��n+1�����������ĵ�
		//���벻ͬ���ļ�ǰҪ�ֶ��޸�vetexnum����������ֵ�������е����,������������
		String path = "src/main/resources/Solomon/c102.txt";//������ַ
		data.Read_data(path,data,vetexnum);
		System.out.println("input succesfully");
		System.out.println("cplex procedure###########################");
		BaB_Vrptw lp = new BaB_Vrptw(data);
		double cplex_time1 = System.nanoTime();
		//ɾ��δ�õĳ�������С��ռ�
		lp=lp.init(lp);
		System.out.println(":   "+lp.data.veh_num);
		lp.branch_and_bound(lp);
		Check check = new Check(lp);
		check.fesible();
		double cplex_time2 = System.nanoTime();
		double cplex_time = (cplex_time2 - cplex_time1) / 1e9;//���ʱ�䣬��λs
		System.out.println("cplex_time " + cplex_time + " bestcost " + lp.cur_best);
		for (int i = 0; i < lp.best_note.node_routes.size(); i++) {
			ArrayList<Integer> r = lp.best_note.node_routes.get(i);
			System.out.println();
			for (int j = 0; j < r.size(); j++) {
				System.out.print(r.get(j)+" ");
			}
		}
	}
}
