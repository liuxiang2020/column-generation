package com.operation.vrptw.lx;
import java.io.BufferedReader;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.Scanner;
//import org.springframework.core.io.ClassPathResource;
import com.sun.org.apache.bcel.internal.util.ClassPath;
import ilog.concert.IloException;
import ilog.concert.IloNumExpr;
import ilog.concert.IloNumVar;
import ilog.concert.IloNumVarType;
import ilog.cplex.IloCplex;
/**
 * @author： huangnan
 * @School: HuaZhong University of science and technology
 * @操作说明：读入不同的文件前要手动修改vetexnum参数，参数值为所有点个数,包括配送中心0和n+1，
 * 代码算例截取于Solomon测试算例
 *
 */

//类功能：建立模型并求解
public class VRPTWSolver {

    Data data;					//定义类Data的对象
    IloCplex model;				//定义cplex内部类的对象
    public IloNumVar[][][] x;	//x[i][j][k]表示弧arcs[i][j]被车辆k访问
    public IloNumVar[][] w;		//车辆访问所有点的时间矩阵
    double cost;				//目标值object
    Solution solution;
    public VRPTWSolver(Data data) {
        this.data = data;
    }
    //函数功能：解模型，并生成车辆路径和得到目标值
    public void solve() throws IloException {

        ArrayList<ArrayList<Integer>> routes = new ArrayList<>();		//定义车辆路径链表
        ArrayList<ArrayList<Double>> servetimes = new ArrayList<>();	//定义花费时间链表
        //初始化车辆路径和花费时间链表，链表长度为车辆数k
        for (int k = 0; k < data.vecnum; k++) {
            ArrayList<Integer> r = new ArrayList<>();	//定义一个对象为int型的链表
            ArrayList<Double> t = new ArrayList<>();	//定义一个对象为double型的链表
            routes.add(r);								//将上述定义的链表加入到链表routes中
            servetimes.add(t);							//同上
        }
        //判断建立的模型是否可解， 若不可解，则直接跳出solve函数
        if(model.solve() == false){
            //模型不可解
            System.out.println("problem should not solve false!!!");
            return;
        }
        else{
            //模型可解，生成车辆路径
            for(int k = 0; k < data.vecnum; k++){
                boolean terminate = true;
                int i = 0;
                routes.get(k).add(0);
                servetimes.get(k).add(0.0);
                while(terminate){
                    for (int j = 0; j < data.vetexnum; j++) {
                        if (data.arcs[i][j]>=0.5 && model.getValue(x[i][j][k])>=0.5) {
                            routes.get(k).add(j);
                            servetimes.get(k).add(model.getValue(w[j][k]));
                            i = j;
                            break;
                        }
                    }
                    if (i == data.vetexnum-1) {
                        terminate = false;
                    }
                }
            }
        }
        solution = new Solution(data,routes,servetimes);
        cost = model.getObjValue();
        System.out.println("routes="+solution.routes);
    }

    //函数功能：根据VRPTW数学模型建立VRPTW的cplex模型
    private void build_model() throws IloException {
        //model
        model = new IloCplex();
        model.setOut(null);
        //variables

        x = new IloNumVar[data.vetexnum][data.vetexnum][data.vecnum];
        //车辆访问点的时间
        w = new IloNumVar[data.vetexnum][data.vecnum];

        //定义cplex变量x和w的数据类型及取值范围
        for (int i = 0; i < data.vetexnum; i++) {
            for (int k = 0; k < data.vecnum; k++) {
                w[i][k] = model.numVar(0, 1e15, IloNumVarType.Float, "w(" + i + "," + k + ")");
            }
            for (int j = 0; j < data.vetexnum; j++) {
                if (data.arcs[i][j]==0) {
                    x[i][j] = null;
                }
                else{
                    //Xijk,公式(10)-(11)
                    for (int k = 0; k < data.vecnum; k++) {
                        x[i][j][k] = model.numVar(0, 1, IloNumVarType.Int, "x(" + i + "," + j + "," + k + ")");
                    }
                }
            }
        }

        //加入目标函数
        //公式(1)
        IloNumExpr obj = model.numExpr();
        for(int i = 0; i < data.vetexnum; i++){
            for(int j = 0; j < data.vetexnum; j++){
                if (data.arcs[i][j]==0) {
                    continue;
                }
                for(int k = 0; k < data.vecnum; k++){
                    obj = model.sum(obj, model.prod(data.dist[i][j], x[i][j][k]));
                }
            }
        }
        model.addMinimize(obj);

        //加入约束
        //公式(2)
        for(int i= 1; i < data.vetexnum-1;i++){
            IloNumExpr expr1 = model.numExpr();
            for (int k = 0; k < data.vecnum; k++) {
                for (int j = 1; j < data.vetexnum; j++) {
                    if (data.arcs[i][j]==1) {
                        expr1 = model.sum(expr1, x[i][j][k]);
                    }
                }
            }
            model.addEq(expr1, 1);
        }

        //公式(3)
        for (int k = 0; k < data.vecnum; k++) {
            IloNumExpr expr2 = model.numExpr();
            for (int j = 1; j < data.vetexnum; j++) {
                if (data.arcs[0][j]==1) {
                    expr2 = model.sum(expr2, x[0][j][k]);
                }
            }
            model.addEq(expr2, 1);
        }
        //公式(4)
        for (int k = 0; k < data.vecnum; k++) {
            for (int j = 1; j < data.vetexnum-1; j++) {
                IloNumExpr expr3 = model.numExpr();
                IloNumExpr subExpr1 = model.numExpr();
                IloNumExpr subExpr2 = model.numExpr();
                for (int i = 0; i < data.vetexnum; i++) {
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
        //公式(5)
        for (int k = 0; k < data.vecnum; k++) {
            IloNumExpr expr4 = model.numExpr();
            for (int i = 0; i < data.vetexnum-1; i++) {
                if (data.arcs[i][data.vetexnum-1]==1) {
                    expr4 = model.sum(expr4,x[i][data.vetexnum-1][k]);
                }
            }
            model.addEq(expr4, 1);
        }
        //公式(6)
        double M = 1e5;
        for (int k = 0; k < data.vecnum; k++) {
            for (int i = 0; i < data.vetexnum; i++) {
                for (int j = 0; j < data.vetexnum; j++) {
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
        //公式(7)
        for (int k = 0; k < data.vecnum; k++) {
            for (int i = 1; i < data.vetexnum-1; i++) {
                IloNumExpr expr7 = model.numExpr();
                for (int j = 0; j < data.vetexnum; j++) {
                    if (data.arcs[i][j] == 1) {
                        expr7 = model.sum(expr7,x[i][j][k]);
                    }
                }
                model.addLe(model.prod(data.a[i], expr7), w[i][k]);
                model.addLe(w[i][k], model.prod(data.b[i], expr7));
            }
        }
        //公式(8)
        for (int k = 0; k < data.vecnum; k++) {
            model.addLe(data.E, w[0][k]);
            model.addLe(data.E, w[data.vetexnum-1][k]);
            model.addLe(w[0][k], data.L);
            model.addLe(w[data.vetexnum-1][k], data.L);
        }
        //公式(9)
        for (int k = 0; k < data.vecnum; k++) {
            IloNumExpr expr8 = model.numExpr();
            for (int i = 1; i < data.vetexnum-1; i++) {
                IloNumExpr expr9 = model.numExpr();
                for (int j = 0; j < data.vetexnum; j++) {
                    if (data.arcs[i][j] == 1) {
                        expr9=model.sum(expr9,x[i][j][k]);
                    }
                }
                expr8 = model.sum(expr8,model.prod(data.demands[i],expr9));
            }
            model.addLe(expr8, data.cap);
        }
    }

    //函数功能：从txt文件中读取数据并初始化参数
    public static void process_solomon(String path, Data data,int vetexnum) throws Exception{
        String line = null;
        String[] substr = null;
        Scanner cin = new Scanner(new BufferedReader(new FileReader(path)));  //读取文件
        for(int i =0; i < 4;i++){
            line = cin.nextLine();  //读取一行
        }
        line = cin.nextLine();
        line.trim(); //返回调用字符串对象的一个副本，删除起始和结尾的空格
        substr = line.split(("\\s+")); //以空格为标志将字符串拆分
        //初始化参数
        vetexnum = vetexnum+2; //添加起点和终点
        data.vetexnum = vetexnum;
        data.vecnum = Integer.parseInt(substr[1]);
        data.cap = Integer.parseInt(substr[2]);
        data.vertexs =new int[data.vetexnum][2];				//所有点的坐标x,y
        data.demands = new int[data.vetexnum];					//需求量
        data.vehicles = new int[data.vecnum];					//车辆编号
        data.a = new double[data.vetexnum];						//时间窗开始时间
        data.b = new double[data.vetexnum];						//时间窗结束时间
        data.s = new double[data.vetexnum];						//服务时间
        data.arcs = new int[data.vetexnum][data.vetexnum];
        data.dist = new double[data.vetexnum][data.vetexnum];	//距离矩阵，满足三角关系,用距离表示cost
        for(int i =0; i < 4;i++){
            line = cin.nextLine();
        }
        //读取vetexnum-1行数据
        for (int i = 0; i < data.vetexnum - 1; i++) {
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
        cin.close();//关闭流
        //初始化配送中心参数
        data.vertexs[data.vetexnum-1] = data.vertexs[0];
        data.demands[data.vetexnum-1] = 0;
        data.a[data.vetexnum-1] = data.a[0];
        data.b[data.vetexnum-1] = data.b[0];
        data.E = data.a[0];
        data.L = data.b[0];
        data.s[data.vetexnum-1] = 0;
        double min1 = 1e15;
        double min2 = 1e15;
        //距离矩阵初始化
        for (int i = 0; i < data.vetexnum; i++) {
            for (int j = 0; j < data.vetexnum; j++) {
                if (i == j) {
                    data.dist[i][j] = 0;
                    continue;
                }
                data.dist[i][j] = Math.sqrt((data.vertexs[i][0]-data.vertexs[j][0])*(data.vertexs[i][0]-data.vertexs[j][0])+
                        (data.vertexs[i][1]-data.vertexs[j][1])*(data.vertexs[i][1]-data.vertexs[j][1]));
                data.dist[i][j]=data.double_truncate(data.dist[i][j]);
            }
        }
        data.dist[0][data.vetexnum-1] = 0;
        data.dist[data.vetexnum-1][0] = 0;
        //距离矩阵满足三角关系
        for (int  k = 0; k < data.vetexnum; k++) {
            for (int i = 0; i < data.vetexnum; i++) {
                for (int j = 0; j < data.vetexnum; j++) {
                    if (data.dist[i][j] > data.dist[i][k] + data.dist[k][j]) {
                        data.dist[i][j] = data.dist[i][k] + data.dist[k][j];
                    }
                }
            }
        }
        //初始化为完全图
        for (int i = 0; i < data.vetexnum; i++) {
            for (int j = 0; j < data.vetexnum; j++) {
                if (i != j) {
                    data.arcs[i][j] = 1;
                }
                else {
                    data.arcs[i][j] = 0;
                }
            }
        }
        //除去不符合时间窗和容量约束的边
        for (int i = 0; i < data.vetexnum; i++) {
            for (int j = 0; j < data.vetexnum; j++) {
                if (i == j) {
                    continue;
                }
                if (data.a[i]+data.s[i]+data.dist[i][j]>data.b[j] || data.demands[i]+data.demands[j]>data.cap) {
                    data.arcs[i][j] = 0;
                }
                if (data.a[0]+data.s[i]+data.dist[0][i]+data.dist[i][data.vetexnum-1]>data.b[data.vetexnum-1]) {
                    System.out.println("the calculating example is false");

                }
            }
        }
        for (int i = 1; i < data.vetexnum-1; i++) {
            if (data.b[i] - data.dist[0][i] < min1) {
                min1 = data.b[i] - data.dist[0][i];
            }
            if (data.a[i] + data.s[i] + data.dist[i][data.vetexnum-1] < min2) {
                min2 = data.a[i] + data.s[i] + data.dist[i][data.vetexnum-1];
            }
        }
        if (data.E > min1 || data.L < min2) {
            System.out.println("Duration false!");
            System.exit(0);//终止程序
        }
        //初始化配送中心0，n+1两点的参数
        data.arcs[data.vetexnum-1][0] = 0;
        data.arcs[0][data.vetexnum-1] = 1;
        for (int i = 1; i < data.vetexnum-1; i++) {
            data.arcs[data.vetexnum-1][i] = 0;
        }
        for (int i = 1; i < data.vetexnum-1; i++) {
            data.arcs[i][0] = 0;
        }
    }

    public static void main(String[] args) throws Exception {
        Data data = new Data();
        //读入不同的文件前要手动修改vetexnum参数，参数值等于所有点个数,不包括配送中心
        int vetexnum = 50;//所有点个数，不包括0，n+1两个配送中心点
        String path = "src/main/resources/Solomon/c101.txt";//算例地址
        // 根据输入的节点数和算例路径，读取数据
        process_solomon(path,data,vetexnum);
        System.out.println("input succesfully");
        System.out.println("cplex procedure###########################");

        //建立模型
        VRPTWSolver cplex = new VRPTWSolver(data);
        cplex.build_model();
        double cplex_time1 = System.nanoTime();
        cplex.solve();
        cplex.solution.fesible();
        double cplex_time2 = System.nanoTime();
        double cplex_time = (cplex_time2 - cplex_time1) / 1e9;//求解时间，单位s

        // 输出结果
        System.out.println("cplex_time " + cplex_time + " bestcost " + cplex.cost);
    }
}