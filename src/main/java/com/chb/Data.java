package com.chb;

import java.io.BufferedReader;
import java.io.FileReader;
import java.util.Scanner;

//定义参数
class Data {
    public static final double gap = 1e-6;
    public static final double big_num = 100000;
    public static int pointnum = 102;      //所有点集合n（包括配送中心和客户点，首尾（0和n）为配送中心）
    public static double E;                //配送中心时间窗开始时间
    public static double L;                //配送中心时间窗结束时间
    public static int carnum;            //车辆数
    public static double cap;            //车辆载荷
    public static int[][] point = new int[pointnum][2];      //所有点的坐标x,y
    public static int[] demand = new int[pointnum];          //需求量
    public static int[] car = new int[carnum];               //车辆编号
    public static double[] a = new double[pointnum];         //时间窗开始时间【a[i],b[i]】
    public static double[] b = new double[pointnum];         //时间窗结束时间【a[i],b[i]】
    public static double[] s = new double[pointnum];         //客户点的服务时长
    public static int[][] arcs = new int[pointnum][pointnum];//arcs[i][j]表示i到j点的弧，arcs[i][j]=1表示其为可行弧，arcs[i][j]=0表示其为不可行弧
    public static double[][] dist = new double[pointnum][pointnum]; //距离矩阵，满足三角关系,暂用距离表示花费 C[i][j]=dist[i][j]

    //截断小数3.26434-->3.2
    public double double_truncate(double v) {
        int iv = (int) v;
        if (iv + 1 - v <= gap)
            return iv + 1;
        double dv = (v - iv) * 10;
        int idv = (int) dv;
        double rv = iv + idv / 10.0;
        return rv;
    }

    public Data() {
        super();
    }

    public void setPointNum(int nbclients) {
        pointnum = nbclients+2;
    }

    //函数功能：从txt文件中读取数据并初始化参数
    public void read_data(String path, Data data) throws Exception {
        String line = null;
        String[] substr = null;
        Scanner cin = new Scanner(new BufferedReader(new FileReader(path)));  //读取文件
        //读取1-4行
        for (int i = 0; i < 4; i++) {
            line = cin.nextLine();
        }
        //读取5行
        line = cin.nextLine();
        line.trim();
        substr = line.split(("\\s+"));
        carnum = Integer.parseInt(substr[1]);
        cap = Integer.parseInt(substr[2]);
        //读取6-9行
        for (int i = 0; i < 4; i++) {
            line = cin.nextLine();
        }
        //读取pointnum-1行数据
        for (int i = 0; i < pointnum - 1; i++) {
            line = cin.nextLine();
            line.trim();
            substr = line.split("\\s+");
            point[i][0] = Integer.parseInt(substr[2]);
            point[i][1] = Integer.parseInt(substr[3]);
            demand[i] = Integer.parseInt(substr[4]);
            a[i] = Integer.parseInt(substr[5]);
            b[i] = Integer.parseInt(substr[6]);
            s[i] = Integer.parseInt(substr[7]);
        }
        cin.close();//关闭流
        //初始化终点参数
        point[pointnum - 1] = point[0];
        demand[pointnum - 1] = 0;
        a[pointnum - 1] = a[0];
        b[pointnum - 1] = b[0];
        E = a[0];
        L = b[0];
        s[pointnum - 1] = 0;
        double min1 = 1e15;
        double min2 = 1e15;
        //距离矩阵初始化
        for (int i = 0; i < pointnum; i++) {
            for (int j = 0; j < pointnum; j++) {
                if (i == j) {
                    dist[i][j] = 0;
                    continue;
                }
                dist[i][j] = Math.sqrt(Math.pow(point[i][0] - point[j][0], 2) + Math.pow(point[i][1] - point[j][1], 2));
                dist[i][j] = double_truncate(dist[i][j]);
            }
        }
        dist[0][pointnum - 1] = 0;
        dist[pointnum - 1][0] = 0;
        //距离矩阵满足三角关系
        for (int k = 0; k < pointnum; k++) {
            for (int i = 0; i < pointnum; i++) {
                for (int j = 0; j < pointnum; j++) {
                    if (dist[i][j] > dist[i][k] + dist[k][j]) {
                        dist[i][j] = dist[i][k] + dist[k][j];
                    }
                }
            }
        }
        //初始化为完全图
        for (int i = 0; i < pointnum; i++) {
            for (int j = 0; j < pointnum; j++) {
                if (i != j) {
                    arcs[i][j] = 1;
                } else {
                    arcs[i][j] = 0;
                }
            }
        }
        //除去不符合时间窗和容量约束的边
        for (int i = 0; i < pointnum; i++) {
            for (int j = 0; j < pointnum; j++) {
                if (i == j) {
                    continue;
                }
                if (a[i] + s[i] + dist[i][j] > b[j] ||
                        demand[i] + demand[j] > cap) {
                    arcs[i][j] = 0;
                }
                if (a[0] + s[i] + dist[0][i] + dist[i][pointnum - 1] > b[pointnum - 1]) {
                    System.out.println("the calculating example is false");
                }
            }
        }
        for (int i = 1; i < pointnum - 1; i++) {
            if (b[i] - dist[0][i] < min1) {
                min1 = b[i] - dist[0][i];
            }
            if (a[i] + s[i] + dist[i][pointnum - 1] < min2) {
                min2 = a[i] + s[i] + dist[i][pointnum - 1];
            }
        }
        if (E > min1 || L < min2) {
            System.out.println("Duration false!");
            System.exit(0);//终止程序
        }
        //初始化配送中心0，n+1两点的参数
        arcs[pointnum - 1][0] = 0;
        arcs[0][pointnum - 1] = 1;
        for (int i = 1; i < pointnum - 1; i++) {
            arcs[pointnum - 1][i] = 0;
        }
        for (int i = 1; i < pointnum - 1; i++) {
            arcs[i][0] = 0;
        }
    }
}

