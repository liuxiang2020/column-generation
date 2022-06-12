package com.operation.vrptw.lx;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;

// VRP问题的基础信息：运输距离（成本），运输时间，节点时间窗
public class VRPParam {
    public boolean verbose;
    public int randseed, mvehicle, nbClients, capacity, totalNode;
    // 用于求解子问题
    public double[][] cost;
    //用于分支定界算法的原始距离
    public double[][] distBase;
    //在分支定界算法中将会更新的距离
    public double[][] dist;
    //点与点之间的运输时间？？
    public double[][] ttime;
    //分支定界算法中每条边的权重
    public double[][] edges;
    //
    public double[] posX, posY, request, wval;
    //
    public int[][] shrink;
    //时间窗和服务时间
    public int[] openTime, closeTime, serviceTimeLength;
    //maxLength 最长路径的长度，并不准确
    public double maxTime, bigM, speed, gap, maxLength;
    //
    public boolean serviceInTimeWindow, debug = true;
    //
    String[] citiesLab, citiesLoc;

    public VRPParam(int nbclients) {
        gap = 1e-10;
        serviceInTimeWindow = false;
        this.nbClients = nbclients;
        // 节点数=顾客数+2
        this.totalNode = nbclients + 2;
        speed = 1;
        mvehicle = 0;
        bigM = 1E10;
    }

    public void initParams(String inputPath) throws IOException {

        try {
            System.out.println(System.getProperty("user.dir"));
            BufferedReader br = new BufferedReader(new FileReader(inputPath));
            String line;
            for (int i = 0; i < 4; i++) {
                line = br.readLine();
            }
            line = br.readLine();
            String[] tokens = line.split("\\s+");

            mvehicle = Integer.parseInt(tokens[1]);
            capacity = Integer.parseInt(tokens[2]);
            citiesLab = new String[nbClients + 2];
            request = new double[nbClients + 2];
            openTime = new int[nbClients + 2];
            closeTime = new int[nbClients + 2];
            serviceTimeLength = new int[nbClients + 2];
            posX = new double[nbClients + 2];
            posY = new double[nbClients + 2];
            distBase = new double[nbClients + 2][nbClients + 2];
            dist = new double[nbClients + 2][nbClients + 2];
            cost = new double[nbClients + 2][nbClients + 2];
            ttime = new double[nbClients + 2][nbClients + 2];

            for (int i = 0; i < 4; i++) {
                line = br.readLine();
            }

            for (int i = 0; i < nbClients + 1; i++) {
                line = br.readLine();
                tokens = line.split("\\s+");
                //城市编码
                citiesLab[i] = tokens[1];
                posX[i] = Double.parseDouble(tokens[2]);
                posY[i] = Double.parseDouble(tokens[3]);
                request[i] = Double.parseDouble(tokens[4]);
                openTime[i] = Integer.parseInt(tokens[5]);
                closeTime[i] = Integer.parseInt(tokens[6]);
                serviceTimeLength[i] = Integer.parseInt(tokens[7]);
            }
            br.close();
            // 预处理,降低后续加减操作
            if (serviceInTimeWindow) {
                for (int i = 0; i < nbClients + 1; i++) {
                    closeTime[i] -= serviceTimeLength[i];
                }
            }
            //末节点
            posX[nbClients + 1] = posX[0];
            posY[nbClients + 1] = posY[0];
            citiesLab[nbClients + 1] = citiesLab[0];
            request[nbClients + 1] = 0.0;
            openTime[nbClients + 1] = openTime[0];
            closeTime[nbClients + 1] = closeTime[0];
            serviceTimeLength[nbClients + 1] = 0;
            //总体最长路径的长度
            maxLength = 0.0;
            for (int i = 0; i < nbClients + 2; i++) {
                double bigNum = 0;
                for (int j = 0; j < nbClients + 2; j++) {
                    distBase[i][j] = ((int) (10 * Math.sqrt((posX[i] - posX[j]) * (posX[i] - posX[j]) + (posY[i] - posY[j]) * (posY[i] - posY[j])))) / 10.0;
                    if (bigNum < distBase[i][j]) {
                        bigNum = distBase[i][j];
                    }

                }
                maxLength += bigNum;
            }

            //更新部分路径的长度--还可以继续更新
            for (int i = 0; i < nbClients + 2; i++) {
                distBase[i][0] = bigM;
                distBase[nbClients + 1][i] = bigM;
                distBase[i][i] = bigM;
            }
            //
            for (int i = 0; i < nbClients + 2; i++) {
                for (int j = 0; j < nbClients + 2; j++) {
                    dist[i][j] = distBase[i][j];
                    //更新运输时间
                    ttime[i][j] = distBase[i][j] / speed;
                }
            }

            //初始化从仓库出发和到达仓库的线路的成本
            for (int j = 0; j < nbClients + 2; j++) {
                cost[0][j] = dist[0][j];
                cost[j][nbClients + 1] = dist[j][nbClients + 1];
            }
        } catch (IOException e) {
            System.err.println("Error: " + e);
        }
        // 下面几个属性暂时未知
        wval = new double[nbClients + 2];
        for (int i = 1; i < nbClients + 2; i++) {
            wval[i] = 0.0;
        }

        edges = new double[nbClients + 2][nbClients + 2];
    }
}
