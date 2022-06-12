package com.operation.vrptw.lx;

import org.checkerframework.checker.units.qual.A;

import java.io.IOException;
import java.util.ArrayList;

// 调用主函数
public class Main {
    public static void main(String[] args) throws IOException {
        VRPParam instance = new VRPParam(25);
        instance.initParams("src/main/resources/Solomon/R101.txt");
        ArrayList<Route> initRoutes = new ArrayList<>();
        ArrayList<Route> bestRoutes = new ArrayList<>();
    }
}
