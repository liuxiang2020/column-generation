package com.operation.vrptw.lx;

import java.util.ArrayList;

public class Route implements Cloneable{
    public double cost, Q;
    public ArrayList<Integer> path;

    public Route(){
        this.path = new ArrayList<>();
        this.cost = 0.0;
    }

    public Route(int pathSize){
        this.path = new ArrayList<>(pathSize);
        this.cost = 0.0;
    }

    //深拷贝
    public Route clone() throws CloneNotSupportedException{
        Route route = (Route) super.clone();
        route.path = (ArrayList<Integer>) path.clone();
        return route;
    }

    public void removeCity(int cityIndex){
        this.path.remove(Integer.valueOf(cityIndex));
    }

    public void addCity(int cityIndex){
        this.path.add(cityIndex);
    }

    public void addCityFromPath(int[] path){
        for(int cityIndex:path )
            this.path.add(cityIndex);
    }

    public void setCost(double cost1){
        this.cost = cost1;
    }

    public double getCost(){return this.cost;}

    public void setQ(double a) { this.Q = a; }

    public double getQ(){
        return this.Q;
    }

    public ArrayList<Integer> getPath(){return this.path;}

    // 逆转轨迹
    public void switchPath(){
        Integer swap;
        int nb = path.size()/2;
        for(int i=0; i<nb; i++){
            swap = path.get(i);
            path.set(i, path.get(path.size()-1-i));
            path.set(path.size()-1-i, swap);
        }
    }

}
