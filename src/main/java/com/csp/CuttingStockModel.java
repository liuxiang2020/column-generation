package com.csp;

import java.io.IOException;
import java.util.*;

public class CuttingStockModel {
    double RC_EPS = 1.0e-6;
    // Data of the problem rtb0  '   11
    /**
     * 原木材长度
     */
    double rollWidth;
    /**
     * 客户要求木材长度
     */
    double[] cutSizeArray;
    /**
     * 客户要求个木材的数量
     */
    double[] demandArray;

    int cuttingSize;

    public static CuttingStockModel readFromData(String fileName) throws InputDataReader.InputDataReaderException, IOException {
        CuttingStockModel cuttingStockModel = new CuttingStockModel();
        InputDataReader reader = new InputDataReader(fileName);
        cuttingStockModel.rollWidth = reader.readDouble();
        cuttingStockModel.cutSizeArray = reader.readDoubleArray();
        cuttingStockModel.demandArray = reader.readDoubleArray();
        cuttingStockModel.cuttingSize = cuttingStockModel.demandArray.length;
        return cuttingStockModel;
    }

    public static CuttingStockModel randomGenerate(int cuttingSize, double rollWidth){
        CuttingStockModel cuttingStockModel = new CuttingStockModel();
        Random random = new Random();
        cuttingStockModel.rollWidth = rollWidth;
        cuttingStockModel.cuttingSize = cuttingSize;
        cuttingStockModel.cutSizeArray = generateSizeArray(cuttingSize, 3, 20, random);
        cuttingStockModel.demandArray = generateDemandArray(cuttingSize, 20, 100, random);
        return cuttingStockModel;
    }

    public static double[] generateSizeArray(int size, int minValue, int maxValue, Random random){
        Set<Integer> sizeSet = new HashSet<>();
        while (sizeSet.size() < size){
            sizeSet.add(minValue+random.nextInt(maxValue-minValue));
        }
        double[] array = new double[size];
        int i=0;
        for(Integer val: sizeSet){
            array[i++] = val;
        }
        Arrays.sort(array);

        return array;
    }

    public static double[] generateDemandArray(int size, int minValue, int maxValue, Random random){
        double[] array = new double[size];
        for (int i = 0; i < size; i++) {
            array[i] = minValue+random.nextInt(maxValue-minValue);
        }
        return array;
    }

}
