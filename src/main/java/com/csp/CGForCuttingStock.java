package com.csp;

import ilog.concert.*;
import ilog.cplex.IloCplex;

import java.io.IOException;

public class CGForCuttingStock {
    private static double RC_EPS = 1.0e-6;

    private final CuttingStockModel model;

    private IloCplex restrictedMasterProblem;

    private IloObjective rmpObjective;

    private IloRange[] demandConstraint;

    private IloNumVarArray rmpVariable;

    private IloCplex subProblem;

    private IloObjective spObjective;

    private IloNumVar[] spVariable;

    public CGForCuttingStock(CuttingStockModel model) {
        this.model = model;
    }

    /**
     * 统计变量值与变量数量
     */
    static class IloNumVarArray {
        int _num           = 0;
        IloNumVar[] _array = new IloNumVar[32];

        void add(IloNumVar ivar) {
            if ( _num >= _array.length ) {
                IloNumVar[] array = new IloNumVar[2 * _array.length];
                System.arraycopy(_array, 0, array, 0, _num);
                _array = array;
            }
            _array[_num++] = ivar;
        }

        IloNumVar getElement(int i) { return _array[i]; }
        int       getSize()         { return _num; }
    }

    public void buildMasterProblemModel() throws IloException {
        /// CUTTING-OPTIMIZATION PROBLEM ///
        restrictedMasterProblem = new IloCplex();
        // 设置主问题目标函数
        rmpObjective = restrictedMasterProblem.addMinimize();
        //添加约束范围
        demandConstraint = new IloRange[model.demandArray.length];
        for (int f = 0; f < model.demandArray.length; f++ ) {
            demandConstraint[f] = restrictedMasterProblem.addRange(model.demandArray[f], Double.MAX_VALUE);
        }
        // 生成初始列，使限制主问题有可行解
        rmpVariable = new IloNumVarArray();

        for (int j = 0; j < model.cuttingSize; j++)  {
            //变量在目标函数系数为1.0
            IloColumn newColumn = restrictedMasterProblem.column(rmpObjective, 1);
            newColumn = newColumn.and(
                    restrictedMasterProblem.column(demandConstraint[j], (int)(model.rollWidth / model.cutSizeArray[j])));

            rmpVariable.add(restrictedMasterProblem.numVar(newColumn, 0.0, Double.MAX_VALUE));
        }
        restrictedMasterProblem.setParam(IloCplex.Param.RootAlgorithm, IloCplex.Algorithm.Primal);
    }

    public void buildSubProblemModel() throws IloException {
        subProblem = new IloCplex();
        spObjective = subProblem.addMinimize();
        // 切割各种尺寸木材的数量
        spVariable = subProblem.numVarArray(model.cuttingSize, 0., Double.MAX_VALUE, IloNumVarType.Int);
        subProblem.addRange(-Double.MAX_VALUE, subProblem.scalProd(model.cutSizeArray, spVariable), model.rollWidth);
        setSubProblemParameter();
    }

    public void solveMethod() throws IloException {
        try {
            buildMasterProblemModel();
            buildSubProblemModel();
            double[] newPlan;
            int iter = 1;
            for (; ; ) {
                System.out.printf("---------------------第%d次迭代-------------------------\n", iter++);
                // step 1: 求解主问题
                restrictedMasterProblem.solve();
                reportMainProblemLog(restrictedMasterProblem, rmpVariable, demandConstraint);
                // step 2: 更新子问题的目标函数，并求解
                double[] dualValueArray = restrictedMasterProblem.getDuals(demandConstraint);
                spObjective.setExpr(subProblem.diff(1., subProblem.scalProd(spVariable, dualValueArray)));
                subProblem.solve();
                reportSubProblemLog(subProblem, spVariable);
                // step 3: 判断是否退出循环
                if (subProblem.getObjValue() > -RC_EPS) {
                    System.out.printf("子问题的解（Reduced cost）小于%.4f, 退出循环\n", subProblem.getObjValue());
                    break;
                }
                // step 4: 获取子问题的解，并添加到主问题中
                addColumnsToRMP();

            }
            System.out.printf("经过%d次迭代，无新的列加入，将主问题转为整数规划模型，并求解\n", iter);
            // 设置主问题的变量为整数变量，获得最优解
            for (int i = 0; i < rmpVariable.getSize(); i++) {
                restrictedMasterProblem.add(restrictedMasterProblem.conversion(rmpVariable.getElement(i), IloNumVarType.Int));
            }
            restrictedMasterProblem.solve();
            reportFinalResult(restrictedMasterProblem, rmpVariable);
            System.out.println("Solution status: " + restrictedMasterProblem.getStatus());
            restrictedMasterProblem.end();
            subProblem.end();
        }catch ( IloException exc ) {
            System.err.println("Concert exception '" + exc + "' caught");
        }
    }

    /**
     * 解集池参数控制
     * @throws IloException
     */
    public void setSubProblemParameter() throws IloException {
        // https://www.ibm.com/docs/zh/icos/12.8.0.0?topic=parameters-solution-pool-replacement-strategy
        subProblem.setParam(IloCplex.Param.MIP.Pool.Replace, 2);
        // Relative objective gap.
        subProblem.setParam(IloCplex.Param.MIP.Pool.RelGap, 0.3);
        subProblem.setParam(IloCplex.Param.MIP.Pool.Capacity, 3);
        subProblem.setParam(IloCplex.Param.MIP.Limits.Populate, 3);
    }

    public void addColumnsToRMP() throws IloException {
        IloLPMatrix lp = (IloLPMatrix)subProblem.LPMatrixIterator().next();
        for (int i = 0; i < subProblem.getSolnPoolNsolns(); i++) {
            if(subProblem.getObjValue(i) <= - RC_EPS)
                addColumn(subProblem.getValues(lp, i));
            else
                subProblem.getValues(lp, i);
        }
    }
    public void addColumn(double[] newPlan) throws IloException {
        IloColumn column = restrictedMasterProblem.column(rmpObjective, 1.);
        for (int i = 0; i < model.cuttingSize; i++) {
            column = column.and(restrictedMasterProblem.column(demandConstraint[i], newPlan[i]));
        }
        rmpVariable.add(restrictedMasterProblem.numVar(column, 0.0, Double.MAX_VALUE));
    }

    static void reportMainProblemLog(IloCplex rmp, IloNumVarArray Cut, IloRange[] Fill) throws IloException {

        System.out.printf("  使用%.2f根木材\n", rmp.getObjValue());
        for (int j = 0; j < Cut.getSize(); j++) {
            System.out.printf("  使用第%d种切割方案的数量%.2f\n", j, rmp.getValue(Cut.getElement(j)));
        }
        System.out.println("  打印对偶变量的值");
        for (int i = 0; i < Fill.length; i++)
            System.out.println("  第" + i + "个约束对应的对偶变量值为" + rmp.getDual(Fill[i]));
        System.out.println();
    }
    //输出影子价格以及新的切法
    static void reportSubProblemLog(IloCplex patSolver, IloNumVar[] Use) throws IloException {
        System.out.println("子问题的目标函数值为 " + patSolver.getObjValue());
        if (patSolver.getObjValue() <= -RC_EPS) {
            for (int i = 0; i < Use.length; i++)
                if(patSolver.getValue(Use[i])>0)
                    System.out.printf("  切出第%d种木材的数量为%.0f\n" , i , patSolver.getValue(Use[i]));
            System.out.println();
        }
    }

    static void reportFinalResult(IloCplex cutSolver, IloNumVarArray Cut) throws IloException {
        System.out.printf("转化为整数规划模型后，使用的总木材的数量为%.0f\n" , cutSolver.getObjValue());
        for (int j = 0; j < Cut.getSize(); j++){
            if(cutSolver.getValue(Cut.getElement(j)) > 0.01)
                System.out.printf("  使用第%d种切割方案的数量%.2f\n", j, cutSolver.getValue(Cut.getElement(j)));
        }

    }

    public static void main(String[] args) throws InputDataReader.InputDataReaderException, IOException, IloException {
        String datafile = "C:\\Users\\刘祥\\Documents\\java\\columnGeneration\\src\\main\\resources\\data\\cutstock.dat";
//        CuttingStockModel cuttingStockModel = CuttingStockModel.readFromData(datafile);
        CuttingStockModel cuttingStockModel = CuttingStockModel.randomGenerate(8, 50);
        CGForCuttingStock CGForCuttingStock = new CGForCuttingStock(cuttingStockModel);
        CGForCuttingStock.solveMethod();
    }
}
