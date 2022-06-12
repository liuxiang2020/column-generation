/* --------------------------------------------------------------------------
 * File: CutStock.java
 * Version 12.8.0  
 * --------------------------------------------------------------------------
 * Licensed Materials - Property of IBM
 * 5725-A06 5725-A29 5724-Y48 5724-Y49 5724-Y54 5724-Y55 5655-Y21
 * Copyright IBM Corporation 2001, 2017. All Rights Reserved.
 *
 * US Government Users Restricted Rights - Use, duplication or
 * disclosure restricted by GSA ADP Schedule Contract with
 * IBM Corp.
 * --------------------------------------------------------------------------
 */
package com.csp;

import ilog.concert.*;
import ilog.cplex.*;
import java.io.*;

class CutStock {
   static double RC_EPS = 1.0e-6;
   // Data of the problem rtb0  '   11
   static double rollWidth;
   static double[] size;
   static double[] demamd;

   static void readData(String fileName) throws IOException, InputDataReader.InputDataReaderException {
      InputDataReader reader = new InputDataReader(fileName);
      rollWidth = reader.readDouble();
      size = reader.readDoubleArray();
      demamd = reader.readDoubleArray();
   }

   /**
    * 输出共切了多少根17英尺的木材以及每种切法用了多少木材
    *
    * @param rmp
    * @param Cut
    * @param Fill
    * @throws IloException
    */
   static void reportMainProblemLog(IloCplex rmp, IloNumVarArray Cut, IloRange[] Fill) throws IloException {
      System.out.println();
      System.out.println("Using " + rmp.getObjValue() + " rolls");
      System.out.println();
      for (int j = 0; j < Cut.getSize(); j++) {
         System.out.println("  Cut" + j + " = " + rmp.getValue(Cut.getElement(j)));
      }
      System.out.println();
      
      for (int i = 0; i < Fill.length; i++) 
         System.out.println("  第" + i + "个约束的对偶变量为" + rmp.getDual(Fill[i]));
      System.out.println();
   }
   //输出影子价格以及新的切法
   static void reportSubProblemLog(IloCplex patSolver, IloNumVar[] Use) throws IloException {
      System.out.println();
      System.out.println("Reduced cost is " + patSolver.getObjValue());
      System.out.println();

      if (patSolver.getObjValue() <= -RC_EPS) {
         for (int i = 0; i < Use.length; i++) 
            System.out.println("  Use" + i + " = " + patSolver.getValue(Use[i]));
         System.out.println();
      }
   }

   /**
    * 输出最优切法所需木材数以及每种切法所需木材数
    * @param cutSolver
    * @param Cut
    * @throws IloException
    */
   static void reportFinalResult(IloCplex cutSolver, IloNumVarArray Cut) throws IloException {
      System.out.println();
      System.out.println("Best integer solution uses " + cutSolver.getObjValue() + " rolls");
      System.out.println();
      for (int j = 0; j < Cut.getSize(); j++) 
         System.out.println("  Cut" + j + " = " + cutSolver.getValue(Cut.getElement(j)));
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

   public static void main( String[] args ) {
      String datafile = "C:\\Users\\刘祥\\Documents\\java\\columnGeneration\\src\\main\\resources\\data\\cutstock.dat";
      try {
         if (args.length > 0)
            datafile = args[0];
         readData(datafile);
         
         /// CUTTING-OPTIMIZATION PROBLEM ///
         IloCplex restrictedMainProblem = new IloCplex();
         // 设置主问题目标函数
         IloObjective rmpObjective = restrictedMainProblem.addMinimize();
         //添加约束范围
         IloRange[]   demandConstraint = new IloRange[demamd.length];
         for (int f = 0; f < demamd.length; f++ ) {
            demandConstraint[f] = restrictedMainProblem.addRange(demamd[f], Double.MAX_VALUE);
         }

         // 生成初始列，使限制主问题有可行解
         IloNumVarArray cutSchemeVariableArray = new IloNumVarArray();
         int nWdth = size.length;

         for (int j = 0; j < nWdth; j++)  {
            //变量在目标函数系数为1.0
            IloColumn newColumn = restrictedMainProblem.column(rmpObjective, 1);
            newColumn = newColumn.and(restrictedMainProblem.column(demandConstraint[j], (int)(rollWidth / size[j])));
            cutSchemeVariableArray.add(restrictedMainProblem.numVar(newColumn, 0.0, Double.MAX_VALUE));
         }
         restrictedMainProblem.setParam(IloCplex.Param.RootAlgorithm, IloCplex.Algorithm.Primal);
       
         /// PATTERN-GENERATION PROBLEM ///
         IloCplex subProblem = new IloCplex();
         IloObjective reducedCost = subProblem.addMinimize();
         // 切割各种尺寸木材的数量
         IloNumVar[] use = subProblem.numVarArray(nWdth, 0., Double.MAX_VALUE, IloNumVarType.Int);
         subProblem.addRange(-Double.MAX_VALUE, subProblem.scalProd(size, use), rollWidth);
       
         /// COLUMN-GENERATION PROCEDURE ///
         double[] newPatt = new double[nWdth];
         /// COLUMN-GENERATION PROCEDURE ///
         double subObjective = -1.0;
         int iter = 1;
         for (;;) {
            System.out.printf("---------------------第%d次迭代-------------------------\n", iter++);
            /// OPTIMIZE OVER CURRENT PATTERNS ///
            restrictedMainProblem.solve();
            reportMainProblemLog(restrictedMainProblem, cutSchemeVariableArray, demandConstraint);
          
            /// FIND AND ADD A NEW PATTERN ///
            double[] price = restrictedMainProblem.getDuals(demandConstraint);
            reducedCost.setExpr(subProblem.diff(1., subProblem.scalProd(use, price)));
            subProblem.solve();
            reportSubProblemLog(subProblem, use);
            if ( subProblem.getObjValue() > -RC_EPS ){
               System.out.printf("子问题的解（Reduced cost）小于%.4f, 退出循环\n", subProblem.getObjValue());
               break;
            }

            // 获取子问题的解
            newPatt = subProblem.getValues(use);
            //添加新列
            IloColumn column = restrictedMainProblem.column(rmpObjective, 1.);
            for (int p = 0; p < newPatt.length; p++ )
               column = column.and(restrictedMainProblem.column(demandConstraint[p], newPatt[p]));
            cutSchemeVariableArray.add(restrictedMainProblem.numVar(column, 0., Double.MAX_VALUE) );
         }

         // 设置主问题的变量为整数变量，获得最优解
         for (int i = 0; i < cutSchemeVariableArray.getSize(); i++ ) {
            restrictedMainProblem.add(restrictedMainProblem.conversion(cutSchemeVariableArray.getElement(i), IloNumVarType.Int));
         }
         restrictedMainProblem.solve();
         reportFinalResult(restrictedMainProblem, cutSchemeVariableArray);
         System.out.println("Solution status: " + restrictedMainProblem.getStatus());
         restrictedMainProblem.end();
         subProblem.end();
      }
      catch ( IloException exc ) {
         System.err.println("Concert exception '" + exc + "' caught");
      }
      catch (IOException exc) {
         System.err.println("Error reading file " + datafile + ": " + exc);
      }
      catch (InputDataReader.InputDataReaderException exc ) {
         System.err.println(exc);
      }
   }
}


/* Example Input file:
115
[25, 40, 50, 55, 70]
[50, 36, 24, 8, 30]
*/
