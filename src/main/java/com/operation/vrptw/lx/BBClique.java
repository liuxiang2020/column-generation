package com.operation.vrptw.lx;

import java.util.Collections;
import java.util.LinkedList;

/**
 * 优先队列式分支限界法--最大团问题
 *
 * @author lican
 */
public class BBClique {
    public int[][] a;//图G的邻接矩阵
    public LinkedList<HeapNodes> heap;

    public BBClique(int[][] a) {
        this.a = a;
        heap = new LinkedList<HeapNodes>();
    }

    /**
     * 将当前构造出的活结点加入到子集空间树中并插入到活结点优先队列
     *
     * @param up
     * @param size
     * @param lev
     * @param par
     * @param ch
     */
    public void addLiveNode(int up, int size, int lev, BBnodes par, boolean ch) {
        BBnodes enode = new BBnodes(par, ch);
        HeapNodes h = new HeapNodes(enode, up, size, lev);
        heap.add(h);
        Collections.sort(heap);
    }

    /**
     * 对子集解空间树的最大优先队列分支限界搜索
     *
     * @param bestx 某点是否在最大团中
     * @return
     */
    public int bbMaxClique(int[] bestx) {
        int n = bestx.length - 1;

        //初始化(初始数据)
        BBnodes enode = null;
        int i = 1;
        int cn = 0;
        int bestn = 0;

        //搜索子集空间树
        while (i != n + 1) {//非叶节点
            boolean ok = true;
            BBnodes bnode = enode;
            for (int j = i - 1; j > 0; j--) {
                if (bnode.leftChild && a[i][j] == 0) {
                    ok = false;
                    break;
                }
                bnode = bnode.parent;
            }
            if (ok) {//左儿子结点为可行结点
                if (cn + 1 > bestn)
                    bestn = cn + 1;
                addLiveNode(cn + n - i + 1, cn + 1, i + 1, enode, true);
            }
            if (cn + n - i >= bestn) {//右子树可能含有最优解
                addLiveNode(cn + n - i, cn, i + 1, enode, false);
            }

            //取下一个扩展结点
            HeapNodes node = heap.poll();
            enode = node.liveNode;
            cn = node.cliqueSize;
            i = node.level;
        }

        //构造当前最优解
        for (int j = n; j > 0; j--) {
            bestx[j] = enode.leftChild ? 1 : 0;
            enode = enode.parent;
            System.out.print(bestx[j] + " ");
        }
        System.out.println();
        return bestn;
    }

    public static void main(String[] args) {
        int[][] a = {{-1, -1, -1, -1, -1, -1}, {-1, 0, 1, 0, 1, 1}, {-1, 1, 0, 1, 0, 1}, {-1, 0, 1, 0, 0, 1}, {-1, 1, 0, 0, 0, 1}, {-1, 1, 1, 1, 1, 0}};//a的下标从1开始，-1的值无用
        int n = 5;
        int[] bestx = new int[n + 1];
        BBClique b = new BBClique(a);
        System.out.println("图G的最大团解向量为：");
        int best = b.bbMaxClique(bestx);
        System.out.println("图G的最大团顶点数为：" + best);
    }
}


/**
 * 解空间树种结点类型为BBnodes
 *
 * @author Lican
 */
class BBnodes {
    BBnodes parent;//父节点
    boolean leftChild;//是否是左儿子

    public BBnodes(BBnodes par, boolean left) {
        this.parent = par;
        this.leftChild = left;
    }
}

/**
 * 活结点优先队列中元素类型为HeapNode
 *
 * @author Lican
 */
class HeapNodes implements Comparable {
    BBnodes liveNode;
    int upperSize;//当前团最大顶点数上界
    int cliqueSize;///当前团的顶点数
    int level;//

    public HeapNodes(BBnodes node, int up, int size, int lev) {
        liveNode = node;
        upperSize = up;
        cliqueSize = size;
        level = lev;
    }

    @Override
    public int compareTo(Object x) {//降序排序
        int ux = ((HeapNodes) x).upperSize;
        if (upperSize > ux) return -1;
        if (upperSize == ux) return 0;
        return 1;
    }
}
