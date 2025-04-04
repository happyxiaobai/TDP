import Jama.CholeskyDecomposition;
import Jama.Matrix;
import gurobi.*;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.*;


/**
 * 分布鲁棒机会约束分区问题的解决方案
 * 实现基于D_1和D_2模糊集的DRICC问题以及基于Bonferroni近似的DRJCC模型
 * 使用Gurobi进行求解
 */
public class DistributionallyRobustAlgo {
    private Instance inst;
    private ArrayList<Area> centers; // 存储所有区域中心
    private ArrayList<Integer>[] zones; // 存储每个区域的基本单元编号
    private double r; // 活动指标平衡容差
    private double gamma; // 机会约束风险参数
    private double[][] scenarios; // 原始场景数据
    private int numScenarios; // 场景数量
    private int[][] scenarioDemands; // 存储所有场景下的需求
    private Random rand; // 随机数生成器

    // 分布鲁棒模型的参数
    private double[] meanVector; // 样本均值向量
    private double[][] covarianceMatrix; // 样本协方差矩阵
    private double delta1; // D_2模糊集参数
    private double delta2; // D_2模糊集参数
    private boolean useD1; // 是否使用D_1模糊集
    private boolean useJointChance; // 是否使用联合机会约束
    private double[] individualGammas; // Bonferroni近似的个体风险分配

    private double demandUpperBound; // 存储需求上限

    // 时间限制和迭代限制
    private int timeLimit = 1000; // 秒
    private int maxIterations = 100;

    /**
     * 构造函数
     */
    public DistributionallyRobustAlgo(Instance instance, double[][] scenarios, double gamma,
                                      long seed, boolean useD1, double delta1, double delta2, boolean useJointChance) {
        this.inst = instance;
        this.scenarios = scenarios;
        this.numScenarios = scenarios.length;
        this.scenarioDemands = new int[numScenarios][inst.getN()];
        for (int s = 0; s < numScenarios; s++) {
            for (int i = 0; i < inst.getN(); i++) {
                this.scenarioDemands[s][i] = (int) scenarios[s][i];
            }
        }
        this.gamma = gamma;
        this.r = 0.1; // 活动指标平衡容差
        this.rand = new Random(seed);
        this.zones = new ArrayList[inst.k];
        this.useD1 = useD1;
        this.delta1 = delta1;
        this.delta2 = delta2;
        this.useJointChance = useJointChance;

        // 计算样本均值和协方差矩阵
        calculateMomentInformation();
        double totalMeanDemand = 0;
        for (int i = 0; i < inst.getN(); i++) {
            totalMeanDemand += meanVector[i];
        }
        this.demandUpperBound = (1 + r) * (totalMeanDemand / inst.k);

        // 如果使用Bonferroni近似，初始化个体风险分配
        if (useJointChance) {
            this.individualGammas = new double[inst.k];
            // 简单均分风险
            for (int j = 0; j < inst.k; j++) {
                individualGammas[j] = gamma / inst.k;
            }
        }
    }

    /**
     * 主要求解方法
     */
    public void run(String filename) throws IOException, GRBException {
        long startTime = System.currentTimeMillis();
        double Best = Double.MAX_VALUE;
        ArrayList<Integer>[] BestZones = new ArrayList[inst.k];

        // Step 1: 构造初始区域中心集合
        ArrayList<Integer> initialCenters = selectInitialCenters();
        centers = new ArrayList<>();
        for (int centerId : initialCenters) {
            centers.add(inst.getAreas()[centerId]);
            inst.getAreas()[centerId].setCenter(true);
        }

        // Step 2: 生成初始可行解
        boolean feasible = generateInitialSolution();

        if (!feasible) {
            System.out.println("无法找到可行解，请检查模型参数");
            return;
        }

        // Step 3: 改善初始解
        boolean change = true;
        double cur_value = evaluateObjective();
        int iteration = 0;

        while (change && iteration < maxIterations) {
            iteration++;
            change = false;

            // 检查每个区域的真正中心
            ArrayList<Area> newCenters = findTrueCenters();

            // 如果区域中心发生变化，更新并重新求解
            if (!compareCenters(centers, newCenters)) {
                centers = newCenters;
                change = true;
                feasible = generateInitialSolution();
                if (feasible) {
                    cur_value = evaluateObjective();
                    System.out.println("Iteration " + iteration + ": 目标函数值 = " + cur_value);
                } else {
                    System.out.println("Iteration " + iteration + ": 无法找到可行解");
                    break;
                }
            }
        }

        // 确保连通性
        if (feasible) {
            ensureConnectivity();
            cur_value = evaluateObjective();
        }

        // 评估最终结果
        if (feasible && cur_value < Best) {
            Best = cur_value;
            for (int z = 0; z < inst.k; z++) {
                BestZones[z] = new ArrayList<>();
                if (zones[z] != null) {
                    BestZones[z].addAll(zones[z]);
                }
            }
        }

        long endTime = System.currentTimeMillis();
        double timeSpentInSeconds = (endTime - startTime) / 1000.0;

        // 输出结果
        String outputFilePath = "./output/" + filename.replace(".dat", "_drcc.txt");
        FileWriter writer = new FileWriter(outputFilePath);
        BufferedWriter buffer = new BufferedWriter(writer);

        for (int io = 0; io < BestZones.length; io++) {
            if (BestZones[io] != null) {
                buffer.write("center ID: " + centers.get(io).getId() + "\n");
                for (int jo = 0; jo < BestZones[io].size(); jo++) {
                    buffer.write(BestZones[io].get(jo) + " ");
                }
                buffer.newLine();
            }
        }

        String result = String.format("%.2f", Best);
        buffer.write("best objective: " + result + "\n");
        buffer.write("程序运行时间为：" + timeSpentInSeconds + "s" + "\n");
        buffer.write("模糊集类型：" + (useD1 ? "D_1" : "D_2") + "\n");
        if (!useD1) {
            buffer.write("delta1: " + delta1 + ", delta2: " + delta2 + "\n");
        }
        buffer.write("约束类型：" + (useJointChance ? "联合约束(DRJCC)" : "个体约束(DRICC)") + "\n");
        buffer.write("风险参数：" + gamma + "\n");
        buffer.close();

        System.out.println("程序运行时间为：" + timeSpentInSeconds + "s");
        System.out.println("最终目标函数值：" + Best);
    }

    /**
     * 计算样本矩信息（均值和协方差矩阵）
     */
    private void calculateMomentInformation() {
        int n = inst.getN();

        // 计算均值向量
        meanVector = new double[n];
        for (int i = 0; i < n; i++) {
            double sum = 0;
            for (int s = 0; s < numScenarios; s++) {
                sum += scenarios[s][i];
            }
            meanVector[i] = sum / numScenarios;
        }

        // 计算协方差矩阵
        covarianceMatrix = new double[n][n];
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                double covariance = 0;
                for (int s = 0; s < numScenarios; s++) {
                    covariance += (scenarios[s][i] - meanVector[i]) * (scenarios[s][j] - meanVector[j]);
                }
                covarianceMatrix[i][j] = covariance / numScenarios;
            }
        }

        // 检查矩阵是否对称
        boolean isSymmetric = isSymmetric(covarianceMatrix);
        System.out.println("协方差矩阵是否对称: " + isSymmetric);

        // 检查矩阵是否半正定
        boolean isPSD = isPSDByEigenvalues();
        if (!isPSD) {
            System.out.println("警告: 协方差矩阵不是半正定的，正在尝试修正...");
            ensurePSDMatrix();

            boolean isPSDAfterFix = isPSDByEigenvalues();
            System.out.println("修正后矩阵是否半正定: " + isPSDAfterFix);
        }
    }

    private boolean isSymmetric(double[][] matrix) {
        for (int i = 0; i < matrix.length; i++) {
            for (int j = i + 1; j < matrix.length; j++) {
                double tolerance = 1e-10;
                if (Math.abs(matrix[i][j] - matrix[j][i]) > tolerance) {
                    return false;
                }
            }
        }
        return true;
    }

    private boolean isPSDByEigenvalues() {
        int n = covarianceMatrix.length;

        try {
            Matrix matrix = new Matrix(covarianceMatrix);
            Jama.EigenvalueDecomposition eig = matrix.eig();
            double[] eigenvalues = eig.getRealEigenvalues();

            double minEigenvalue = Double.MAX_VALUE;
            for (double ev : eigenvalues) {
                minEigenvalue = Math.min(minEigenvalue, ev);
            }

            double epsilon = 1e-15; // 数值稳定性的容差
            System.out.println("最小特征值: " + minEigenvalue);

            return minEigenvalue >= -epsilon;
        } catch (java.lang.Exception e) {
            System.out.println("特征值分解失败: " + e.getMessage());
            return false;
        }
    }

    private void ensurePSDMatrix() {
        // 在对角线上添加一个小的正数
        double epsilon = 1e-5; // 略大于最小负特征值的绝对值
        for (int i = 0; i < covarianceMatrix.length; i++) {
            covarianceMatrix[i][i] += epsilon;
        }
    }

    /**
     * 选择初始区域中心
     */
    private ArrayList<Integer> selectInitialCenters() throws GRBException {
        // 这部分代码保持不变
        int InitialNum = 5; // 可调整的初始场景数
        ArrayList<Integer> candidateCenters = new ArrayList<>();
        HashMap<Integer, Integer> centerFrequency = new HashMap<>();

        int scenariosProcessed = 0;
        while (scenariosProcessed < InitialNum && scenariosProcessed < numScenarios) {
            int scenarioIndex = rand.nextInt(numScenarios);

            // 使用该场景的需求求解确定性模型
            ArrayList<Integer> scenarioCenters = solveForScenario(scenarioIndex);

            if (scenarioCenters.size() == inst.k) {
                scenariosProcessed++;
                System.out.println("处理场景 " + scenariosProcessed + "/" + InitialNum + "，场景索引: " + scenarioIndex);

                // 更新中心频率
                for (int center : scenarioCenters) {
                    centerFrequency.put(center, centerFrequency.getOrDefault(center, 0) + 1);
                }
            }
        }

        // 按频率排序选择前k个中心
        List<Map.Entry<Integer, Integer>> sortedCenters = new ArrayList<>(centerFrequency.entrySet());
        sortedCenters.sort(Map.Entry.<Integer, Integer>comparingByValue().reversed());

        for (int i = 0; i < Math.min(inst.k, sortedCenters.size()); i++) {
            candidateCenters.add(sortedCenters.get(i).getKey());
        }

        // 如果中心数量不足，随机补充
        while (candidateCenters.size() < inst.k) {
            int randomCenter = rand.nextInt(inst.getN());
            if (!candidateCenters.contains(randomCenter)) {
                candidateCenters.add(randomCenter);
            }
        }

        return candidateCenters;
    }

    private Instance createScenarioInstance(int scenarioIndex) {
        return new Instance(inst, scenarioDemands[scenarioIndex]);
    }

    /**
     * 求解单一场景的确定性模型
     */
    private ArrayList<Integer> solveForScenario(int scenarioIndex) throws GRBException {
        // 设置确定性场景的求解时间限制
        int localTimeLimit = 60; // 秒

        try {
            // 创建基于特定场景需求的实例
            Instance scenarioInstance = createScenarioInstance(scenarioIndex);

            // 创建Algo对象并设置时间限制
            Algo algo = new Algo(scenarioInstance);
            algo.setTimeLimit(localTimeLimit);

            // 获取该场景下的求解结果中心点
            //TODO 函数内部可以修改平衡约束的不等式，当前同时存在大于等于和小于等于
            //TODO 对于场景无法准确求解的情况，应该随机选择一个新的场景进行尝试，这里需要修改
            ArrayList<Integer> scenarioCenters = algo.getCorrectSolutionCenters();

            // 如果算法未能返回足够的中心点，则随机补充
            if (scenarioCenters.size() < inst.k) {
                // 随机补充中心点
                Set<Integer> centerSet = new HashSet<>(scenarioCenters);
                while (centerSet.size() < inst.k) {
                    int candidate = rand.nextInt(inst.getN());
                    if (!centerSet.contains(candidate)) {
                        centerSet.add(candidate);
                        scenarioCenters.add(candidate);
                    }
                }
                System.out.println("场景 " + scenarioIndex + " 中心点不足，随机补充至 " + scenarioCenters.size() + " 个");
            }

            return scenarioCenters;
        } catch (Exception e) {
            System.out.println("求解场景 " + scenarioIndex + " 时出错: " + e.getMessage());

            // 出错时使用随机选择作为备选方案
            ArrayList<Integer> fallbackCenters = new ArrayList<>();
            Set<Integer> centerSet = new HashSet<>();

            while (centerSet.size() < inst.k) {
                int candidate = rand.nextInt(inst.getN());
                if (!centerSet.contains(candidate)) {
                    centerSet.add(candidate);
                    fallbackCenters.add(candidate);
                }
            }

            System.out.println("场景 " + scenarioIndex + " 求解失败，使用随机选择了 " + fallbackCenters.size() + " 个中心点");
            return fallbackCenters;
        }
    }

    /**
     * 生成初始可行解 - 使用Gurobi
     */
    private boolean generateInitialSolution() throws GRBException {
        try {
            // 创建Gurobi环境和模型
            GRBEnv env = new GRBEnv(true);
            env.set(GRB.IntParam.OutputFlag, 0);
            env.set(GRB.IntParam.LogToConsole, 0);
            env.set(GRB.StringParam.LogFile, "");
            env.set(GRB.IntParam.Seed, 42);
            env.start();
            GRBModel model = new GRBModel(env);

            // 设置Gurobi参数
            model.set(GRB.IntParam.OutputFlag, 0);
            model.set(GRB.DoubleParam.TimeLimit, timeLimit);

            // 决策变量 x_ij
            GRBVar[][] x = new GRBVar[inst.getN()][centers.size()];
            for (int i = 0; i < inst.getN(); i++) {
                for (int j = 0; j < centers.size(); j++) {
                    x[i][j] = model.addVar(0, 1, 0, GRB.BINARY, "x_" + i + "_" + centers.get(j).getId());
                    if (i == centers.get(j).getId()) {
                        // 将中心点固定为1
                        model.addConstr(x[i][j], GRB.EQUAL, 1.0, "center_" + i + "_" + j);
                    }
                }
            }

            // 约束: 每个基本单元必须且只能属于一个区域
            for (int i = 0; i < inst.getN(); i++) {
                GRBLinExpr rowSum = new GRBLinExpr();
                for (int j = 0; j < centers.size(); j++) {
                    rowSum.addTerm(1.0, x[i][j]);
                }
                model.addConstr(rowSum, GRB.EQUAL, 1.0, "unit_" + i);
            }

            // 添加分布鲁棒机会约束
            addDistributionallyRobustConstraints(model, x);

            // 目标函数: 最小化总距离
            GRBLinExpr obj = new GRBLinExpr();
            for (int i = 0; i < inst.getN(); i++) {
                for (int j = 0; j < centers.size(); j++) {
                    obj.addTerm(inst.dist[i][centers.get(j).getId()], x[i][j]);
                }
            }
            model.setObjective(obj, GRB.MINIMIZE);

            // 求解模型
            model.optimize();

            // 检查模型状态
            if (model.get(GRB.IntAttr.Status) != GRB.OPTIMAL &&
                    model.get(GRB.IntAttr.Status) != GRB.SUBOPTIMAL) {
                model.dispose();
                env.dispose();
                return false;
            }

            // 提取解决方案
            for (int j = 0; j < centers.size(); j++) {
                zones[j] = new ArrayList<>();
                for (int i = 0; i < inst.getN(); i++) {
                    if (Math.abs(x[i][j].get(GRB.DoubleAttr.X) - 1.0) < 1e-6) {
                        zones[j].add(i);
                    }
                }
            }

            // 释放资源
            model.dispose();
            env.dispose();

            return true;
        } catch (GRBException e) {
            System.out.println("Gurobi求解模型时发生错误: " + e.getMessage());
            return false;
        }
    }

    /**
     * 添加分布鲁棒机会约束 - 使用Gurobi实现
     */
    private void addDistributionallyRobustConstraints(GRBModel model, GRBVar[][] x) throws GRBException {
        if (useJointChance) {
            // 使用Bonferroni近似的DRJCC模型
            for (int j = 0; j < centers.size(); j++) {
                // 根据DRICC添加个体约束，使用individualGammas[j]作为风险参数
                addDRICCConstraint(model, x, j, individualGammas[j]);
            }
        } else {
            // 直接使用DRICC模型
            for (int j = 0; j < centers.size(); j++) {
                addDRICCConstraint(model, x, j, gamma);
            }
        }
    }

    /**
     * 添加DRICC约束 - 使用Gurobi实现
     */
    private void addDRICCConstraint(GRBModel model, GRBVar[][] x, int j, double riskParam) throws GRBException {
        double U = demandUpperBound; // 区域容量上限

        // 构建均值项
        GRBLinExpr meanTerm = new GRBLinExpr();
        for (int i = 0; i < inst.getN(); i++) {
            meanTerm.addTerm(meanVector[i], x[i][j]);
        }

        double factor;
        if (useD1) {
            // 使用D_1模糊集的约束
            // μ^T*x_j + sqrt((1-γ)/γ)*sqrt(x_j^T*Σ*x_j) ≤ U
            factor = Math.sqrt((1 - riskParam) / riskParam);
        } else {
            // 使用D_2模糊集的约束
            if (delta1 / delta2 <= riskParam) {
                // 第一种情况
                factor = Math.sqrt(delta1) + Math.sqrt((1 - riskParam) / riskParam * (delta2 - delta1));
            } else {
                // 第二种情况
                factor = Math.sqrt(delta2 / riskParam);
            }
        }

        // 创建辅助变量t
        GRBVar t = model.addVar(0, GRB.INFINITY, 0, GRB.CONTINUOUS, "t_" + j);

        // 使用Cholesky分解获取矩阵平方根
        Matrix covMatrix = new Matrix(covarianceMatrix);
        CholeskyDecomposition chol = new CholeskyDecomposition(covMatrix);

        // 如果Cholesky分解成功
        if (chol.isSPD()) {
            Matrix cholL = chol.getL();
            double[][] L = cholL.getArray();

            // 创建SOCP约束: ||Ax|| <= t
            // 在Gurobi中，可以使用二次约束x'A'Ax <= t^2来表示

            // 首先计算Ax的每个分量
            GRBVar[] z = new GRBVar[inst.getN()];
            for (int i = 0; i < inst.getN(); i++) {
                z[i] = model.addVar(-GRB.INFINITY, GRB.INFINITY, 0, GRB.CONTINUOUS, "z_" + j + "_" + i);

                GRBLinExpr zExpr = new GRBLinExpr();
                for (int k = 0; k < inst.getN(); k++) {
                    zExpr.addTerm(L[i][k], x[k][j]);
                }
                model.addConstr(z[i], GRB.EQUAL, zExpr, "z_def_" + j + "_" + i);
            }

            // 创建SOCP约束: sum(z_i^2) <= t^2
            GRBQuadExpr socpExpr = new GRBQuadExpr();
            for (int i = 0; i < inst.getN(); i++) {
                socpExpr.addTerm(1.0, z[i], z[i]);
            }

            // t^2
            GRBQuadExpr tSquare = new GRBQuadExpr();
            tSquare.addTerm(1.0, t, t);

            model.addQConstr(socpExpr, GRB.LESS_EQUAL, tSquare, "socp_" + j);

            // 添加均值+因子*t ≤ U的线性约束
            GRBLinExpr finalExpr = new GRBLinExpr();
            finalExpr.add(meanTerm);
            finalExpr.addTerm(factor, t);
            model.addConstr(finalExpr, GRB.LESS_EQUAL, U, "demand_" + j);
        } else {
            // 如果Cholesky分解失败，尝试使用对角化方法
            System.out.println("警告：Cholesky分解失败，尝试使用替代方法");

            // 创建二次表达式
            GRBQuadExpr quadExpr = new GRBQuadExpr();
            for (int i = 0; i < inst.getN(); i++) {
                for (int k = 0; k < inst.getN(); k++) {
                    quadExpr.addTerm(covarianceMatrix[i][k], x[i][j], x[k][j]);
                }
            }

            // 创建辅助变量q表示二次项
            GRBVar q = model.addVar(0, GRB.INFINITY, 0, GRB.CONTINUOUS, "q_" + j);

            // 添加约束 quadExpr <= q
            model.addQConstr(quadExpr, GRB.LESS_EQUAL, q, "quad_" + j);

            // 添加约束 q <= t^2
            GRBQuadExpr tSquare = new GRBQuadExpr();
            tSquare.addTerm(1.0, t, t);
            model.addQConstr(q, GRB.LESS_EQUAL, tSquare, "t_square_" + j);

            // 添加均值+因子*t ≤ U的线性约束
            GRBLinExpr finalExpr = new GRBLinExpr();
            finalExpr.add(meanTerm);
            finalExpr.addTerm(factor, t);
            model.addConstr(finalExpr, GRB.LESS_EQUAL, U, "demand_" + j);
        }
    }

    /**
     * 查找每个区域的真正中心
     */
    private ArrayList<Area> findTrueCenters() {
        ArrayList<Area> newCenters = new ArrayList<>();

        for (int j = 0; j < centers.size(); j++) {
            if (zones[j] == null || zones[j].isEmpty()) {
                newCenters.add(centers.get(j));
                continue;
            }

            int bestCenter = -1;
            double minTotalDist = Double.MAX_VALUE;

            for (int i : zones[j]) {
                double totalDist = 0;
                for (int k : zones[j]) {
                    totalDist += inst.dist[i][k];
                }

                if (totalDist < minTotalDist) {
                    minTotalDist = totalDist;
                    bestCenter = i;
                }
            }

            newCenters.add(inst.getAreas()[bestCenter]);
        }

        return newCenters;
    }

    /**
     * 比较两组中心是否相同
     */
    private boolean compareCenters(ArrayList<Area> centers1, ArrayList<Area> centers2) {
        if (centers1.size() != centers2.size()) {
            return false;
        }

        for (int i = 0; i < centers1.size(); i++) {
            if (centers1.get(i).getId() != centers2.get(i).getId()) {
                return false;
            }
        }

        return true;
    }

    /**
     * 确保每个区域的连通性 - 使用Gurobi实现
     */
    private void ensureConnectivity() throws GRBException {
        boolean allConnected = false;
        int iteration = 0;
        int maxIterations = 1000; // 限制迭代次数

        try {
            GRBEnv env = new GRBEnv(true);
            env.set(GRB.IntParam.OutputFlag, 0);
            env.set(GRB.IntParam.LogToConsole, 0);
            env.set(GRB.StringParam.LogFile, "");
            env.set(GRB.IntParam.Seed, 42);
            env.start();
            GRBModel model = new GRBModel(env);

            // 设置Gurobi参数
            model.set(GRB.IntParam.OutputFlag, 0);
            model.set(GRB.DoubleParam.TimeLimit, timeLimit);

            // 决策变量 x_ij
            GRBVar[][] x = new GRBVar[inst.getN()][centers.size()];
            for (int i = 0; i < inst.getN(); i++) {
                for (int j = 0; j < centers.size(); j++) {
                    x[i][j] = model.addVar(0, 1, 0, GRB.BINARY, "x_" + i + "_" + centers.get(j).getId());
                    if (i == centers.get(j).getId()) {
                        model.addConstr(x[i][j], GRB.EQUAL, 1.0, "center_" + i + "_" + j);
                    }
                }
            }

            // 约束: 每个基本单元必须且只能属于一个区域
            for (int i = 0; i < inst.getN(); i++) {
                GRBLinExpr rowSum = new GRBLinExpr();
                for (int j = 0; j < centers.size(); j++) {
                    rowSum.addTerm(1.0, x[i][j]);
                }
                model.addConstr(rowSum, GRB.EQUAL, 1.0, "unit_" + i);
            }

            // 添加分布鲁棒机会约束
            addDistributionallyRobustConstraints(model, x);

            // 目标函数: 最小化总距离
            GRBLinExpr obj = new GRBLinExpr();
            for (int i = 0; i < inst.getN(); i++) {
                for (int j = 0; j < centers.size(); j++) {
                    obj.addTerm(inst.dist[i][centers.get(j).getId()], x[i][j]);
                }
            }
            model.setObjective(obj, GRB.MINIMIZE);

            // 记录已添加的约束总数
            int totalConstraints = 0;

            while (!allConnected && iteration < maxIterations) {
                iteration++;

                // 求解模型
                model.optimize();

                // 检查模型状态
                if (model.get(GRB.IntAttr.Status) != GRB.OPTIMAL &&
                        model.get(GRB.IntAttr.Status) != GRB.SUBOPTIMAL) {
                    System.out.println("连通性处理迭代 " + iteration + " 失败，模型无解");
                    break;
                }

                // 检查所有区域是否连通
                boolean hasDisconnection = false;
                Map<Integer, List<ArrayList<Integer>>> allDisconnectedComponents = new HashMap<>();

                // 提取当前解
                if (iteration > 1) {
                    for (int j = 0; j < centers.size(); j++) {
                        zones[j] = new ArrayList<>();
                        for (int i = 0; i < inst.getN(); i++) {
                            if (Math.abs(x[i][j].get(GRB.DoubleAttr.X) - 1.0) < 1e-6) {
                                zones[j].add(i);
                            }
                        }
                    }
                }

                // 对每个区域检查连通性
                for (int j = 0; j < centers.size(); j++) {
                    ArrayList<ArrayList<Integer>> components = findConnectedComponents(zones[j]);

                    if (components.size() > 1) {
                        hasDisconnection = true;

                        // 找出中心所在的连通组件
                        int centerComponentIndex = -1;
                        for (int c = 0; c < components.size(); c++) {
                            if (components.get(c).contains(centers.get(j).getId())) {
                                centerComponentIndex = c;
                                break;
                            }
                        }

                        // 保存不包含中心的连通组件
                        List<ArrayList<Integer>> disconnectedComponents = new ArrayList<>();
                        for (int c = 0; c < components.size(); c++) {
                            if (c != centerComponentIndex) {
                                disconnectedComponents.add(components.get(c));
                            }
                        }

                        if (!disconnectedComponents.isEmpty()) {
                            allDisconnectedComponents.put(j, disconnectedComponents);
                        }
                    }
                }

                if (!hasDisconnection) {
                    allConnected = true;
                    continue;
                }

                // 添加连通性约束
                int constraintCounter = 0;
                for (Map.Entry<Integer, List<ArrayList<Integer>>> entry : allDisconnectedComponents.entrySet()) {
                    int districtIndex = entry.getKey();
                    List<ArrayList<Integer>> disconnectedComponents = entry.getValue();

                    for (ArrayList<Integer> component : disconnectedComponents) {
                        // 找出该组件的邻居
                        HashSet<Integer> neighbors = new HashSet<>();
                        for (int node : component) {
                            for (int neighbor : inst.getAreas()[node].getNeighbors()) {
                                if (!component.contains(neighbor)) {
                                    neighbors.add(neighbor);
                                }
                            }
                        }

                        // 添加约束: 组件中的所有节点都分配给区域districtIndex，或至少有一个邻居也分配给该区域
                        GRBLinExpr constrExpr = new GRBLinExpr();

                        // 对所有的邻居节点
                        for (int neighbor : neighbors) {
                            constrExpr.addTerm(1.0, x[neighbor][districtIndex]);
                        }

                        // 对当前组件中的所有节点
                        for (int node : component) {
                            constrExpr.addTerm(-1.0, x[node][districtIndex]);
                        }

                        model.addConstr(constrExpr, GRB.GREATER_EQUAL, 1 - component.size(), "conn_" + districtIndex + "_" + constraintCounter);
                        constraintCounter++;
                        totalConstraints++;
                    }
                }

                System.out.println("连通性处理迭代 " + iteration + " 完成，添加了 " + constraintCounter + " 个连通性约束");
            }

            model.optimize();

            // 最后一次提取解决方案
            for (int j = 0; j < centers.size(); j++) {
                zones[j] = new ArrayList<>();
                for (int i = 0; i < inst.getN(); i++) {
                    if (Math.abs(x[i][j].get(GRB.DoubleAttr.X) - 1.0) < 1e-6) {
                        zones[j].add(i);
                    }
                }
            }

            if (!allConnected) {
                System.out.println("警告：在最大迭代次数内未能保证所有区域的连通性");
            }

            // 释放资源
            model.dispose();
            env.dispose();

        } catch (GRBException e) {
            System.out.println("确保连通性时Gurobi错误: " + e.getMessage());
            e.printStackTrace();
        }
    }

    /**
     * 找出一个区域内的所有连通组件
     */
    private ArrayList<ArrayList<Integer>> findConnectedComponents(ArrayList<Integer> zone) {
        ArrayList<ArrayList<Integer>> components = new ArrayList<>();
        boolean[] visited = new boolean[inst.getN()];

        for (int i : zone) {
            if (!visited[i]) {
                ArrayList<Integer> component = new ArrayList<>();
                Queue<Integer> queue = new LinkedList<>();

                queue.add(i);
                visited[i] = true;

                while (!queue.isEmpty()) {
                    int current = queue.poll();
                    component.add(current);

                    for (int neighbor : inst.getAreas()[current].getNeighbors()) {
                        if (zone.contains(neighbor) && !visited[neighbor]) {
                            queue.add(neighbor);
                            visited[neighbor] = true;
                        }
                    }
                }

                components.add(component);
            }
        }

        return components;
    }

    /**
     * 计算当前解的目标函数值
     */
    private double evaluateObjective() {
        double totalDist = 0;

        for (int j = 0; j < centers.size(); j++) {
            if (zones[j] != null) {
                for (int i : zones[j]) {
                    totalDist += inst.dist[i][centers.get(j).getId()];
                }
            }
        }

        return totalDist;
    }

    /**
     * 获取区域
     */
    public ArrayList<Integer>[] getZones() {
        return zones;
    }

    /**
     * 获取中心
     */
    public ArrayList<Area> getCenters() {
        return centers;
    }
}