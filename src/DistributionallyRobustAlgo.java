import gurobi.*;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.*;

/**
 * 分布鲁棒机会约束分区问题的解决方案
 * 实现基于D_1和D_2模糊集的DRICC问题以及基于Bonferroni近似的DRJCC模型
 */
public class DistributionallyRobustAlgo {
    private Instance inst;
    private ArrayList<Area> centers; // 存储所有区域中心
    private ArrayList<Integer>[] zones; // 存储每个区域的基本单元编号
    private double r; // 活动指标平衡容差
    private double gamma; // 机会约束风险参数
    private double[][] scenarios; // 原始场景数据
    private int numScenarios; // 场景数量
    private Random rand; // 随机数生成器

    // 分布鲁棒模型的参数
    private double[] meanVector; // 样本均值向量
    private double[][] covarianceMatrix; // 样本协方差矩阵
    private double delta1; // D_2模糊集参数
    private double delta2; // D_2模糊集参数
    private boolean useD1; // 是否使用D_1模糊集
    private boolean useJointChance; // 是否使用联合机会约束
    private double[] individualGammas; // Bonferroni近似的个体风险分配

    // 时间限制和迭代限制
    private int timeLimit = 1000; // 秒
    private int maxIterations = 100;

    /**
     * 构造函数
     * @param instance 问题实例
     * @param scenarios 场景数据
     * @param gamma 风险参数
     * @param seed 随机种子
     * @param useD1 是否使用D_1模糊集（false则使用D_2模糊集）
     * @param delta1 D_2模糊集参数
     * @param delta2 D_2模糊集参数
     * @param useJointChance 是否使用联合机会约束（Bonferroni近似）
     */
    public DistributionallyRobustAlgo(Instance instance, double[][] scenarios, double gamma,
                                      long seed, boolean useD1, double delta1, double delta2, boolean useJointChance) {
        this.inst = instance;
        this.scenarios = scenarios;
        this.numScenarios = scenarios.length;
        this.gamma = gamma;
        this.r = 0.05; // 活动指标平衡容差
        this.rand = new Random(seed);
        this.zones = new ArrayList[inst.k];
        this.useD1 = useD1;
        this.delta1 = delta1;
        this.delta2 = delta2;
        this.useJointChance = useJointChance;

        // 计算样本均值和协方差矩阵
        calculateMomentInformation();

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
     * @param filename 输出文件名
     */
    public void run(String filename) throws GRBException, IOException {
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
    }

    /**
     * 选择初始区域中心
     */
    private ArrayList<Integer> selectInitialCenters() throws GRBException {
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

    /**
     * 求解单一场景的确定性模型
     */
    private ArrayList<Integer> solveForScenario(int scenarioIndex) throws GRBException {
        ArrayList<Integer> centers = new ArrayList<>();
        Set<Integer> centerSet = new HashSet<>();

        try {
            GRBEnv env = new GRBEnv();
            env.set(GRB.IntParam.LogToConsole, 0);
            env.set(GRB.IntParam.Seed, 42);
            GRBModel model = new GRBModel(env);

            // 创建决策变量
            GRBVar[][] x = new GRBVar[inst.getN()][inst.getN()];
            for (int i = 0; i < inst.getN(); i++) {
                for (int j = 0; j < inst.getN(); j++) {
                    x[i][j] = model.addVar(0, 1, 0, GRB.BINARY, "x_" + i + "_" + j);
                }
            }

            // 每个点都必须属于一个区域
            for (int i = 0; i < inst.getN(); i++) {
                GRBLinExpr expr = new GRBLinExpr();
                for (int j = 0; j < inst.getN(); j++) {
                    expr.addTerm(1.0, x[i][j]);
                }
                model.addConstr(expr, GRB.EQUAL, 1.0, "assign_" + i);
            }

            // 创建p个区域中心
            GRBLinExpr centerExpr = new GRBLinExpr();
            for (int j = 0; j < inst.getN(); j++) {
                centerExpr.addTerm(1.0, x[j][j]);
            }
            model.addConstr(centerExpr, GRB.EQUAL, inst.k, "p_centers");

            // 平衡性约束
            double U = (1 + r) * inst.average1;
            for (int j = 0; j < inst.getN(); j++) {
                GRBLinExpr demandExpr = new GRBLinExpr();
                for (int i = 0; i < inst.getN(); i++) {
                    demandExpr.addTerm(scenarios[scenarioIndex][i], x[i][j]);
                }
                model.addConstr(demandExpr, GRB.LESS_EQUAL, U, "capacity_" + j);
            }

            // 目标函数: 最小化总距离
            GRBLinExpr objExpr = new GRBLinExpr();
            for (int i = 0; i < inst.getN(); i++) {
                for (int j = 0; j < inst.getN(); j++) {
                    objExpr.addTerm(inst.dist[i][j], x[i][j]);
                }
            }
            model.setObjective(objExpr, GRB.MINIMIZE);

            // 设置求解时间限制
            model.set(GRB.DoubleParam.TimeLimit, 30);

            // 求解
            model.optimize();

            // 检查解的状态
            if (model.get(GRB.IntAttr.Status) == GRB.Status.OPTIMAL ||
                    model.get(GRB.IntAttr.Status) == GRB.Status.SUBOPTIMAL) {

                // 提取区域中心
                for (int j = 0; j < inst.getN(); j++) {
                    if (Math.abs(x[j][j].get(GRB.DoubleAttr.X) - 1.0) < 1e-6) {
                        centerSet.add(j);
                    }
                }
            }

            model.dispose();
            env.dispose();

        } catch (GRBException e) {
            System.out.println("求解场景时发生错误: " + e.getMessage());
        }

        centers.addAll(centerSet);

        // 如果中心不足，随机补充
        while (centers.size() < inst.k) {
            int candidate = rand.nextInt(inst.getN());
            if (!centerSet.contains(candidate)) {
                centers.add(candidate);
                centerSet.add(candidate);
            }
        }

        return centers;
    }

    /**
     * 生成初始可行解
     */
    private boolean generateInitialSolution() throws GRBException {
        GRBEnv env = new GRBEnv();
        env.set(GRB.IntParam.LogToConsole, 0);
        env.set(GRB.IntParam.Seed, 42);
        GRBModel model = new GRBModel(env);

        // 决策变量 x_ij
        GRBVar[][] x = new GRBVar[inst.getN()][centers.size()];
        for (int i = 0; i < inst.getN(); i++) {
            for (int j = 0; j < centers.size(); j++) {
                x[i][j] = model.addVar(0, 1, 0, GRB.BINARY, "x_" + i + "_" + centers.get(j).getId());
                if (i == centers.get(j).getId()) {
                    x[i][j].set(GRB.DoubleAttr.LB, 1);
                    x[i][j].set(GRB.DoubleAttr.UB, 1);
                }
            }
        }

        // 约束: 每个基本单元必须且只能属于一个区域
        for (int i = 0; i < inst.getN(); i++) {
            GRBLinExpr expr = new GRBLinExpr();
            for (int j = 0; j < centers.size(); j++) {
                expr.addTerm(1.0, x[i][j]);
            }
            model.addConstr(expr, GRB.EQUAL, 1.0, "assign_" + i);
        }

        // 添加分布鲁棒机会约束
        addDistributionallyRobustConstraints(model, x);

        // 目标函数: 最小化总距离
        GRBLinExpr objExpr = new GRBLinExpr();
        for (int i = 0; i < inst.getN(); i++) {
            for (int j = 0; j < centers.size(); j++) {
                objExpr.addTerm(inst.dist[i][centers.get(j).getId()], x[i][j]);
            }
        }
        model.setObjective(objExpr, GRB.MINIMIZE);

        // 设置求解时间限制
        model.set(GRB.DoubleParam.TimeLimit, timeLimit);

        try {
            // 求解
            model.optimize();

            // 检查模型状态
            if (model.get(GRB.IntAttr.Status) != GRB.Status.OPTIMAL &&
                    model.get(GRB.IntAttr.Status) != GRB.Status.SUBOPTIMAL) {
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

            model.dispose();
            env.dispose();
            return true;

        } catch (GRBException e) {
            System.out.println("求解模型时发生错误: " + e.getMessage());
            model.dispose();
            env.dispose();
            return false;
        }
    }

    /**
     * 添加分布鲁棒机会约束
     */
    private void addDistributionallyRobustConstraints(GRBModel model, GRBVar[][] x) throws GRBException {
        double U = (1 + r) * inst.average1; // 区域容量上限

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
     * 添加DRICC约束
     */
    private void addDRICCConstraint(GRBModel model, GRBVar[][] x, int j, double riskParam) throws GRBException {
        double U = (1 + r) * inst.average1; // 区域容量上限

        // 构建x向量的线性表达式
        GRBLinExpr meanTerm = new GRBLinExpr();
        for (int i = 0; i < inst.getN(); i++) {
            meanTerm.addTerm(meanVector[i], x[i][j]);
        }

        if (useD1) {
            // 使用D_1模糊集的约束
            // μ^T*x_j + sqrt((1-γ)/γ)*sqrt(x_j^T*Σ*x_j) ≤ U
            double factor = Math.sqrt((1 - riskParam) / riskParam);

            // 创建二次约束的表达式
            GRBQuadExpr quadTerm = new GRBQuadExpr();
            for (int i = 0; i < inst.getN(); i++) {
                for (int k = 0; k < inst.getN(); k++) {
                    quadTerm.addTerm(covarianceMatrix[i][k], x[i][j], x[k][j]);
                }
            }

            // 添加SOCP约束 (通过引入新变量t)
            GRBVar t = model.addVar(0, GRB.INFINITY, 0, GRB.CONTINUOUS, "t_" + j);
            model.addQConstr(quadTerm, GRB.LESS_EQUAL, t * t, "socp1_" + j);

            // 均值项 + factor * t ≤ U
            GRBLinExpr constr = new GRBLinExpr();
            constr.add(meanTerm);
            constr.addTerm(factor, t);
            model.addConstr(constr, GRB.LESS_EQUAL, U, "dr_capacity_" + j);

        } else {
            // 使用D_2模糊集的约束
            double factor;

            if (delta1 / delta2 <= riskParam) {
                // 第一种情况的约束
                // μ^T*x_j + (sqrt(δ_1) + sqrt((1-γ)/γ)*(δ_2-δ_1))*sqrt(x_j^T*Σ*x_j) ≤ U
                factor = Math.sqrt(delta1) + Math.sqrt((1 - riskParam) / riskParam * (delta2 - delta1));
            } else {
                // 第二种情况的约束
                // μ^T*x_j + sqrt(δ_2/γ)*sqrt(x_j^T*Σ*x_j) ≤ U
                factor = Math.sqrt(delta2 / riskParam);
            }

            // 创建二次约束的表达式
            GRBQuadExpr quadTerm = new GRBQuadExpr();
            for (int i = 0; i < inst.getN(); i++) {
                for (int k = 0; k < inst.getN(); k++) {
                    quadTerm.addTerm(covarianceMatrix[i][k], x[i][j], x[k][j]);
                }
            }

            // 添加SOCP约束 (通过引入新变量t)
            GRBVar t = model.addVar(0, GRB.INFINITY, 0, GRB.CONTINUOUS, "t_" + j);
            model.addQConstr(quadTerm, GRB.LESS_EQUAL, t * t, "socp2_" + j);

            // 均值项 + factor * t ≤ U
            GRBLinExpr constr = new GRBLinExpr();
            constr.add(meanTerm);
            constr.addTerm(factor, t);
            model.addConstr(constr, GRB.LESS_EQUAL, U, "dr_capacity_" + j);
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
     * 确保每个区域的连通性
     */
    private void ensureConnectivity() throws GRBException {
        boolean allConnected = false;
        int iteration = 0;
        int maxIterations = 1000; // 限制迭代次数

        // 创建环境和模型
        GRBEnv env = new GRBEnv();
        env.set(GRB.IntParam.LogToConsole, 0);
        env.set(GRB.IntParam.Seed, 42);
        GRBModel model = new GRBModel(env);

        // 决策变量 x_ij
        GRBVar[][] x = new GRBVar[inst.getN()][centers.size()];
        for (int i = 0; i < inst.getN(); i++) {
            for (int j = 0; j < centers.size(); j++) {
                x[i][j] = model.addVar(0, 1, 0, GRB.BINARY, "x_" + i + "_" + centers.get(j).getId());
                if (i == centers.get(j).getId()) {
                    x[i][j].set(GRB.DoubleAttr.LB, 1);
                    x[i][j].set(GRB.DoubleAttr.UB, 1);
                }
            }
        }

        // 约束: 每个基本单元必须且只能属于一个区域
        for (int i = 0; i < inst.getN(); i++) {
            GRBLinExpr expr = new GRBLinExpr();
            for (int j = 0; j < centers.size(); j++) {
                expr.addTerm(1.0, x[i][j]);
            }
            model.addConstr(expr, GRB.EQUAL, 1.0, "assign_" + i);
        }

        // 添加分布鲁棒机会约束
        addDistributionallyRobustConstraints(model, x);

        // 目标函数: 最小化总距离
        GRBLinExpr objExpr = new GRBLinExpr();
        for (int i = 0; i < inst.getN(); i++) {
            for (int j = 0; j < centers.size(); j++) {
                objExpr.addTerm(inst.dist[i][centers.get(j).getId()], x[i][j]);
            }
        }
        model.setObjective(objExpr, GRB.MINIMIZE);

        // 迭代添加连通性约束
        while (!allConnected && iteration < maxIterations) {
            iteration++;

            // 检查所有区域是否连通
            boolean hasDisconnection = false;
            Map<Integer, List<ArrayList<Integer>>> allDisconnectedComponents = new HashMap<>();

            // 先获取当前解
            if (iteration > 1) {
                // 提取当前解决方案
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
                if (zones[j] == null || zones[j].isEmpty()) {
                    continue;
                }

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

                    model.addConstr(constrExpr, GRB.GREATER_EQUAL, 1 - component.size(), "connectivity_" + iteration + "_" + constraintCounter);
                    constraintCounter++;
                }
            }

            // 求解更新后的模型
            model.optimize();

            // 如果找不到可行解，退出
            if (model.get(GRB.IntAttr.Status) != GRB.Status.OPTIMAL &&
                    model.get(GRB.IntAttr.Status) != GRB.Status.SUBOPTIMAL) {
                System.out.println("连通性处理迭代 " + iteration + " 失败，模型无解");
                break;
            }

            System.out.println("连通性处理迭代 " + iteration + " 完成，添加了 " + constraintCounter + " 个连通性约束");
        }

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

        // 销毁模型和环境
        model.dispose();
        env.dispose();
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
