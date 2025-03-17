import gurobi.*;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.*;

public class ChanceConstrainedAlgo {
    private Instance inst;
    private ArrayList<Area> centers; // 存储所有大区域中心的坐标
    private ArrayList<Integer>[] zones;
    private double r; // 活动指标平衡容差
    private double gamma; // 机会约束风险参数
    private int[][] scenarioDemands; // 存储所有场景下的需求
    private int numScenarios; // 场景数量

    private Random rand; // 添加全局Random对象

    // 修改构造函数，接收一个种子参数
    public ChanceConstrainedAlgo(Instance instance, double[][] scenarios, double gamma, long seed) {
        this.inst = instance;
        this.zones = new ArrayList[inst.k];
        this.r = 0.05;
        this.gamma = gamma;
        this.rand = new Random(seed); // 使用固定种子初始化随机数生成器

        // 初始化场景需求
        this.numScenarios = scenarios.length;
        this.scenarioDemands = new int[numScenarios][inst.getN()];
        for (int s = 0; s < numScenarios; s++) {
            for (int i = 0; i < inst.getN(); i++) {
                this.scenarioDemands[s][i] = (int) scenarios[s][i];
            }
        }
    }

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
        boolean feasible = false;
        if (useScenarioGenerationMethod()) {
            feasible = generateInitialSolutionWithScenarioGeneration();
        } else {
            feasible = generateInitialSolutionWithExactMethod();
        }

        if (!feasible) {
            System.out.println("无法找到可行解，请检查模型参数或增加场景数量");
            return;
        }

        // Step 3: 改善初始解
        boolean change = true;
        double cur_value = evaluateObjective();

        while (change) {
            change = false;

            // 检查每个区域的真正中心
            ArrayList<Area> newCenters = findTrueCenters();

            // 如果区域中心发生变化，更新并重新求解
            if (!compareCenters(centers, newCenters)) {
                centers = newCenters;
                change = true;

                if (useScenarioGenerationMethod()) {
                    feasible = generateInitialSolutionWithScenarioGeneration();
                } else {
                    feasible = generateInitialSolutionWithExactMethod();
                }

                if (feasible) {
                    cur_value = evaluateObjective();
                }
            }
        }

        // 检查并确保连通性
        ensureConnectivity();

        // 评估最终结果
        cur_value = evaluateObjective();
        if (cur_value < Best) {
            Best = cur_value;
            for (int z = 0; z < inst.k; z++) {
                BestZones[z] = new ArrayList<>();
                BestZones[z].addAll(zones[z]);
            }
        }

        long endTime = System.currentTimeMillis();
        double timeSpentInSeconds = (endTime - startTime) / 1000.0;

        // 输出结果
        String outputFilePath = "./output/" + filename.replace(".dat", "_cc.txt");
        FileWriter writer = new FileWriter(outputFilePath);
        BufferedWriter buffer = new BufferedWriter(writer);

        for (int io = 0; io < BestZones.length; io++) {
            buffer.write("center ID: " + centers.get(io).getId() + "\n");
            for (int jo = 0; jo < BestZones[io].size(); jo++) {
                buffer.write(BestZones[io].get(jo) + " ");
            }
            buffer.newLine();
        }

        String result = String.format("%.2f", Best);
        buffer.write("best objective: " + result + "\n");
        buffer.write("程序运行时间为：" + timeSpentInSeconds + "s" + "\n");
        buffer.write("机会约束风险参数：" + gamma + "\n");
        buffer.close();
    }

    // 选择初始区域中心
    private ArrayList<Integer> selectInitialCenters() throws GRBException {
        int InitialNum = 5; // 可调整的初始场景数
        ArrayList<Integer> candidateCenters = new ArrayList<>();
        HashMap<Integer, Integer> centerFrequency = new HashMap<>();

        // 这里不再创建新的Random对象，而是使用类的全局rand
        int scenariosProcessed = 0;

        while (scenariosProcessed < InitialNum) {
            int scenarioIndex = rand.nextInt(numScenarios); // 使用类的全局rand

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
        sortedCenters.sort((a, b) -> b.getValue().compareTo(a.getValue()));

        for (int i = 0; i < Math.min(inst.k, sortedCenters.size()); i++) {
            candidateCenters.add(sortedCenters.get(i).getKey());
        }

        // 如果中心数量不足，随机补充
        while (candidateCenters.size() < inst.k) {
            int randomCenter = rand.nextInt(inst.getN()); // 使用类的全局rand
            if (!candidateCenters.contains(randomCenter)) {
                candidateCenters.add(randomCenter);
            }
        }

        return candidateCenters;
    }

    // 求解单一场景的确定性模型 - 修改后的方法
    private ArrayList<Integer> solveForScenario(int scenarioIndex) throws GRBException {
        // 设置求解时间限制
        int localTimeLimit = 60; // 秒

        try {
            // 创建基于特定场景需求的实例
            Instance scenarioInstance = createScenarioInstance(scenarioIndex);

            // 创建Algo对象并设置时间限制
            Algo algo = new Algo(scenarioInstance);
            algo.setTimeLimit(localTimeLimit);

            // 获取该场景下的求解结果中心点
            ArrayList<Integer> scenarioCenters = algo.getSolutionCenters();

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

    // 创建基于特定场景需求的Instance实例
    private Instance createScenarioInstance(int scenarioIndex) {
        return new Instance(inst, scenarioDemands[scenarioIndex]);
    }

    // 方法选择标志
    private boolean useScenarioGenerationMethod() {
        // 根据问题规模决定使用哪种方法
        return inst.getN() > 100; // 当基本单元数量大于100时使用场景生成法
    }

    // 使用精确方法生成初始可行解
    private boolean generateInitialSolutionWithExactMethod() throws GRBException {
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

        // 场景违反标志 z_omega
        GRBVar[] z = new GRBVar[numScenarios];
        for (int s = 0; s < numScenarios; s++) {
            z[s] = model.addVar(0, 1, 0, GRB.BINARY, "z_" + s);
        }

        // 约束2: 每个基本单元必须且只能属于一个区域
        for (int i = 0; i < inst.getN(); i++) {
            GRBLinExpr expr = new GRBLinExpr();
            for (int j = 0; j < centers.size(); j++) {
                expr.addTerm(1.0, x[i][j]);
            }
            model.addConstr(expr, GRB.EQUAL, 1.0, "assign_" + i);
        }

        // 约束22: 容量上限约束
        double U = (1 + r) * inst.average1; // 区域最大容量上限
        double M = 10 * U; // 足够大的数

        for (int j = 0; j < centers.size(); j++) {
            for (int s = 0; s < numScenarios; s++) {
                GRBLinExpr expr = new GRBLinExpr();
                for (int i = 0; i < inst.getN(); i++) {
                    expr.addTerm(scenarioDemands[s][i], x[i][j]);
                }
                expr.addTerm(M, z[s]);
                model.addConstr(expr, GRB.LESS_EQUAL, U + M, "capacity_" + j + "_" + s);
            }
        }

        // 约束27: 场景违反限制
        GRBLinExpr violationExpr = new GRBLinExpr();
        for (int s = 0; s < numScenarios; s++) {
            violationExpr.addTerm(1.0, z[s]);
        }
        int maxViolations = (int) Math.floor(gamma * numScenarios);
        model.addConstr(violationExpr, GRB.LESS_EQUAL, maxViolations, "violations");

        // 目标函数: 最小化总距离
        GRBLinExpr objExpr = new GRBLinExpr();
        for (int i = 0; i < inst.getN(); i++) {
            for (int j = 0; j < centers.size(); j++) {
                objExpr.addTerm(inst.dist[i][centers.get(j).getId()], x[i][j]);
            }
        }
        model.setObjective(objExpr, GRB.MINIMIZE);

        // 求解模型
        model.optimize();

        // 检查是否找到可行解
        if (model.get(GRB.IntAttr.Status) == GRB.Status.OPTIMAL ||
                model.get(GRB.IntAttr.Status) == GRB.Status.SUBOPTIMAL) {

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
        }

        model.dispose();
        env.dispose();
        return false;
    }

    // 使用场景生成方法生成初始可行解
    private boolean generateInitialSolutionWithScenarioGeneration() throws GRBException {
        // 创建初始场景子集
        // 不再创建新的Random对象，使用类的全局rand
        HashSet<Integer> selectedScenarios = new HashSet<>();
        selectedScenarios.add(rand.nextInt(numScenarios)); // 开始时选择一个随机场景

        GRBEnv env = new GRBEnv();

        env.set(GRB.IntParam.LogToConsole, 0);
        // 设置Gurobi的随机种子参数，确保求解过程确定性
        env.set(GRB.IntParam.Seed, 42); // 固定种子

        boolean feasible = false;
        int iterationLimit = 100; // 最大迭代次数
        int iteration = 0;

        while (!feasible && iteration < iterationLimit) {
            iteration++;

            // 求解基于当前选定场景的子模型
            GRBModel subModel = createSubModel(env, selectedScenarios);
            subModel.optimize();

            if (subModel.get(GRB.IntAttr.Status) == GRB.Status.OPTIMAL ||
                    subModel.get(GRB.IntAttr.Status) == GRB.Status.SUBOPTIMAL) {

                // 提取当前解
                extractSolution(subModel);

                // 检查所有场景的可行性
                HashSet<Integer> feasibleScenarios = checkFeasibility();

                // 如果有足够比例的场景是可行的，则解是可接受的
                int requiredFeasibleScenarios = (int) Math.ceil((1 - gamma) * numScenarios);
                if (feasibleScenarios.size() >= requiredFeasibleScenarios) {
                    feasible = true;
                } else {
                    // 添加一些当前不可行的场景到子模型中
                    List<Integer> infeasibleScenarios = new ArrayList<>();
                    for (int s = 0; s < numScenarios; s++) {
                        if (!feasibleScenarios.contains(s)) {
                            infeasibleScenarios.add(s);
                        }
                    }

                    // 随机选择一些不可行场景加入
                    int numToAdd = Math.min(2, infeasibleScenarios.size()); // 每次添加2个或更少
                    for (int i = 0; i < numToAdd && !infeasibleScenarios.isEmpty(); i++) {
                        int idx = rand.nextInt(infeasibleScenarios.size()); // 使用类的全局rand
                        selectedScenarios.add(infeasibleScenarios.get(idx));
                        infeasibleScenarios.remove(idx);
                    }
                }
            } else {
                // 子模型不可行，尝试不同的场景组合
                selectedScenarios.clear();
                selectedScenarios.add(rand.nextInt(numScenarios)); // 使用类的全局rand
            }

            subModel.dispose();
        }

        env.dispose();
        return feasible;
    }

    // 创建基于选定场景的子模型
    private GRBModel createSubModel(GRBEnv env, HashSet<Integer> selectedScenarios) throws GRBException {
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

        // 约束2: 每个基本单元必须且只能属于一个区域
        for (int i = 0; i < inst.getN(); i++) {
            GRBLinExpr expr = new GRBLinExpr();
            for (int j = 0; j < centers.size(); j++) {
                expr.addTerm(1.0, x[i][j]);
            }
            model.addConstr(expr, GRB.EQUAL, 1.0, "assign_" + i);
        }

        // 约束22: 容量上限约束 (只对选定的场景)
        double U = (1 + r) * inst.average1; // 区域最大容量上限

        for (int j = 0; j < centers.size(); j++) {
            for (int s : selectedScenarios) {
                GRBLinExpr expr = new GRBLinExpr();
                for (int i = 0; i < inst.getN(); i++) {
                    expr.addTerm(scenarioDemands[s][i], x[i][j]);
                }
                model.addConstr(expr, GRB.LESS_EQUAL, U, "capacity_" + j + "_" + s);
            }
        }

        // 目标函数: 最小化总距离
        GRBLinExpr objExpr = new GRBLinExpr();
        for (int i = 0; i < inst.getN(); i++) {
            for (int j = 0; j < centers.size(); j++) {
                objExpr.addTerm(inst.dist[i][centers.get(j).getId()], x[i][j]);
            }
        }
        model.setObjective(objExpr, GRB.MINIMIZE);

        return model;
    }

    // 从模型中提取解决方案
    private void extractSolution(GRBModel model) throws GRBException {
        // 初始化区域
        for (int j = 0; j < centers.size(); j++) {
            zones[j] = new ArrayList<>();
        }

        // 获取决策变量
        for (int i = 0; i < inst.getN(); i++) {
            for (int j = 0; j < centers.size(); j++) {
                GRBVar var = model.getVarByName("x_" + i + "_" + centers.get(j).getId());
                if (var != null && Math.abs(var.get(GRB.DoubleAttr.X) - 1.0) < 1e-6) {
                    zones[j].add(i);
                    break;
                }
            }
        }
    }

    // 检查所有场景的可行性
    private HashSet<Integer> checkFeasibility() {
        HashSet<Integer> feasibleScenarios = new HashSet<>();
        double U = (1 + r) * inst.average1; // 区域最大容量上限

        for (int s = 0; s < numScenarios; s++) {
            boolean scenarioFeasible = true;

            for (int j = 0; j < centers.size(); j++) {
                double totalDemand = 0;
                for (int i : zones[j]) {
                    totalDemand += scenarioDemands[s][i];
                }

                if (totalDemand > U) {
                    scenarioFeasible = false;
                    break;
                }
            }

            if (scenarioFeasible) {
                feasibleScenarios.add(s);
            }
        }

        return feasibleScenarios;
    }

    // 查找每个区域的真正中心
    private ArrayList<Area> findTrueCenters() {
        ArrayList<Area> newCenters = new ArrayList<>();

        for (int j = 0; j < centers.size(); j++) {
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

    // 比较两组中心是否相同
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

    // 确保每个区域的连通性
    private void ensureConnectivity() throws GRBException {
        boolean allConnected = false;

        while (!allConnected) {
            allConnected = true;

            for (int j = 0; j < centers.size(); j++) {
                ArrayList<ArrayList<Integer>> components = findConnectedComponents(zones[j]);

                if (components.size() > 1) {
                    allConnected = false;

                    // 添加连通性约束并重新求解
                    GRBEnv env = new GRBEnv();

                    env.set(GRB.IntParam.LogToConsole, 0);
                    env.set(GRB.IntParam.Seed, 42);

                    GRBModel model = createSubModelWithConnectivity(env, j, components);
                    model.optimize();

                    if (model.get(GRB.IntAttr.Status) == GRB.Status.OPTIMAL ||
                            model.get(GRB.IntAttr.Status) == GRB.Status.SUBOPTIMAL) {

                        // 更新当前区域的解决方案
                        zones[j].clear();
                        for (int i = 0; i < inst.getN(); i++) {
                            GRBVar var = model.getVarByName("x_" + i + "_" + j);
                            if (var != null && Math.abs(var.get(GRB.DoubleAttr.X) - 1.0) < 1e-6) {
                                zones[j].add(i);
                            }
                        }
                    }

                    model.dispose();
                    env.dispose();
                }
            }
        }
    }

    // 找出一个区域内的所有连通组件
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

    // 创建带有连通性约束的子模型
    private GRBModel createSubModelWithConnectivity(GRBEnv env, int centerIndex, ArrayList<ArrayList<Integer>> components) throws GRBException {
        GRBModel model = new GRBModel(env);

        // 决策变量 x_ij (只考虑当前区域)
        GRBVar[] x = new GRBVar[inst.getN()];
        for (int i = 0; i < inst.getN(); i++) {
            x[i] = model.addVar(0, 1, 0, GRB.BINARY, "x_" + i + "_" + centerIndex);
        }

        // 设置当前区域中心为1
        x[centers.get(centerIndex).getId()].set(GRB.DoubleAttr.LB, 1);
        x[centers.get(centerIndex).getId()].set(GRB.DoubleAttr.UB, 1);

        // 找出中心所在的连通组件
        int centerComponent = -1;
        for (int c = 0; c < components.size(); c++) {
            if (components.get(c).contains(centers.get(centerIndex).getId())) {
                centerComponent = c;
                break;
            }
        }

        // 为每个不包含中心的连通组件添加连通性约束
        for (int c = 0; c < components.size(); c++) {
            if (c != centerComponent) {
                ArrayList<Integer> component = components.get(c);

                // 找出该组件的邻居
                HashSet<Integer> neighbors = new HashSet<>();
                for (int node : component) {
                    for (int neighbor : inst.getAreas()[node].getNeighbors()) {
                        if (!component.contains(neighbor)) {
                            neighbors.add(neighbor);
                        }
                    }
                }

                // 添加约束: 至少有一个邻居被分配到该区域
                GRBLinExpr expr = new GRBLinExpr();
                for (int neighbor : neighbors) {
                    expr.addTerm(1.0, x[neighbor]);
                }

                // 对当前组件中的所有节点 
                for (int node : component) {
                    expr.addTerm(-1.0, x[node]);
                }

                model.addConstr(expr, GRB.GREATER_EQUAL, 1 - component.size(), "connectivity_" + centerIndex + "_" + c);
            }
        }

        // 约束: 保持其他区域不变
        for (int j = 0; j < centers.size(); j++) {
            if (j != centerIndex) {
                for (int i : zones[j]) {
                    x[i].set(GRB.DoubleAttr.UB, 0); // 禁止分配给当前区域
                }
            }
        }

        // 约束: 容量限制 (使用平均需求)
        double U = (1 + r) * inst.average1;
        GRBLinExpr capacityExpr = new GRBLinExpr();
        for (int i = 0; i < inst.getN(); i++) {
            // 这里使用平均需求，也可以考虑使用最差情况需求
            double avgDemand = 0;
            for (int s = 0; s < numScenarios; s++) {
                avgDemand += scenarioDemands[s][i];
            }
            avgDemand /= numScenarios;

            capacityExpr.addTerm(avgDemand, x[i]);
        }
        model.addConstr(capacityExpr, GRB.LESS_EQUAL, U, "capacity_" + centerIndex);

        // 目标函数: 最小化总距离
        GRBLinExpr objExpr = new GRBLinExpr();
        for (int i = 0; i < inst.getN(); i++) {
            objExpr.addTerm(inst.dist[i][centers.get(centerIndex).getId()], x[i]);
        }
        model.setObjective(objExpr, GRB.MINIMIZE);

        return model;
    }

    // 计算当前解的目标函数值
    private double evaluateObjective() {
        double totalDist = 0;

        for (int j = 0; j < centers.size(); j++) {
            for (int i : zones[j]) {
                totalDist += inst.dist[i][centers.get(j).getId()];
            }
        }

        return totalDist;
    }

    public ArrayList<Integer>[] getZones() {
        return zones;
    }

    public ArrayList<Area> getCenters() {
        return centers;
    }
}