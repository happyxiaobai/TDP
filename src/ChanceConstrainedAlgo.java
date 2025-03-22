import gurobi.*;

import java.io.IOException;
import java.util.*;

public class ChanceConstrainedAlgo {
    private Instance inst;
    private ArrayList<Area> centers; // 存储所有区域中心的坐标
    private ArrayList<Integer>[] zones; //存储每个区域的基本单元编号
    private double r; // 活动指标平衡容差
    private double gamma; // 机会约束风险参数
    private int[][] scenarioDemands; // 存储所有场景下的需求
    private int numScenarios; // 场景数量

    private Random rand; // 添加全局Random对象

    private HashSet<Integer> selectedScenarios; // 存储选定的场景集合

    private double baseDemandUpperBound; // 基础上限(基于原始实例)
    private double[] scenarioDemandUpperBounds; // 每个场景的上限

    // 修改构造函数，接收一个种子参数
    public ChanceConstrainedAlgo(Instance instance, double[][] scenarios, double gamma, long seed,double r) {
        this.inst = instance;
        this.zones = new ArrayList[inst.k];
        this.r = r;
        this.gamma = gamma;
        this.rand = new Random(seed); // 使用固定种子初始化随机数生成器
        this.selectedScenarios = new HashSet<>(); // 初始化选定场景集合

        // 初始化场景需求，把传入的需求场景复制到本地
        this.numScenarios = scenarios.length;
        this.scenarioDemands = new int[numScenarios][inst.getN()];
        this.scenarioDemandUpperBounds = new double[numScenarios];

        // 计算基础需求上限
        double totalDemand = 0;
        for (int i = 0; i < inst.getN(); i++) {
            totalDemand += inst.getAreas()[i].getActiveness()[0];
        }
        this.baseDemandUpperBound = (1 + r) * (totalDemand / inst.k);

        for (int s = 0; s < numScenarios; s++) {
            double scenarioTotalDemand = 0;
            for (int i = 0; i < inst.getN(); i++) {
                this.scenarioDemands[s][i] = (int) scenarios[s][i];
                scenarioTotalDemand += scenarios[s][i];
            }
            this.scenarioDemandUpperBounds[s] = (1 + r) * (scenarioTotalDemand / inst.k);
        }
    }

    public double run(String filename, boolean useScenarioGeneration) throws GRBException, IOException {
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
        if (useScenarioGeneration) {
            // 使用场景生成法
            feasible = generateInitialSolutionWithScenarioGeneration();

            if (feasible) {
                // Step 3: 改善初始解 - 场景生成法需要迭代调整中心
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
                        feasible = generateInitialSolutionWithScenarioGeneration();
                        if (feasible) {
                            cur_value = evaluateObjective();
                        }
                    }
                }

                // 检查并确保连通性
                ensureConnectivity();
            }
        } else {
            // 使用精确方法 - 连通性将在内部处理
            feasible = generateInitialSolutionWithExactMethod();
            // 精确方法已经在内部处理了连通性和中心调整
        }

        if (!feasible) {
            System.out.println("无法找到可行解，请检查模型参数或增加场景数量");
            return -1;
        }

        // 评估最终结果
        double cur_value = evaluateObjective();
        if (cur_value < Best) {
            Best = cur_value;
            for (int z = 0; z < inst.k; z++) {
                BestZones[z] = new ArrayList<>();
                BestZones[z].addAll(zones[z]);
            }
        }

        long endTime = System.currentTimeMillis();
        double timeSpentInSeconds = (endTime - startTime) / 1000.0;

//        // 输出结果
//        String outputFilePath = "./output/" + filename.replace(".dat", "_cc.txt");
//        FileWriter writer = new FileWriter(outputFilePath);
//        BufferedWriter buffer = new BufferedWriter(writer);
//
//        for (int io = 0; io < BestZones.length; io++) {
//            buffer.write("center ID: " + centers.get(io).getId() + "\n");
//            for (int jo = 0; jo < BestZones[io].size(); jo++) {
//                buffer.write(BestZones[io].get(jo) + " ");
//            }
//            buffer.newLine();
//        }
//
//        String result = String.format("%.2f", Best);
//        buffer.write("best objective: " + result + "\n");
//        buffer.write("程序运行时间为：" + timeSpentInSeconds + "s" + "\n");
//        buffer.write("机会约束风险参数：" + gamma + "\n");
//        buffer.close();
//        System.out.println("程序运行时间为：" + timeSpentInSeconds + "s" + "\n");
        return Best;
    }

    // 保持原函数的向后兼容性
    public void run(String filename) throws GRBException, IOException {
        // 默认情况下，根据问题规模选择方法
        run(filename, inst.getN() > 100);
    }

    // 选择初始区域中心
    private ArrayList<Integer> selectInitialCenters() throws GRBException {
        int InitialNum = 5; // 可调整的初始场景数
        ArrayList<Integer> candidateCenters = new ArrayList<>();
        HashMap<Integer, Integer> centerFrequency = new HashMap<>();

        // 这里不再创建新的Random对象，而是使用类的全局rand
        int scenariosProcessed = 0;


        while (scenariosProcessed < InitialNum) {
            int scenarioIndex = rand.nextInt(numScenarios); // 选择一个随机场景

            // 使用该场景的需求求解确定性模型
            ArrayList<Integer> scenarioCenters = solveForScenario(scenarioIndex);

            if (scenarioCenters.size() == inst.k) {
                scenariosProcessed++;
//                System.out.println("处理场景 " + scenariosProcessed + "/" + InitialNum + "，场景索引: " + scenarioIndex);

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
    // Modified method with global time limit of 1200 seconds
    private boolean generateInitialSolutionWithExactMethod() throws GRBException {
        // Record the start time
        long startTime = System.currentTimeMillis();
        final long TIME_LIMIT_MS = 1200 * 1000; // 1200 seconds in milliseconds

        boolean feasible = false;
        boolean centersChanged = true;

        while (centersChanged) {
            // Check if the global time limit has been exceeded
            if (System.currentTimeMillis() - startTime > TIME_LIMIT_MS) {
                System.out.println("Global time limit of 1200 seconds exceeded in exact method");
                return false;
            }

            centersChanged = false;

            GRBEnv env = new GRBEnv(true);
            env.set(GRB.IntParam.OutputFlag, 0);
            env.set(GRB.IntParam.LogToConsole, 0);
            env.set(GRB.StringParam.LogFile, "");
            env.set(GRB.IntParam.Seed, 42);
            env.start();

            GRBModel model = new GRBModel(env);

            // Calculate remaining time for solver
            long remainingTimeMs = TIME_LIMIT_MS - (System.currentTimeMillis() - startTime);
            double remainingTimeSec = Math.max(1.0, remainingTimeMs / 1000.0);
            model.set(GRB.DoubleParam.TimeLimit, remainingTimeSec);

            // Decision variables x_ij
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

            // Scenario violation flags z_omega
            GRBVar[] z = new GRBVar[numScenarios];
            for (int s = 0; s < numScenarios; s++) {
                z[s] = model.addVar(0, 1, 0, GRB.BINARY, "z_" + s);
            }

            // Constraint: each basic unit must belong to exactly one district
            for (int i = 0; i < inst.getN(); i++) {
                GRBLinExpr expr = new GRBLinExpr();
                for (int j = 0; j < centers.size(); j++) {
                    expr.addTerm(1.0, x[i][j]);
                }
                model.addConstr(expr, GRB.EQUAL, 1.0, "assign_" + i);
            }

            // Capacity constraints for all scenarios
            double U = 0; // Region maximum capacity
            double M = 0; // Big M value

            for (int j = 0; j < centers.size(); j++) {
                for (int s = 0; s < numScenarios; s++) {
                    GRBLinExpr expr = new GRBLinExpr();
                    for (int i = 0; i < inst.getN(); i++) {
                        expr.addTerm(scenarioDemands[s][i], x[i][j]);
                    }
                    U = scenarioDemandUpperBounds[s];
                    M = centers.size() * U;
                    expr.addTerm(-M, z[s]); // Add with negative coefficient
                    model.addConstr(expr, GRB.LESS_EQUAL, U, "capacity_" + j + "_" + s);
                }
            }

            // Scenario violation limit constraint
            GRBLinExpr violationExpr = new GRBLinExpr();
            for (int s = 0; s < numScenarios; s++) {
                violationExpr.addTerm(1.0, z[s]);
            }
            int maxViolations = (int) Math.floor(gamma * numScenarios);
            model.addConstr(violationExpr, GRB.LESS_EQUAL, maxViolations, "violations");

            // Objective function: minimize total distance
            GRBLinExpr objExpr = new GRBLinExpr();
            for (int i = 0; i < inst.getN(); i++) {
                for (int j = 0; j < centers.size(); j++) {
                    objExpr.addTerm(inst.dist[i][centers.get(j).getId()], x[i][j]);
                }
            }
            model.setObjective(objExpr, GRB.MINIMIZE);

            // Connectivity constraint iterative process
            boolean connectivityViolation = true;
            int connectivityIterations = 0;
            int maxConnectivityIterations = 1000;
            int totalConstraints = 0;

            while (connectivityViolation && connectivityIterations < maxConnectivityIterations) {
                // Check if global time limit has been exceeded
                if (System.currentTimeMillis() - startTime > TIME_LIMIT_MS) {
                    System.out.println("Global time limit exceeded during connectivity iterations");
                    model.dispose();
                    env.dispose();
                    return false;
                }

                connectivityIterations++;

                // Solve current model
                model.optimize();

                // Check model status
                if (model.get(GRB.IntAttr.Status) == GRB.Status.TIME_LIMIT) {
                    System.out.println("Optimization timed out on connectivity iteration " + connectivityIterations);
                    model.dispose();
                    env.dispose();
                    return false;
                }

                if (model.get(GRB.IntAttr.Status) != GRB.Status.OPTIMAL &&
                        model.get(GRB.IntAttr.Status) != GRB.Status.SUBOPTIMAL) {
                    // Model is infeasible
                    model.dispose();
                    env.dispose();
                    return false;
                }

                // Extract current solution
                for (int j = 0; j < centers.size(); j++) {
                    zones[j] = new ArrayList<>();
                    for (int i = 0; i < inst.getN(); i++) {
                        if (Math.abs(x[i][j].get(GRB.DoubleAttr.X) - 1.0) < 1e-6) {
                            zones[j].add(i);
                        }
                    }
                }

                // Check connectivity
                connectivityViolation = false;
                int constraintCounter = 0;

                for (int j = 0; j < centers.size(); j++) {
                    ArrayList<ArrayList<Integer>> components = findConnectedComponents(zones[j]);

                    if (components.size() > 1) {
                        connectivityViolation = true;

                        // Find the component containing the center
                        int centerComponentIndex = -1;
                        for (int c = 0; c < components.size(); c++) {
                            if (components.get(c).contains(centers.get(j).getId())) {
                                centerComponentIndex = c;
                                break;
                            }
                        }

                        // Add connectivity constraints for components not containing the center
                        for (int c = 0; c < components.size(); c++) {
                            if (c != centerComponentIndex) {
                                ArrayList<Integer> component = components.get(c);

                                // Find component neighbors
                                HashSet<Integer> neighbors = new HashSet<>();
                                for (int node : component) {
                                    for (int neighbor : inst.getAreas()[node].getNeighbors()) {
                                        if (!component.contains(neighbor)) {
                                            neighbors.add(neighbor);
                                        }
                                    }
                                }

                                // Add connectivity constraint
                                GRBLinExpr expr = new GRBLinExpr();

                                // For all neighbor nodes
                                for (int neighbor : neighbors) {
                                    expr.addTerm(1.0, x[neighbor][j]);
                                }

                                // For all nodes in this component
                                for (int node : component) {
                                    expr.addTerm(-1.0, x[node][j]);
                                }

                                model.addConstr(expr, GRB.GREATER_EQUAL, 1 - component.size(), "connectivity_" + totalConstraints);
                                constraintCounter++;
                                totalConstraints++;
                            }
                        }
                    }
                }

                // If no connectivity violations, exit the loop
                if (!connectivityViolation) {
                    break;
                }

                // Update the solver time limit for the next iteration
                remainingTimeMs = TIME_LIMIT_MS - (System.currentTimeMillis() - startTime);
                remainingTimeSec = Math.max(1.0, remainingTimeMs / 1000.0);
                model.set(GRB.DoubleParam.TimeLimit, remainingTimeSec);

                System.out.println("Added " + constraintCounter + " connectivity constraints, continuing iteration...");
            }

            if (connectivityViolation) {
                System.out.println("Warning: Couldn't ensure connectivity within max iterations");
                model.dispose();
                env.dispose();
                return false;
            }

            // Connectivity constraints satisfied, result is valid
            feasible = true;

            // Update district centers
            ArrayList<Area> newCenters = findTrueCenters();

            // If centers changed, resolve
            if (!compareCenters(centers, newCenters)) {
                centers = newCenters;
                centersChanged = true;
                System.out.println("District centers changed, resolving...");
            }

            // Only keep final solution when centers no longer change
            if (!centersChanged) {
                // Ensure zones contains the final solution
                for (int j = 0; j < centers.size(); j++) {
                    zones[j] = new ArrayList<>();
                    for (int i = 0; i < inst.getN(); i++) {
                        if (Math.abs(x[i][j].get(GRB.DoubleAttr.X) - 1.0) < 1e-6) {
                            zones[j].add(i);
                        }
                    }
                }
            }

            model.dispose();
            env.dispose();
        }

        return feasible;
    }
//    private boolean generateInitialSolutionWithExactMethod() throws GRBException {
//        boolean feasible = false;
//        boolean centersChanged = true;
//
//        while (centersChanged) {
//            centersChanged = false;
//
//            GRBEnv env = new GRBEnv(true);  // Create the env with manual start mode
//
//// Set logging parameters BEFORE starting the environment
//            env.set(GRB.IntParam.OutputFlag, 0);        // Suppress all output
//            env.set(GRB.IntParam.LogToConsole, 0);      // Disable console logging
//            env.set(GRB.StringParam.LogFile, "");       // Empty log file path
//            env.set(GRB.IntParam.Seed, 42);
//// Now start the environment
//            env.start();
//
//            GRBModel model = new GRBModel(env);
//
//            // 决策变量 x_ij
//            GRBVar[][] x = new GRBVar[inst.getN()][centers.size()];
//            for (int i = 0; i < inst.getN(); i++) {
//                for (int j = 0; j < centers.size(); j++) {
//                    x[i][j] = model.addVar(0, 1, 0, GRB.BINARY, "x_" + i + "_" + centers.get(j).getId());
//                    if (i == centers.get(j).getId()) {
//                        x[i][j].set(GRB.DoubleAttr.LB, 1);
//                        x[i][j].set(GRB.DoubleAttr.UB, 1);
//                    }
//                }
//            }
//
//            // 场景违反标志 z_omega
//            GRBVar[] z = new GRBVar[numScenarios];
//            for (int s = 0; s < numScenarios; s++) {
//                z[s] = model.addVar(0, 1, 0, GRB.BINARY, "z_" + s);
//            }
//
//            // 约束: 每个基本单元必须且只能属于一个区域
//            for (int i = 0; i < inst.getN(); i++) {
//                GRBLinExpr expr = new GRBLinExpr();
//                for (int j = 0; j < centers.size(); j++) {
//                    expr.addTerm(1.0, x[i][j]);
//                }
//                model.addConstr(expr, GRB.EQUAL, 1.0, "assign_" + i);
//            }
//
//            // 容量上限约束 - 对所有场景
//            double U = 0; // 区域最大容量上限
//            double M = 0; // 足够大的数
//
//            for (int j = 0; j < centers.size(); j++) {
//                for (int s = 0; s < numScenarios; s++) {
//                    GRBLinExpr expr = new GRBLinExpr();
//                    for (int i = 0; i < inst.getN(); i++) {
//                        expr.addTerm(scenarioDemands[s][i], x[i][j]);
//                    }
//                    U = scenarioDemandUpperBounds[s];
//                    M = centers.size() * U;
//                    expr.addTerm(-M, z[s]); // Add with negative coefficient
//                    model.addConstr(expr, GRB.LESS_EQUAL, U, "capacity_" + j + "_" + s);
//                }
//            }
//
//            // 场景违反限制
//            GRBLinExpr violationExpr = new GRBLinExpr();
//            for (int s = 0; s < numScenarios; s++) {
//                violationExpr.addTerm(1.0, z[s]);
//            }
//            int maxViolations = (int) Math.floor(gamma * numScenarios);
//            model.addConstr(violationExpr, GRB.LESS_EQUAL, maxViolations, "violations");
//
//            // 设置目标函数: 最小化总距离
//            GRBLinExpr objExpr = new GRBLinExpr();
//            for (int i = 0; i < inst.getN(); i++) {
//                for (int j = 0; j < centers.size(); j++) {
//                    objExpr.addTerm(inst.dist[i][centers.get(j).getId()], x[i][j]);
//                }
//            }
//            model.setObjective(objExpr, GRB.MINIMIZE);
//
//            // 添加连通性约束的迭代过程
//            boolean connectivityViolation = true;
//            int connectivityIterations = 0;
//            int maxConnectivityIterations = 1000;
//            int totalConstraints = 0;
//
//            while (connectivityViolation && connectivityIterations < maxConnectivityIterations) {
//                connectivityIterations++;
//
//                // 求解当前模型
//                model.optimize();
//
//                // 检查模型状态
//                if (model.get(GRB.IntAttr.Status) != GRB.Status.OPTIMAL &&
//                        model.get(GRB.IntAttr.Status) != GRB.Status.SUBOPTIMAL) {
//                    // 模型不可行
//                    model.dispose();
//                    env.dispose();
//                    return false;
//                }
//
//                // 提取当前解
//                for (int j = 0; j < centers.size(); j++) {
//                    zones[j] = new ArrayList<>();
//                    for (int i = 0; i < inst.getN(); i++) {
//                        if (Math.abs(x[i][j].get(GRB.DoubleAttr.X) - 1.0) < 1e-6) {
//                            zones[j].add(i);
//                        }
//                    }
//                }
//
//                // 检查连通性
//                connectivityViolation = false;
//                int constraintCounter = 0;
//
//                for (int j = 0; j < centers.size(); j++) {
//                    ArrayList<ArrayList<Integer>> components = findConnectedComponents(zones[j]);
//
//                    if (components.size() > 1) {
//                        connectivityViolation = true;
//
//                        // 找出中心所在的连通组件
//                        int centerComponentIndex = -1;
//                        for (int c = 0; c < components.size(); c++) {
//                            if (components.get(c).contains(centers.get(j).getId())) {
//                                centerComponentIndex = c;
//                                break;
//                            }
//                        }
//
//                        // 为每个不包含中心的连通组件添加连通性约束
//                        for (int c = 0; c < components.size(); c++) {
//                            if (c != centerComponentIndex) {
//                                ArrayList<Integer> component = components.get(c);
//
//                                // 找出该组件的邻居
//                                HashSet<Integer> neighbors = new HashSet<>();
//                                for (int node : component) {
//                                    for (int neighbor : inst.getAreas()[node].getNeighbors()) {
//                                        if (!component.contains(neighbor)) {
//                                            neighbors.add(neighbor);
//                                        }
//                                    }
//                                }
//
//                                // 添加连通性约束
//                                GRBLinExpr expr = new GRBLinExpr();
//
//                                // 对所有的邻居节点
//                                for (int neighbor : neighbors) {
//                                    expr.addTerm(1.0, x[neighbor][j]);
//                                }
//
//                                // 对当前组件中的所有节点
//                                for (int node : component) {
//                                    expr.addTerm(-1.0, x[node][j]);
//                                }
//
//                                model.addConstr(expr, GRB.GREATER_EQUAL, 1 - component.size(), "connectivity_" + totalConstraints);
//                                constraintCounter++;
//                                totalConstraints++;
//                            }
//                        }
//                    }
//                }
//
//                // 如果没有连通性违反，退出循环
//                if (!connectivityViolation) {
//                    break;
//                }
//
//                System.out.println("添加了 " + constraintCounter + " 个连通性约束，继续迭代...");
//            }
//
//            if (connectivityViolation) {
//                System.out.println("警告：在最大迭代次数内未能保证所有区域的连通性");
//                model.dispose();
//                env.dispose();
//                return false;
//            }
//
//            // 连通性约束满足，结果有效
//            feasible = true;
//
//            // 更新区域中心
//            ArrayList<Area> newCenters = findTrueCenters();
//
//            // 如果中心发生变化，需要重新求解
//            if (!compareCenters(centers, newCenters)) {
//                centers = newCenters;
//                centersChanged = true;
////                System.out.println("区域中心发生变化，重新求解...");
//            }
//
//            // 只有当中心不再变化时才保留最终解
//            if (!centersChanged) {
//                // 确保zones中包含最终解
//                for (int j = 0; j < centers.size(); j++) {
//                    zones[j] = new ArrayList<>();
//                    for (int i = 0; i < inst.getN(); i++) {
//                        if (Math.abs(x[i][j].get(GRB.DoubleAttr.X) - 1.0) < 1e-6) {
//                            zones[j].add(i);
//                        }
//                    }
//                }
//            }
//
//            model.dispose();
//            env.dispose();
//        }
//
//        return feasible;
//    }

    // 使用场景生成方法生成初始可行解
    //TODO 这个函数也需要修改，生成可行解之后，需要保留最进一步的进入求解过程的场景，用在最后的改善阶段
    // Modified method with global time limit of 600 seconds
    private boolean generateInitialSolutionWithScenarioGeneration() throws GRBException {
        // Record the start time
        long startTime = System.currentTimeMillis();
        final long TIME_LIMIT_MS = 600 * 1000; // 600 seconds in milliseconds

        // Create initial scenario subset
        selectedScenarios.clear(); // Clear previous scenarios
        selectedScenarios.add(rand.nextInt(numScenarios)); // Start with one random scenario

        // Create tabu list to store problematic scenarios
        HashSet<Integer> tabuScenarios = new HashSet<>();

        // Create the environment
        GRBEnv env = new GRBEnv(true);  // Create the env with manual start mode

        // Set logging parameters BEFORE starting the environment
        env.set(GRB.IntParam.OutputFlag, 0);        // Suppress all output
        env.set(GRB.IntParam.LogToConsole, 0);      // Disable console logging
        env.set(GRB.StringParam.LogFile, "");       // Empty log file path
        env.set(GRB.IntParam.Seed, 42);             // Fixed seed
        env.start();

        boolean feasible = false;
        int iterationLimit = 100; // Maximum iterations
        int iteration = 0;
        int previousFeasibleCount = 0;
        int lastAddedScenario = -1;

        while (!feasible) {
            // Check if the global time limit has been exceeded
            long currentTime = System.currentTimeMillis();
            if (currentTime - startTime > TIME_LIMIT_MS) {
                System.out.println("Global time limit of 600 seconds exceeded in scenario generation method");
                env.dispose();
                return false;
            }


            // Solve based on current selected scenarios
            GRBModel subModel = createSubModel(env, selectedScenarios);

            // Set time limit to the solver - calculate remaining time
            long remainingTimeMs = TIME_LIMIT_MS - (System.currentTimeMillis() - startTime);
            double remainingTimeSec = Math.max(1.0, remainingTimeMs / 1000.0); // Ensure at least 1 second
            subModel.set(GRB.DoubleParam.TimeLimit, remainingTimeSec);

            subModel.optimize();

            // Check if optimization timed out
            if (subModel.get(GRB.IntAttr.Status) == GRB.Status.TIME_LIMIT) {
                System.out.println("Optimization timed out");

                // If we just added a scenario, add it to the tabu list
                if (lastAddedScenario != -1) {
                    tabuScenarios.add(lastAddedScenario);
                    selectedScenarios.remove(lastAddedScenario);
//                    System.out.println("Added scenario " + lastAddedScenario + " to tabu list due to timeout");
                    lastAddedScenario = -1;
                }

                // If this happens on the first iteration with just one scenario, we need to try a different one
                if (selectedScenarios.size() <= 1) {
                    selectedScenarios.clear();
                    int newScenario;
                    do {
                        newScenario = rand.nextInt(numScenarios);
                    } while (tabuScenarios.contains(newScenario));

                    selectedScenarios.add(newScenario);
//                    System.out.println("Restarting with new initial scenario: " + newScenario);
                }

                subModel.dispose();
                continue;
            }

            if (subModel.get(GRB.IntAttr.Status) == GRB.Status.OPTIMAL ||
                    subModel.get(GRB.IntAttr.Status) == GRB.Status.SUBOPTIMAL) {

                // Extract current solution
                extractSolution(subModel);

                // Check feasibility for all scenarios
                HashSet<Integer> feasibleScenarios = checkFeasibility();

                // Calculate required number of feasible scenarios
                int requiredFeasibleScenarios = (int) Math.ceil((1 - gamma) * numScenarios);
                int currentFeasibleCount = feasibleScenarios.size();

                // Check if we need to add the last scenario to tabu list (starting from the second iteration)
                if (iteration > 1 && lastAddedScenario != -1) {
                    if (currentFeasibleCount < previousFeasibleCount) {
                        // The last added scenario made things worse, add it to tabu list
                        tabuScenarios.add(lastAddedScenario);
                        selectedScenarios.remove(lastAddedScenario);
//                        System.out.println("Added scenario " + lastAddedScenario + " to tabu list");

                        // Re-solve with the updated scenario set (without the tabu scenario)
                        subModel.dispose();

                        // Check time limit again before re-solving
                        if (System.currentTimeMillis() - startTime > TIME_LIMIT_MS) {
                            System.out.println("Global time limit exceeded before re-solving");
                            env.dispose();
                            return false;
                        }

                        subModel = createSubModel(env, selectedScenarios);

                        // Recalculate remaining time for solver
                        remainingTimeMs = TIME_LIMIT_MS - (System.currentTimeMillis() - startTime);
                        remainingTimeSec = Math.max(1.0, remainingTimeMs / 1000.0);
                        subModel.set(GRB.DoubleParam.TimeLimit, remainingTimeSec);

                        subModel.optimize();

                        if (subModel.get(GRB.IntAttr.Status) == GRB.Status.TIME_LIMIT) {
                            System.out.println("Optimization timed out after removing problematic scenario");
                            subModel.dispose();
                            continue;
                        }

                        if (subModel.get(GRB.IntAttr.Status) == GRB.Status.OPTIMAL ||
                                subModel.get(GRB.IntAttr.Status) == GRB.Status.SUBOPTIMAL) {
                            extractSolution(subModel);
                            feasibleScenarios = checkFeasibility();
                            currentFeasibleCount = feasibleScenarios.size();
                        }
                    }
                }

                // Update previousFeasibleCount for next iteration
                previousFeasibleCount = currentFeasibleCount;

                if (currentFeasibleCount >= requiredFeasibleScenarios) {
                    // Solution is acceptable
                    feasible = true;
                } else {
                    // Need to add more scenarios

                    // Get all infeasible scenarios
                    List<Integer> infeasibleScenarios = new ArrayList<>();
                    for (int s = 0; s < numScenarios; s++) {
                        if (!feasibleScenarios.contains(s) && !selectedScenarios.contains(s) && !tabuScenarios.contains(s)) {
                            infeasibleScenarios.add(s);
                        }
                    }

                    // Check if we have any non-tabu scenarios to add
                    if (infeasibleScenarios.isEmpty()) {
                        System.out.println("No valid scenarios left to add - all scenarios are either selected or tabu");
                        subModel.dispose();
                        feasible = false;
                        break;
                    }

                    // Randomly select ONE scenario to add
                    int idx = rand.nextInt(infeasibleScenarios.size());
                    lastAddedScenario = infeasibleScenarios.get(idx);
                    selectedScenarios.add(lastAddedScenario);
//                    System.out.println("Added scenario " + lastAddedScenario);
                }
            } else {
                // Sub-model is infeasible, try a different combination
                System.out.println("Sub-model infeasible, resetting selection");
                selectedScenarios.clear();

                // Choose a random scenario that's not in the tabu list
                int newScenario;
                do {
                    newScenario = rand.nextInt(numScenarios);
                } while (tabuScenarios.contains(newScenario));

                selectedScenarios.add(newScenario);
                lastAddedScenario = -1;
                previousFeasibleCount = 0;
            }

            subModel.dispose();
        }

        env.dispose();
        return feasible;
    }


// Modified method with tabu list for problematic scenarios
    // Modified method with tabu list for problematic scenarios and timeout handling
//    private boolean generateInitialSolutionWithScenarioGeneration() throws GRBException {
//        // Create initial scenario subset
//        selectedScenarios.clear(); // Clear previous scenarios
//        selectedScenarios.add(rand.nextInt(numScenarios)); // Start with one random scenario
//
//        // Create tabu list to store problematic scenarios
//        HashSet<Integer> tabuScenarios = new HashSet<>();
//
//        // Create the environment
//        GRBEnv env = new GRBEnv(true);  // Create the env with manual start mode
//
//        // Set logging parameters BEFORE starting the environment
//        env.set(GRB.IntParam.OutputFlag, 0);        // Suppress all output
//        env.set(GRB.IntParam.LogToConsole, 0);      // Disable console logging
//        env.set(GRB.StringParam.LogFile, "");       // Empty log file path
//        env.set(GRB.IntParam.Seed, 42);             // Fixed seed
//        env.start();
//
//        boolean feasible = false;
//        int iterationLimit = numScenarios/2; // Maximum iterations
//        int iteration = 0;
//        int previousFeasibleCount = 0;
//        int lastAddedScenario = -1;
//
//        while (!feasible && iteration < iterationLimit) {
//            iteration++;
//            System.out.println("Iteration " + iteration + ": Selected scenarios: " + selectedScenarios.size());
//
//            // Solve based on current selected scenarios
//            GRBModel subModel = createSubModel(env, selectedScenarios);
//
//            // Set time limit to 600 seconds
//            subModel.set(GRB.DoubleParam.TimeLimit, 300.0);
//
//            subModel.optimize();
//
//            // Check if optimization timed out
//            if (subModel.get(GRB.IntAttr.Status) == GRB.Status.TIME_LIMIT) {
//                System.out.println("Optimization timed out after 300 seconds");
//
//                // If we just added a scenario, add it to the tabu list
//                if (lastAddedScenario != -1) {
//                    tabuScenarios.add(lastAddedScenario);
//                    selectedScenarios.remove(lastAddedScenario);
//                    System.out.println("Added scenario " + lastAddedScenario + " to tabu list due to timeout");
//                    lastAddedScenario = -1;
//                }
//
//                // If this happens on the first iteration with just one scenario, we need to try a different one
//                if (selectedScenarios.size() <= 1) {
//                    selectedScenarios.clear();
//                    int newScenario;
//                    do {
//                        newScenario = rand.nextInt(numScenarios);
//                    } while (tabuScenarios.contains(newScenario));
//
//                    selectedScenarios.add(newScenario);
//                    System.out.println("Restarting with new initial scenario: " + newScenario);
//                }
//
//                subModel.dispose();
//                continue;
//            }
//
//            if (subModel.get(GRB.IntAttr.Status) == GRB.Status.OPTIMAL ||
//                    subModel.get(GRB.IntAttr.Status) == GRB.Status.SUBOPTIMAL) {
//
//                // Extract current solution
//                extractSolution(subModel);
//
//                // Check feasibility for all scenarios
//                HashSet<Integer> feasibleScenarios = checkFeasibility();
//
//                // Calculate required number of feasible scenarios
//                int requiredFeasibleScenarios = (int) Math.ceil((1 - gamma) * numScenarios);
//                int currentFeasibleCount = feasibleScenarios.size();
//                System.out.println("Current feasible scenarios: " + currentFeasibleCount);
//                System.out.println("Required feasible scenarios: " + requiredFeasibleScenarios);
//
//                // Check if we need to add the last scenario to tabu list (starting from the second iteration)
//                if (iteration > 1 && lastAddedScenario != -1) {
//                    if (currentFeasibleCount < previousFeasibleCount) {
//                        // The last added scenario made things worse, add it to tabu list
//                        tabuScenarios.add(lastAddedScenario);
//                        selectedScenarios.remove(lastAddedScenario);
//                        System.out.println("Added scenario " + lastAddedScenario + " to tabu list");
//
//                        // Re-solve with the updated scenario set (without the tabu scenario)
//                        subModel.dispose();
//                        subModel = createSubModel(env, selectedScenarios);
//                        subModel.set(GRB.DoubleParam.TimeLimit, 300.0);
//                        subModel.optimize();
//
//                        if (subModel.get(GRB.IntAttr.Status) == GRB.Status.TIME_LIMIT) {
//                            System.out.println("Optimization timed out after removing problematic scenario");
//                            subModel.dispose();
//                            continue;
//                        }
//
//                        if (subModel.get(GRB.IntAttr.Status) == GRB.Status.OPTIMAL ||
//                                subModel.get(GRB.IntAttr.Status) == GRB.Status.SUBOPTIMAL) {
//                            extractSolution(subModel);
//                            feasibleScenarios = checkFeasibility();
//                            currentFeasibleCount = feasibleScenarios.size();
//                        }
//                    }
//                }
//
//                // Update previousFeasibleCount for next iteration
//                previousFeasibleCount = currentFeasibleCount;
//
//                if (currentFeasibleCount >= requiredFeasibleScenarios) {
//                    // Solution is acceptable
//                    feasible = true;
//                } else {
//                    // Need to add more scenarios
//
//                    // Get all infeasible scenarios
//                    List<Integer> infeasibleScenarios = new ArrayList<>();
//                    for (int s = 0; s < numScenarios; s++) {
//                        if (!feasibleScenarios.contains(s) && !selectedScenarios.contains(s) && !tabuScenarios.contains(s)) {
//                            infeasibleScenarios.add(s);
//                        }
//                    }
//
//                    // Check if we have any non-tabu scenarios to add
//                    if (infeasibleScenarios.isEmpty()) {
//                        System.out.println("No valid scenarios left to add - all scenarios are either selected or tabu");
//                        subModel.dispose();
//                        feasible = false;
//                        break;
//                    }
//
//                    // Randomly select ONE scenario to add
//                    int numToAdd = 1; // Add exactly 1 scenario at a time
//
//                    int idx = rand.nextInt(infeasibleScenarios.size());
//                    lastAddedScenario = infeasibleScenarios.get(idx);
//                    selectedScenarios.add(lastAddedScenario);
//                    System.out.println("Added scenario " + lastAddedScenario);
//                }
//            } else {
//                // Sub-model is infeasible, try a different combination
//                System.out.println("Sub-model infeasible, resetting selection");
//                selectedScenarios.clear();
//
//                // Choose a random scenario that's not in the tabu list
//                int newScenario;
//                do {
//                    newScenario = rand.nextInt(numScenarios);
//                } while (tabuScenarios.contains(newScenario));
//
//                selectedScenarios.add(newScenario);
//                lastAddedScenario = -1;
//                previousFeasibleCount = 0;
//            }
//
//            subModel.dispose();
//        }
//
//        env.dispose();
//        return feasible;
//    }

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
            //实际上多一个场景就是多一些需求平衡约束，其他约束没什么变化
            for (int s : selectedScenarios) {
                GRBLinExpr expr = new GRBLinExpr();
                for (int i = 0; i < inst.getN(); i++) {
                    expr.addTerm(scenarioDemands[s][i], x[i][j]);
                }
                U = scenarioDemandUpperBounds[s];
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
            U = scenarioDemandUpperBounds[s];
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
        int iteration = 0;
        int maxIterations = 1000; // 限制迭代次数
// Create the environment
        GRBEnv env = new GRBEnv(true);  // Create the env with manual start mode

// Set logging parameters BEFORE starting the environment
        env.set(GRB.IntParam.OutputFlag, 0);        // Suppress all output
        env.set(GRB.IntParam.LogToConsole, 0);      // Disable console logging
        env.set(GRB.StringParam.LogFile, "");       // Empty log file path

// Now start the environment

        env.set(GRB.IntParam.Seed, 42);
        env.start();

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

        // 添加需求约束: 使用之前选定的场景
        double U = (1 + r) * inst.average1; // 区域最大容量上限

        for (int j = 0; j < centers.size(); j++) {
            for (int s : selectedScenarios) {
                GRBLinExpr expr = new GRBLinExpr();
                for (int i = 0; i < inst.getN(); i++) {
                    expr.addTerm(scenarioDemands[s][i], x[i][j]);
                }
                U = scenarioDemandUpperBounds[s];
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

        // 记录已添加的约束总数
        int totalConstraints = 0;

        while (!allConnected && iteration < maxIterations) {
            iteration++;

            // 检查所有区域是否连通
            boolean hasDisconnection = false;
            Map<Integer, List<ArrayList<Integer>>> allDisconnectedComponents = new HashMap<>();

            // 先获取当前解
            if (iteration > 1) {
                // 提取解决方案
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
                    GRBLinExpr expr = new GRBLinExpr();

                    // 对所有的邻居节点
                    for (int neighbor : neighbors) {
                        expr.addTerm(1.0, x[neighbor][districtIndex]);
                    }

                    // 对当前组件中的所有节点
                    for (int node : component) {
                        expr.addTerm(-1.0, x[node][districtIndex]);
                    }

                    model.addConstr(expr, GRB.GREATER_EQUAL, 1 - component.size(), "connectivity_" + totalConstraints);
                    constraintCounter++;
                    totalConstraints++;
                }
            }

            // 求解模型
            model.optimize();

            // 如果找不到可行解，退出
            if (model.get(GRB.IntAttr.Status) != GRB.Status.OPTIMAL &&
                    model.get(GRB.IntAttr.Status) != GRB.Status.SUBOPTIMAL) {
                System.out.println("连通性处理迭代 " + iteration + " 失败，模型无解");
                break;
            }

//            System.out.println("连通性处理迭代 " + iteration + " 完成，添加了 " + constraintCounter + " 个连通性约束");
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

        // 最后才销毁模型和环境
        model.dispose();
        env.dispose();
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