import java.util.Random;

public class DistributionallyRobustMain {
    public static void main(String[] args) throws Exception {
        // 指定输入文件和输出文件名
        String instanceFile = "./Instances/DU200-05-8.dat";  // 使用与ChanceConstrainedMain类似的实例
        String outputFileName = "DU200-05-8_drcc";

        // 设置分布鲁棒优化参数
        double gamma = 0.05;  // 风险参数（违反概率）
        int numScenarios = 1000;  // 场景数量
        long seed = 12345678;  // 随机种子
        boolean useD1 = true;  // 是否使用D_1模糊集（false则使用D_2模糊集）
        double delta1 = 0.05;  // D_2模糊集参数
        double delta2 = 0.5;   // D_2模糊集参数（需保证delta2 > max{delta1, 1}）
        boolean useJointChance = false;  // 是否使用联合机会约束（Bonferroni近似）

        // 加载实例
        Instance instance = new Instance(instanceFile);

        // 生成随机场景
        double[][] scenarios = generateScenarios(instance.getN(), numScenarios, seed);

        // 创建并运行分布鲁棒算法
        DistributionallyRobustAlgo algo = new DistributionallyRobustAlgo(
                instance, scenarios, gamma, seed, useD1, delta1, delta2, useJointChance);

        // 运行算法并生成结果
        algo.run(outputFileName);

        System.out.println("基于分布鲁棒机会约束的配送区域划分问题求解完成。");

        // 可视化结果
        String outputImagePath = "./output/" + outputFileName + "_visualization.png";
        DistrictVisualizer visualizer = new DistrictVisualizer(instance, algo.getZones(), algo.getCenters());
        visualizer.saveVisualization(outputImagePath);

        System.out.println("可视化图像已保存至: " + outputImagePath);

        // 可选：添加与确定性模型和场景近似模型的比较分析
        runComparativeAnalysis(instance, scenarios, gamma, seed, outputFileName);
    }

    // 生成随机场景
    private static double[][] generateScenarios(int n, int numScenarios, long seed) {
        double[][] scenarios = new double[numScenarios][n];
        Random rand = new Random(seed);

        // 基于正态分布生成需求场景
        for (int s = 0; s < numScenarios; s++) {
            for (int i = 0; i < n; i++) {
                // 假设需求遵循均值为100，方差为25的正态分布
                double demand = rand.nextGaussian() * 25 + 100;
                // 确保需求为正值
                scenarios[s][i] = Math.max(1, demand);
            }
        }

        return scenarios;
    }

    // 比较不同模型的性能
    private static void runComparativeAnalysis(Instance instance, double[][] scenarios,
                                               double gamma, long seed, String baseFileName) throws Exception {
        System.out.println("\n开始进行比较分析...");

        // 1. 运行确定性模型
        Algo deterministicAlgo = new Algo(instance);
        deterministicAlgo.run(baseFileName + "_det");

        // 2. 运行场景近似模型
        ChanceConstrainedAlgo ccaAlgo = new ChanceConstrainedAlgo(instance, scenarios, gamma, seed);
        ccaAlgo.run(baseFileName + "_cc");

        // 3. 运行D_2模糊集分布鲁棒模型
        DistributionallyRobustAlgo drAlgo = new DistributionallyRobustAlgo(
                instance, scenarios, gamma, seed, false, 0.05, 0.5, false);
        drAlgo.run(baseFileName + "_dr_d2");

        // 输出各模型的目标函数值和求解时间等信息
        System.out.println("比较分析完成。详细结果请参见输出文件。");

        // 可以添加对样本外性能的评估代码
    }

    // 可以添加一个方法评估样本外性能
//    private static void evaluateOutOfSamplePerformance(Instance instance,
//                                                       ArrayList<Integer>[] zones,
//                                                       ArrayList<Area> centers,
//                                                       double gamma,
//                                                       int numTestScenarios) {
//        // 生成新的测试场景
//        // 评估各个模型解决方案在新场景下违反容量约束的概率
//        // 输出样本外性能评估结果
//    }
}