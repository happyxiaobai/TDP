import java.util.Random;

public class ChanceConstrainedMain {
    public static void main(String[] args) throws Exception {
        // 指定输入文件
        String instanceFile = "./Instances/2DU100-05-1.dat";
        String outputFileName = "2DU100-05-1_cc";
        double gamma = 0.1; // 机会约束风险参数
        int numScenarios = 100; // 场景数量
        long seed = 12345678; // 添加一个固定种子

        // 加载实例
        Instance instance = new Instance(instanceFile);

        // 生成随机场景 - 这里已经有固定种子12345
        double[][] scenarios = generateScenarios(instance.getN(), numScenarios);


        // 创建并运行算法 - 传入固定种子
        ChanceConstrainedAlgo algo = new ChanceConstrainedAlgo(instance, scenarios, gamma, seed);
        algo.run(outputFileName);

        System.out.println("基于机会约束的配送区域划分问题求解完成。");
        String outputImagePath = "./output/" + outputFileName + "_visualization.png";
        DistrictVisualizer visualizer = new DistrictVisualizer(instance, algo.getZones(), algo.getCenters());
        visualizer.saveVisualization(outputImagePath);

        System.out.println("可视化图像已保存至: " + outputImagePath);
    }

    // 生成随机场景
    //第一个参数是基本单元个数，第二个参数是场景数量
    private static double[][] generateScenarios(int n, int numScenarios) {
        double[][] scenarios = new double[numScenarios][n];//每一行是一个场景
        Random rand = new Random(12345); // 固定种子以保证可重复性

        //这里的代码控制着随机生成需求场景
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
}
