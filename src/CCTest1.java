import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Random;

public class CCTest1 {
    public static void main(String[] args) throws Exception {
        // 实验配置
        double E = 50.0; // 期望值
        double[] RSDValues = {0.125, 0.25, 0.5}; // 相对标准差数组
        double[] rValues = {0.1, 0.2, 0.3}; // 容差参数取值
        double[] gammaValues = {0.7, 0.8, 0.9}; // 机会约束风险参数
        int[] scenarioNumValues = {500, 1000, 5000}; // 场景数量
        long seed = 12345678; // 随机种子

        // 输出结果的CSV文件
        String outputCSVPath = "./output/chance_constrained_results.csv";

        // 准备CSV文件
        try (BufferedWriter csvWriter = new BufferedWriter(new FileWriter(outputCSVPath))) {
            // 写入CSV标题行
            csvWriter.write("Instance,RSD,r,gamma,Scenarios,Runtime(s),Objective");
            csvWriter.newLine();

            // 获取instances目录下的所有.dat文件
            File dir = new File("./instances");
            File[] instanceFiles = dir.listFiles((d, name) -> name.endsWith(".dat"));

            // 如果没有找到实例文件，给出提示
            if (instanceFiles == null || instanceFiles.length == 0) {
                System.out.println("No .dat files found in ./instances directory.");
                return;
            }

            // 遍历所有实例文件
            for (File instanceFile : instanceFiles) {
                String instanceName = instanceFile.getName();

                // 遍历所有参数组合
                for (double RSD : RSDValues) {
                    for (double r : rValues) {
                        for (double gamma : gammaValues) {
                            for (int numScenarios : scenarioNumValues) {
                                // 打印当前实验信息
                                System.out.println("Running experiment:");
                                System.out.println("Instance: " + instanceName);
                                System.out.println("RSD: " + RSD);
                                System.out.println("r: " + r);
                                System.out.println("gamma: " + gamma);
                                System.out.println("Scenarios: " + numScenarios);

                                // 生成随机场景
                                Instance instance = new Instance(instanceFile.getPath());
                                double[][] scenarios = generateScenarios(
                                        instance.getN(), numScenarios, E, RSD, seed
                                );

                                // 记录开始时间
                                long startTime = System.currentTimeMillis();

                                // 创建算法实例 - 注意：需要修改ChanceConstrainedAlgo构造函数以支持r参数
                                ChanceConstrainedAlgo algo = new ChanceConstrainedAlgo(
                                        instance, scenarios, gamma, seed, r
                                );

                                // 运行算法
                                String outputFileName = String.format(
                                        "CC_%s_RSD%.3f_r%.1f_gamma%.1f_scen%d",
                                        instanceName.replace(".dat", ""),
                                        RSD, r, gamma, numScenarios
                                );

                                // 运行算法并获取目标函数值
                                double objectiveValue = 0;
                                try {
                                    objectiveValue = algo.run(outputFileName, false);
                                } catch (Exception e) {
                                    System.err.println("Error running experiment: " + e.getMessage());
                                    continue;
                                }

                                // 计算运行时间
                                long endTime = System.currentTimeMillis();
                                double runtime = (endTime - startTime) / 1000.0;

                                // 写入CSV结果
                                csvWriter.write(String.format(
                                        "%s,%.3f,%.1f,%.1f,%d,%.3f,%.4f",
                                        instanceName, RSD, r, gamma, numScenarios,
                                        runtime, objectiveValue
                                ));
                                csvWriter.newLine();

                                // 刷新以确保实时写入
                                csvWriter.flush();

                                // 可视化
                                String outputImagePath = String.format(
                                        "./output/%s_visualization.png", outputFileName
                                );
                                DistrictVisualizer visualizer = new DistrictVisualizer(
                                        instance, algo.getZones(), algo.getCenters()
                                );
                                visualizer.saveVisualization(outputImagePath);
                            }
                        }
                    }
                }
            }
        } catch (IOException e) {
            System.err.println("Error writing results to CSV: " + e.getMessage());
        }

        System.out.println("实验完成，结果已保存到 " + outputCSVPath);
    }

    // 生成随机场景 - 使用均匀分布
    private static double[][] generateScenarios(int n, int numScenarios, double E, double RSD, long seed) {
        double[][] scenarios = new double[numScenarios][n];
        Random rand = new Random(seed); // 固定种子以保证可重复性

        // 计算均匀分布的左右端点
        double lowerBound = E * (1 - Math.sqrt(3) * RSD);
        double upperBound = E * (1 + Math.sqrt(3) * RSD);

        // 生成场景
        for (int s = 0; s < numScenarios; s++) {
            for (int i = 0; i < n; i++) {
                // 均匀分布生成需求
                double demand = lowerBound + rand.nextDouble() * (upperBound - lowerBound);

                // 确保需求为正值
                scenarios[s][i] = Math.max(1, demand);
            }
        }

        return scenarios;
    }
}