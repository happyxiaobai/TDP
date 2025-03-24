import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Random;

public class CCTest1 {
    public static void main(String[] args) throws Exception {
        // Experiment configuration
        double E = 50.0; // Expected value
        double[] RSDValues = {0.125, 0.25, 0.5}; // Relative standard deviation array
        double[] rValues = {0.1, 0.2, 0.3}; // Tolerance parameter values
        double[] gammaValues = {0.3, 0.2, 0.1}; // Chance constraint risk parameter
        int[] scenarioNumValues = {500, 1000, 5000}; // Number of scenarios
        boolean[] useScenarioGeneration = {true}; // Whether to use scenario generation
        long seed = 12345678; // Random seed

        long testSeed = seed + 1000;
        int numTestScenarios = 1000;

        // Output CSV file
        String outputCSVPath = "./output/chance_constrained_results.csv";

        // Prepare CSV file
        // Modified code for CCTest1 class - CSV output section

// Prepare CSV file
        try (BufferedWriter csvWriter = new BufferedWriter(new FileWriter(outputCSVPath))) {
            // Write CSV header with modified columns - replacing Instance with NumUnits and NumRegions
            csvWriter.write("InstanceName,NumUnits,NumRegions,RSD,r,gamma,Scenarios,UseScenarioGeneration,Runtime(s),Objective,OutOfSamplePerformance");
            csvWriter.newLine();

            // Get all .dat files in instances directory
            File dir = new File("./Instances");
            File[] instanceFiles = dir.listFiles((d, name) -> name.endsWith(".dat"));

            // If no instance files are found, provide a hint
            if (instanceFiles == null || instanceFiles.length == 0) {
                System.out.println("No .dat files found in ./instances directory.");
                return;
            }
            int cnt = 0;
            // Iterate through all instance files
            for (File instanceFile : instanceFiles) {
                String instanceName = instanceFile.getName();

                // Iterate through all parameter combinations
                for (double RSD : RSDValues) {
                    for (double r : rValues) {
                        for (double gamma : gammaValues) {
                            for (int numScenarios : scenarioNumValues) {
                                for (boolean useScenario : useScenarioGeneration) {
                                    // Print current experiment information
                                    System.out.println("current inst number:" + cnt + "    >>>>>>>>>>>Running experiment:");
                                    cnt++;
                                    System.out.println("Instance: " + instanceName);
                                    System.out.println("RSD: " + RSD);
                                    System.out.println("r: " + r);
                                    System.out.println("gamma: " + gamma);
                                    System.out.println("Scenarios: " + numScenarios);
                                    System.out.println("Use Scenario Generation: " + useScenario);

                                    // Generate random scenarios
                                    Instance instance = new Instance(instanceFile.getPath());

                                    // Get the number of basic units and regions from the instance
                                    int numUnits = instance.getN();
                                    int numRegions = instance.k;

                                    double[][] scenarios = generateScenarios(
                                            numUnits, numScenarios, E, RSD, seed
                                    );

                                    // Record start time
                                    long startTime = System.currentTimeMillis();

                                    // Create algorithm instance
                                    ChanceConstrainedAlgo algo = new ChanceConstrainedAlgo(
                                            instance, scenarios, gamma, seed, r
                                    );

                                    // Run algorithm and get objective value
                                    double objectiveValue = 0;
                                    try {
                                        // Pass the useScenario parameter to control scenario generation
                                        objectiveValue = algo.run("", useScenario);
                                    } catch (Exception e) {
                                        System.err.println("Error running experiment: " + e.getMessage());
                                        continue;
                                    }
                                    if (objectiveValue == -1) {
                                        System.out.println("Error: Objective value is -1, skipping this instance:" + instanceName);
                                    }
                                    // Calculate runtime
                                    long endTime = System.currentTimeMillis();
                                    double runtime = (endTime - startTime) / 1000.0;

                                    // Test out-of-sample performance
                                    double outOfSamplePerformance = testOutOfSamplePerformance(
                                            instance, algo, E, RSD, testSeed, r, numTestScenarios);

                                    // Write to CSV file with the modified fields (numUnits and numRegions instead of instanceName)
                                    csvWriter.write(String.format(
                                            "%s,%d,%d,%.3f,%.1f,%.1f,%d,%s,%.3f,%.4f,%.4f",
                                            instanceName, numUnits, numRegions, RSD, r, gamma, numScenarios,
                                            useScenario ? "true" : "false",
                                            runtime, objectiveValue, outOfSamplePerformance
                                    ));
                                    csvWriter.newLine();

                                    // Flush to ensure real-time writing
                                    csvWriter.flush();
                                }
                            }
                        }
                    }
                }
            }
        } catch (IOException e) {
            System.err.println("Error writing results to CSV: " + e.getMessage());
        }

        System.out.println("Experiment completed, results saved to " + outputCSVPath);
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


    /**
     * Tests the out-of-sample performance of a solution by checking constraint satisfaction
     * across newly generated scenarios not used during optimization.
     *
     * @param instance         The problem instance
     * @param algo             The algorithm with a computed solution
     * @param E                Expected demand value
     * @param RSD              Relative standard deviation for demand generation
     * @param testSeed         Random seed for test scenario generation
     * @param r                Capacity tolerance parameter
     * @param numTestScenarios Number of test scenarios to generate
     * @return The percentage of test scenarios where constraints are satisfied
     */
    private static double testOutOfSamplePerformance(
            Instance instance,
            ChanceConstrainedAlgo algo,
            double E,
            double RSD,
            long testSeed,
            double r,
            int numTestScenarios) {

        // Generate test scenarios with a different seed than training
        double[][] testScenarios = generateScenarios(
                instance.getN(), numTestScenarios, E, RSD, testSeed);

        // Get the solution
        ArrayList<Integer>[] zones = algo.getZones();
        ArrayList<Area> centers = algo.getCenters();

        // Count satisfied scenarios
        int satisfiedScenarios = 0;

        // For each test scenario
        for (int s = 0; s < numTestScenarios; s++) {
            // Calculate this scenario's total demand
            double scenarioTotalDemand = 0;
            for (int i = 0; i < instance.getN(); i++) {
                scenarioTotalDemand += testScenarios[s][i];
            }

            // Calculate scenario-specific capacity limit
            double scenarioCapacityLimit = (1 + r) * (scenarioTotalDemand / instance.k);

            boolean scenarioSatisfied = true;

            // Check each district
            for (int j = 0; j < zones.length; j++) {
                if (zones[j] == null || zones[j].isEmpty()) {
                    continue;
                }

                // Calculate total demand for this district in this scenario
                double districtDemand = 0;
                for (int areaId : zones[j]) {
                    districtDemand += testScenarios[s][areaId];
                }

                // Check if capacity constraint is violated
                if (districtDemand > scenarioCapacityLimit) {
                    scenarioSatisfied = false;
                    break;
                }
            }

            if (scenarioSatisfied) {
                satisfiedScenarios++;
            }
        }

        // Return percentage of satisfied scenarios
        return (double) satisfiedScenarios / numTestScenarios;
    }

}