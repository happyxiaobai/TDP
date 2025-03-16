import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.Scanner;

public class Instance {
    private int n; // 区域个数
    private Area[] areas; // 区域信息
    private int[][] edges; // 边集合

    public double average1;
    public double average2;
    public int k; // 需要分成的大区域个数
    double[][] dist;

    public Instance(String filepath) throws FileNotFoundException {
        File file = new File(filepath);
        Scanner sc = new Scanner(file);

        n = sc.nextInt(); // 区域个数

        areas = new Area[n]; // 存储区域信息
        double sum = 0.0;
        double sum2 = 0.0;
        for (int i = 0; i < n; i++) {
            int id = sc.nextInt();
            double x = sc.nextDouble();
            double y = sc.nextDouble();


            double[] activeness = new double[3];
            for (int j = 0; j < 3; j++) {
                activeness[j] = sc.nextDouble();
            }
            sum += activeness[0];
            sum2 += activeness[1];
            areas[i] = new Area(id, x, y, activeness);
        }


        edges = new int[n][n];
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                edges[i][j] = 0;
            }
        }

        int m = sc.nextInt(); // 区域间的边数
        for (int i = 0; i < m; i++) {

            int a = sc.nextInt();
            int b = sc.nextInt();
            areas[a].addNeighbor(b);
            areas[b].addNeighbor(a);
            edges[a][b] = 1;
            edges[b][a] = 1;
        }

        k = sc.nextInt(); // 需要分成的大区域个数
        average1 = sum / k;
        average2 = sum2 / k;
        sc.close();

        // 生成距离矩阵
        dist = new double[n][n];
        for (int i = 0; i < n; i++) {
            for (int j = i + 1; j < n; j++) {
                double distance = Math.sqrt(Math.pow(areas[i].getX() - areas[j].getX(), 2) + Math.pow(areas[i].getY() - areas[j].getY(), 2));
                dist[i][j] = distance;
                dist[j][i] = dist[i][j];
            }
        }
    }

    public Instance() {

    }

    /**
     * 创建当前实例的深度克隆
     * @return 克隆的实例对象
     */
    public Instance clone() {
        Instance cloned = new Instance();
        cloned.n = this.n;
        cloned.k = this.k;
        cloned.average1 = this.average1;
        cloned.average2 = this.average2;

        // 复制区域信息
        cloned.areas = new Area[this.n];
        for (int i = 0; i < this.n; i++) {
            cloned.areas[i] = new Area();
            cloned.areas[i].setId(this.areas[i].getId());
            cloned.areas[i].setX(this.areas[i].getX());
            cloned.areas[i].setY(this.areas[i].getY());

            // 复制活跃度指标
            double[] originalActiveness = this.areas[i].getActiveness();
            double[] newActiveness = new double[originalActiveness.length];
            System.arraycopy(originalActiveness, 0, newActiveness, 0, originalActiveness.length);
            cloned.areas[i].setActiveness(newActiveness);

            // 复制中心标识
            cloned.areas[i].setCenter(this.areas[i].isCenter());

            // 复制邻居列表
            ArrayList<Integer> neighbors = new ArrayList<>(this.areas[i].getNeighbors());
            for (int neighbor : this.areas[i].getNeighbors()) {
                cloned.areas[i].addNeighbor(neighbor);
            }
        }

        // 复制边信息
        cloned.edges = new int[this.n][this.n];
        for (int i = 0; i < this.n; i++) {
            for (int j = 0; j < this.n; j++) {
                cloned.edges[i][j] = this.edges[i][j];
            }
        }

        // 复制距离矩阵
        cloned.dist = new double[this.n][this.n];
        for (int i = 0; i < this.n; i++) {
            for (int j = 0; j < this.n; j++) {
                cloned.dist[i][j] = this.dist[i][j];
            }
        }

        return cloned;
    }

    /**
     * 基于原始实例创建新实例，使用特定场景的需求
     * @param original 原始实例
     * @param scenarioDemands 场景需求数组
     */
    public Instance(Instance original, int[] scenarioDemands) {
        this.n = original.n;
        this.k = original.k;
        this.average1 = original.average1;
        this.average2 = original.average2;

        // 复制区域信息，但使用新的需求值
        this.areas = new Area[this.n];
        double sum1 = 0.0;
        double sum2 = 0.0;

        for (int i = 0; i < this.n; i++) {
            this.areas[i] = new Area();
            this.areas[i].setId(original.areas[i].getId());
            this.areas[i].setX(original.areas[i].getX());
            this.areas[i].setY(original.areas[i].getY());

            // 使用场景需求作为第一个活跃度指标
            double[] originalActiveness = original.areas[i].getActiveness();
            double[] newActiveness = new double[originalActiveness.length];
            System.arraycopy(originalActiveness, 0, newActiveness, 0, originalActiveness.length);

            // 更新第一个活跃度指标为场景需求
            newActiveness[0] = scenarioDemands[i];
            this.areas[i].setActiveness(newActiveness);
            sum1 += newActiveness[0];
            sum2 += newActiveness[1]; // 保留原来的第二个指标

            // 复制中心标识
            this.areas[i].setCenter(original.areas[i].isCenter());

            // 复制邻居列表
            for (int neighbor : original.areas[i].getNeighbors()) {
                this.areas[i].addNeighbor(neighbor);
            }
        }

        // 更新平均值
        this.average1 = sum1 / this.k;
        this.average2 = sum2 / this.k;

        // 复制边信息
        this.edges = new int[this.n][this.n];
        for (int i = 0; i < this.n; i++) {
            for (int j = 0; j < this.n; j++) {
                this.edges[i][j] = original.edges[i][j];
            }
        }

        // 复制距离矩阵
        this.dist = new double[this.n][this.n];
        for (int i = 0; i < this.n; i++) {
            for (int j = 0; j < this.n; j++) {
                this.dist[i][j] = original.dist[i][j];
            }
        }
    }

    public int getN() {
        return n;
    }

    public void setN(int n) {
        this.n = n;
    }

    public Area[] getAreas() {
        return areas;
    }

    public void setAreas(Area[] areas) {
        this.areas = areas;
    }

    public int[][] getEdges() {
        return edges;
    }

    public void output() {
        System.out.println("区域个数: " + n);
        System.out.println("区域信息:");
        for (Area area : areas) {
            System.out.println("区域编号：" + area.getId() + "，横坐标：" + area.getX() + "，纵坐标：" + area.getY() +
                    "，活跃度指标：{" + area.getActiveness()[0] + ", " + area.getActiveness()[1] + ", " + area.getActiveness()[2] + "}");
        }
        System.out.println("边集合:");
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                System.out.print(edges[i][j] + " ");
            }
            System.out.println();
        }
    }


    public void setEdges(int[][] edges) {
        this.edges = edges;
    }
}


class Area {
    private int id; // 区域编号
    private double x; // 横坐标
    private double y; // 纵坐标
    private double[] activeness; // 活跃度指标
    private boolean isCenter = false; // 是否是大区域中心
    private ArrayList<Integer> neighbors; // 存储所有相邻区域的编号

    public Area() {
    }

    public Area(int id, double x, double y, double[] activeness) {
        this.id = id;
        this.x = x;
        this.y = y;
        this.activeness = activeness;
        this.isCenter = false; // 默认不是大区域中心
        this.neighbors = new ArrayList<>();
    }

    public int getId() {
        return id;
    }

    public void setId(int id) {
        this.id = id;
    }

    public double getX() {
        return x;
    }

    public void setX(double x) {
        this.x = x;
    }

    public double getY() {
        return y;
    }

    public void setY(double y) {
        this.y = y;
    }

    public double[] getActiveness() {
        return activeness;
    }

    public void setActiveness(double[] activeness) {
        this.activeness = activeness;
    }

    public boolean isCenter() {
        return isCenter;
    }

    public void setCenter(boolean center) {
        isCenter = center;
    }

    public ArrayList<Integer> getNeighbors() {
        return neighbors;
    }

    public void addNeighbor(int neighborId) {
        neighbors.add(neighborId);
    }
}


