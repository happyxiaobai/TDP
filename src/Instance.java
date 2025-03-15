import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintStream;
import java.util.Scanner;
import java.util.ArrayList;

public class Instance {
    private int n; // 区域个数
    private Area[] areas; // 区域信息
    private int[][] edges; // 边集合

    public double average1;
    public double average2;
    public int k;
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


