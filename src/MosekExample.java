public class MosekExample {
    public static void main(String[] args) {
        try {
            // Create a task object and attach a log handler
            try (mosek.Env env = new mosek.Env()) {
                try (mosek.Task task = new mosek.Task(env, 0, 0)) {
                    // Direct the task's output to the console
                    task.set_Stream(
                            mosek.streamtype.log,
                            new mosek.Stream() {
                                public void stream(String msg) {
                                    System.out.print(msg);
                                }
                            });

                    // Define your optimization problem here
                    // For example, a simple LP problem
                    task.appendvars(3); // 3 variables
                    task.appendcons(1); // 1 constraint

                    // Set variable bounds
                    for (int j = 0; j < 3; ++j)
                        task.putvarbound(j, mosek.boundkey.lo, 0.0, +1.0e+30);

                    // Set constraint bounds
                    double[] aval = {1.0, 1.0, 1.0};
                    int[] asub = {0, 1, 2};
                    task.putarow(0, asub, aval);
                    task.putconbound(0, mosek.boundkey.up, -1.0e+30, 1.0);

                    // Set objective coefficients
                    task.putcj(0, 1.0);
                    task.putcj(1, 2.0);
                    task.putcj(2, 0.0);

                    // Set objective sense (minimize or maximize)
                    task.putobjsense(mosek.objsense.minimize);

                    // Solve the problem
                    task.optimize();

                    // Print a summary of the solution
                    task.solutionsummary(mosek.streamtype.msg);

                    // Get the solution values
                    double[] xx = new double[3];
                    task.getxx(mosek.soltype.bas, xx);

                    System.out.println("Solution: x = " + xx[0] + ", y = " + xx[1] + ", z = " + xx[2]);
                }
            }
        } catch (mosek.Exception e) {
            System.out.println("Mosek error: " + e.toString());
        }
    }
}