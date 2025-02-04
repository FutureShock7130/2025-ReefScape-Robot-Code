package frc.FSLib2025.math;

public class LinearRegression {
    
    private double[][] data;

    public LinearRegression (double[][] mapValues) {
        data = mapValues;
    }

    public double calculate (double xValues) {
        int index = 0;
        for (double[] i : data) {
            if(i[0] >= xValues) break;
            index++;
        }
        double dx = xValues - data[index-1][0];
        double x = data[index][0] - data[index-1][0];
        return data[index-1][1] * (1-dx/x) + data[index][1] * dx/x;
    }

}
