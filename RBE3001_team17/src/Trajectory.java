public class Trajectory {
    double[][] coefficients;
    double time;
    boolean jointspace;
    public Trajectory(double[][] coefficients, double time, boolean jointspace){
        this.coefficients=coefficients;
        this.time=time;
        this.jointspace=jointspace;
    }

    public double[][] getCoefficients() {
        return coefficients;
    }

    public double getTime() {
        return time;
    }
    public boolean getJointspace(){
        return jointspace;
    }
}
