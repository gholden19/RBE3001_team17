public class VelocityTrajectory {
    double[] position;
    double velocity;
    public VelocityTrajectory(double[] position, double velocity){
        this.position=position;
        this.velocity=velocity;
    }

    public double[] getPosition() {
        return position;
    }

    public double getVelocity() {
        return velocity;
    }
}