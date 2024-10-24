#include "cmath"
#include "random"
class robot{
private:
    float length;
    float x = 0.0;
    float y = 0.0;
    float orientation = 0.0;
    float steering_noise = 0.0;
    float distance_noise = 0.0;
    float steering_drift = 0.0;
    std::default_random_engine generator;
public:    
    robot(float l = 20.0){
        length = l;
    }
    void set(float new_x, float new_y, float new_orientation);
    void set_noise(float new_s_noise, float new_d_noise);
    void set_steering_drift(float new_drift);
    void move(float steering, float distance, float tolerance=0.001, float max_steering_angle=M_PI / 4.0);
    float cte(float radius);
    void show();
};

void robot::set(float new_x, float new_y, float new_orientation){
    x = new_x;
    y = new_y;
    orientation = new_orientation;
}
void robot::set_noise(float new_s_noise, float new_d_noise){
    steering_noise = new_s_noise;
    distance_noise = new_d_noise; 
}
void robot::set_steering_drift(float new_drift){
    steering_drift = new_drift;
}

void robot::move(float steering, float distance, float tolerance, float max_steering_angle){
    if (steering > max_steering_angle) steering = max_steering_angle;
    if (steering < -max_steering_angle) steering = -max_steering_angle;

    // Apply noise and drift to movement
    std::normal_distribution<double> dist(steering, steering_noise);
    double steering2 = dist(generator);
    steering2 += steering_drift;
    dist = std::normal_distribution<double>(distance, distance_noise);
    double distance2 = dist(generator);
    if (distance < 0.0) distance = 0.0;

    float turn = tan(steering2) * distance2 / length;
    if (fabs(turn) < tolerance){
        // Aproximate as straight line motion
        x += (distance2 * cos(orientation));
        y += (distance2 * sin(orientation));
        orientation = fmod((orientation + turn), (2.0 * M_PI));
    } 
    else{
        // Aproximate bicycle model for motion
        float radius = distance2 / turn;
        float cx = x - (sin(orientation) * radius);
        float cy = y + (cos(orientation) * radius);
        orientation = fmod((orientation + turn), (2.0 * M_PI));
        x = cx + (sin(orientation) * radius);
        y = cy - (cos(orientation) * radius);
    }
}

float robot::cte(float radius){
    float cte;
    // On curves
    if (x < radius){
        cte = sqrt(pow((x - radius), 2.0) + pow((y - radius), 2.0)) - radius;
    }
    else if (x > 3.0 * radius){
        cte = sqrt(pow((x - 3.0 * radius), 2.0) + pow((y - radius), 2.0)) - radius;
    }
    // On straightaway
    else if (y > radius){
        cte = y - 2.0 * radius;
    }
    else{
        cte = -y;
    }
    return cte;
}

void robot::show(){
    printf("[x=%.5f y=%.5f orientation=%.5f]\n", x, y, orientation);
}

float run(std::vector<float> params, float radius, bool printflag=false){
    robot myRobot;
    myRobot.set(0.0, radius, M_PI / 2.0);
    double speed = 1.0;
    double err = 0.0;
    int N = 200;
    double int_crosstrack_error = 0.0;
    double crosstrack_error = myRobot.cte(radius);

    for (int i = 0; i < N * 2; i++){
        double diff_crosstrack_error = -crosstrack_error;
        crosstrack_error = myRobot.cte(radius);
        diff_crosstrack_error += crosstrack_error;
        int_crosstrack_error += crosstrack_error;
        // Apply PID control
        float steer = -params[0] * crosstrack_error - params[1] * diff_crosstrack_error - params[2] * int_crosstrack_error;
        myRobot.move(steer, speed);

        if (i >= N){
            err += pow(crosstrack_error, 2);
        }
        if (printflag){
            myRobot.show();
        }
    }
    return err / float(N);
}
int main(){
    float radius = 25.0;
    std::vector<float> params = {10.0, 15.0, 0.0};
    float err = run(params, radius, true);
    printf("\nerror -> %.5f\n", err);
    return 0;
}


