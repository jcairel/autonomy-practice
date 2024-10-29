#include "cmath"
#include "random"
#include "algorithm"


float max_steering_angle = M_PI / 4.0;
float bearing_noise = 0.1; 
float steering_noise = 0.1; 
float distance_noise = 5.0; 

float world_size = 100.0; // world is NOT cyclic. Robot is allowed to travel "out of bounds"
std::vector<std::vector<float>> landmarks = {{0.0, 100.0}, {0.0, 0.0}, {100.0, 0.0}, {100.0, 100.0}};

class robot{
private:
    float bearing_noise = 0.0;
    float steering_noise = 0.0;
    float distance_noise = 0.0;
    std::mt19937 generator;
    

public:
    float x;
    float y;
    float orientation;
    float length;
    
    robot(float new_length = 20.0);
    void set(float new_x, float new_y, float new_orientation);
    void set_noise(float new_b_noise, float new_s_noise, float new_d_noise);
    float measurement_prob(std::vector<float> measurements);
    void move(std::vector<float> movement);
    std::vector<float> sense(bool add_noise=true);
};

robot::robot(float set_length){
    std::uniform_real_distribution<> dist(0, 1);
    x = dist(generator) * world_size;
    y = dist(generator) * world_size;
    orientation = dist(generator) * 2.0 * M_PI;
    length = set_length;
}

void robot::set(float new_x, float new_y, float new_orientation){
    x = new_x;
    y = new_y;
    orientation = fmod(new_orientation, 2.0 * M_PI);
}

void robot::set_noise(float new_b_noise, float new_s_noise, float new_d_noise){
    bearing_noise = new_b_noise;
    steering_noise = new_s_noise;
    distance_noise = new_d_noise;
}


float robot::measurement_prob(std::vector<float> measurements){
    std::vector<float> predicted_measuments = sense(false);
    double error = 1.0;
    for (int i = 0; i < measurements.size(); i++){
        float error_bearing = fabs(measurements[i] - predicted_measuments[i]);
        error_bearing = fmod(error_bearing + M_PI, (2.0 * M_PI)) - M_PI;
        // Update Gaussian
        long double temp = (-1.0 * std::pow(error_bearing, 2.0) / std::pow(bearing_noise, 2.0) / 2.0);
        error = error * (std::exp(temp) / 
            std::sqrt(2.0 * M_PI * std::pow(bearing_noise, 2.0)));
        //printf("eror: %.5lf \n", temp);            
    }
    
    return error;
}

void robot::move(std::vector<float> movement){
    float steering = movement[0];
    float distance = movement[1];

    // Apply noise and drift to movement
    std::normal_distribution<double> dist(steering, steering_noise);
    double steering2 = dist(generator);
    steering2 += steering_noise;
    dist = std::normal_distribution<double>(distance, distance_noise);
    double distance2 = dist(generator);
    if (distance < 0.0) distance = 0.0;

    float turn = tan(steering2) * distance2 / length;
    if (fabs(turn) < 0.001){
        // Aproximate as straight line motion
        x += (distance2 * cos(orientation));
        y += (distance2 * sin(orientation));
        orientation = fmod(orientation + turn, 2.0 * M_PI);
    } 
    else{
        // Aproximate bicycle model for motion
        float radius = distance2 / turn;
        float cx = x - (sin(orientation) * radius);
        float cy = y + (cos(orientation) * radius);
        orientation = fmod(orientation + turn, 2.0 * M_PI);
        x = cx + (sin(orientation) * radius);
        y = cy - (cos(orientation) * radius);
    }
}

std::vector<float> robot::sense(bool add_noise){
    std::vector<float> Z;
    for (std::vector<float> landmark : landmarks){
        float dx =  landmark[1] - x;
        float dy = landmark[0] - y;
        float res = std::fmod(std::atan2(dy, dx) - orientation, 2.0 * M_PI);
        if (res < 0) res += 2.0 * M_PI;
        if (add_noise){
            std::normal_distribution<double> dist(0, bearing_noise);
            res += dist(generator);
        }
        Z.push_back(res);
    }
    return Z;
}

std::vector<float> get_position(std::vector<robot*> p){
    float x = 0.0, y = 0.0, orientation = 0.0;
    for (int i = 0; i < p.size(); i++){
        x += p[i]->x;
        y += p[i]->y;
        // Orientation is tricky because it is cyclic. By normalizing
        // around the first particle we are somewhat more robust to the 0=2pi problem
        orientation += (fmod(p[i]->orientation - p[0]->orientation + M_PI, 2.0 * M_PI) 
                        + p[0]->orientation - M_PI);
    }
    return {x / float(p.size()), y / float(p.size()), orientation / float(p.size())};
}

std::vector<std::vector<float>> generate_ground_truth(robot* myRobot, std::vector<std::vector<float>> movements){
    std::vector<std::vector<float>> Z;
    for (std::vector<float> movement : movements){
        myRobot->move(movement);
        Z.push_back(myRobot->sense());
    }
    return Z;
}

bool check_output(robot* finalRobot, std::vector<float> estimated_position){
    float tolerance_xy = 15.0; 
    float tolerance_orientation = 0.25; 

    float error_x = fabs(finalRobot->x - estimated_position[0]);
    float error_y = fabs(finalRobot->y - estimated_position[1]);
    float error_orientation = fabs(finalRobot->orientation - estimated_position[2]);
    error_orientation = fmod(error_orientation + M_PI, 2.0 * M_PI) - M_PI;
    return error_x < tolerance_xy && error_y < tolerance_xy && error_orientation < tolerance_orientation;
}

std::vector<float> particle_filter(std::vector<std::vector<float>> movements, std::vector<std::vector<float>> measurements, int N = 500){
    std::vector<robot*> p;
    robot* temp;
    for (int i = 0; i < N; i++){
        temp = new robot();
        temp->set_noise(bearing_noise, steering_noise, distance_noise);
        p.push_back(temp);
    }

    std::uniform_real_distribution<float> dist(0, 1);
    std::mt19937 generator;
    for (int t = 0; t < movements.size(); t++){
        // move all particles
        for (int i = 0; i < N; i++){
            p[i]->move(movements[t]);
        }
        // Update weights
        std::vector<float> w;
        for (int i = 0; i < N; i++){
            w.push_back(p[i]->measurement_prob(measurements[t]));
            //printf("%f \n", p[i]->measurement_prob(movements[t]));
        }
        // Resampling
        std::vector<robot*> p2;
        int index = int(dist(generator) * N);
        float beta = 0.0;
        float maxW = *(std::max_element(w.begin(), w.end()));
        //printf("%f \n", maxW);
        for (int i = 0; i < N; i++){
            beta += dist(generator) * 2.0 * maxW;
            while (beta > w[index]){
                beta -= w[index];
                index = (index + 1) % N;
            }
            p2.push_back(p[index]);
        }
        p = p2;
    }
    return get_position(p);
}

int main(){
    std::vector<std::vector<float>> motions = {{2.0 * M_PI / 10, 20.0}, {2.0 * M_PI / 10, 20.0}, 
    {2.0 * M_PI / 10, 20.0}, {2.0 * M_PI / 10, 20.0}, {2.0 * M_PI / 10, 20.0}, {2.0 * M_PI / 10, 20.0},
    {2.0 * M_PI / 10, 20.0}, {2.0 * M_PI / 10, 20.0}};
    std::vector<std::vector<float>> measurements = {{4.746936, 3.859782, 3.045217, 2.045506},
               {3.510067, 2.916300, 2.146394, 1.598332},
               {2.972469, 2.407489, 1.588474, 1.611094},
               {1.906178, 1.193329, 0.619356, 0.807930},
               {1.352825, 0.662233, 0.144927, 0.799090},
               {0.856150, 0.214590, 5.651497, 1.062401},
               {0.194460, 5.660382, 4.761072, 2.471682},
               {5.717342, 4.736780, 3.909599, 2.342536}};
    std::vector<float> position = particle_filter(motions, measurements);
    printf("[x=%.5f y=%.5f orientation=%.5f]\n", position[0], position[1], position[2]);
    //printf("Ground Truth: [x=%.5f y=%.5f orientation=%.5f]\n", );
    robot* myRobot = new robot();
    myRobot->set_noise(bearing_noise, steering_noise, distance_noise);
    myRobot->set(93.476, 75.186, 5.2664);
    std::vector<robot*> test = {myRobot};
    std::vector<float> last = myRobot->sense(false);
    printf("{%.5f, %.5f, %.5f, %.5f}", last[0], last[1], last[2], last[3]);
    

    // TODO: Change fmod so it stays in range[0, 2pi] 
    // right now can be negative number and is throwing things off
}