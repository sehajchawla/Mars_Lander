

#include "lander.h"

void autopilot (void)
// Autopilot to adjust the engine throttle, parachute and attitude control
{
    double Kh;
    Kh = 0.0117;// Kh=0.0117 for scenario 1, Kh=0.016  for scenario 5
    double h;
    h= position.abs() - MARS_RADIUS; //h is altitude
    double e;
    e= - (0.5 + Kh*h + velocity * position.norm()); //given in handout
    
    double Kp;
    Kp = 0.7; //from trial and error
    
    double Pout;
    double delta;
    delta = 2/3;
    cout<< e << endl;
    Pout = Kp * e; //given in handout
    
    
    if (Pout <= - delta ) {
        
        throttle = 0;
    }
    
    else {
        
        if (1 - delta > Pout && Pout > - delta) {
            
            throttle = delta + Pout;
        }
        else {
            throttle = 2;
        }
        
        
    }
    
    ofstream fout;
    
    fout.open("dat_output.dat", ios::app);
    
    fout << h << " " << -(0.5+ Kh*h) << " " << velocity << endl;
    fout.close();
    
}

void numerical_dynamics (void)
// This is the function that performs the numerical integration to update the
// lander's pose. The time step is delta_t (global variable).
{
    static vector3d previous_position;
    double LOADED_LANDER_MASS;
    static vector3d GRAVITY_FORCE;
    static vector3d DRAG_FORCE;
    static vector3d current_position;
    static vector3d acceleration;
    current_position=position;
    LOADED_LANDER_MASS=UNLOADED_LANDER_MASS+ FUEL_CAPACITY*FUEL_DENSITY* fuel;
    GRAVITY_FORCE= (GRAVITY * LOADED_LANDER_MASS * MARS_MASS /(position * position))* ((-position).norm());
    
    if (parachute_status == 0) {
        
        DRAG_FORCE = (-0.5)*(atmospheric_density(current_position))*(M_PI)*(pow(LANDER_SIZE,2))*(DRAG_COEF_LANDER)*(velocity*velocity)*(velocity.norm());
    }
    
    else {
        
        DRAG_FORCE = (-0.5)*(atmospheric_density(position))*(velocity * velocity)*(DRAG_COEF_LANDER*M_PI*(pow(LANDER_SIZE,2)) + DRAG_COEF_CHUTE*5*4*(pow(LANDER_SIZE,2)))*(velocity.norm());
    }
    
    acceleration = (thrust_wrt_world() + GRAVITY_FORCE + DRAG_FORCE)/LOADED_LANDER_MASS;
    if (simulation_time < delta_t) {
        position = position + delta_t*velocity;
        velocity = velocity + delta_t*acceleration;
    }
    else {
        position = 2*position + acceleration*(pow(delta_t,2)) - previous_position ;
        velocity = (position - previous_position)/(2*delta_t);
    }
    previous_position = current_position;
    
    
    
    // Here we can apply an autopilot to adjust the thrust, parachute and attitude
    if (autopilot_enabled) autopilot();
    
    // Here we can apply 3-axis stabilization to ensure the base is always pointing downwards
    if (stabilized_attitude) attitude_stabilization();
}

void initialize_simulation (void)
// Lander pose initialization - selects one of 10 possible scenarios
{
    // The parameters to set are:
    // position - in Cartesian planetary coordinate system (m)
    // velocity - in Cartesian planetary coordinate system (m/s)
    // orientation - in lander coordinate system (xyz Euler angles, degrees)
    // delta_t - the simulation time step
    // boolean state variables - parachute_status, stabilized_attitude, autopilot_enabled
    // scenario_description - a descriptive string for the help screen
    
    scenario_description[0] = "circular orbit";
    scenario_description[1] = "descent from 10km";
    scenario_description[2] = "elliptical orbit, thrust changes orbital plane";
    scenario_description[3] = "polar launch at escape velocity (but drag prevents escape)";
    scenario_description[4] = "elliptical orbit that clips the atmosphere and decays";
    scenario_description[5] = "descent from 200km";
    scenario_description[6] = "";
    scenario_description[7] = "";
    scenario_description[8] = "";
    scenario_description[9] = "";
    
    switch (scenario) {
            
        case 0:
            // a circular equatorial orbit
            position = vector3d(1.2*MARS_RADIUS, 0.0, 0.0);
            velocity = vector3d(0.0, -3247.087385863725, 0.0);
            orientation = vector3d(0.0, 90.0, 0.0);
            delta_t = 0.1;
            parachute_status = NOT_DEPLOYED;
            stabilized_attitude = false;
            autopilot_enabled = false;
            break;
            
        case 1:
            // a descent from rest at 10km altitude
            position = vector3d(0.0, -(MARS_RADIUS + 10000.0), 0.0);
            velocity = vector3d(0.0, 0.0, 0.0);
            orientation = vector3d(0.0, 0.0, 90.0);
            delta_t = 0.1;
            parachute_status = NOT_DEPLOYED;
            stabilized_attitude = true;
            autopilot_enabled = false;
            break;
            
        case 2:
            // an elliptical polar orbit
            position = vector3d(0.0, 0.0, 1.2*MARS_RADIUS);
            velocity = vector3d(3500.0, 0.0, 0.0);
            orientation = vector3d(0.0, 0.0, 90.0);
            delta_t = 0.1;
            parachute_status = NOT_DEPLOYED;
            stabilized_attitude = false;
            autopilot_enabled = false;
            break;
            
        case 3:
            // polar surface launch at escape velocity (but drag prevents escape)
            position = vector3d(0.0, 0.0, MARS_RADIUS + LANDER_SIZE/2.0);
            velocity = vector3d(0.0, 0.0, 5027.0);
            orientation = vector3d(0.0, 0.0, 0.0);
            delta_t = 0.1;
            parachute_status = NOT_DEPLOYED;
            stabilized_attitude = false;
            autopilot_enabled = false;
            break;
            
        case 4:
            // an elliptical orbit that clips the atmosphere each time round, losing energy
            position = vector3d(0.0, 0.0, MARS_RADIUS + 100000.0);
            velocity = vector3d(4000.0, 0.0, 0.0);
            orientation = vector3d(0.0, 90.0, 0.0);
            delta_t = 0.1;
            parachute_status = NOT_DEPLOYED;
            stabilized_attitude = false;
            autopilot_enabled = false;
            break;
            
        case 5:
            // a descent from rest at the edge of the exosphere
            position = vector3d(0.0, -(MARS_RADIUS + EXOSPHERE), 0.0);
            velocity = vector3d(0.0, 0.0, 0.0);
            orientation = vector3d(0.0, 0.0, 90.0);
            delta_t = 0.1;
            parachute_status = NOT_DEPLOYED;
            stabilized_attitude = true;
            autopilot_enabled = false;
            break;
            
        case 6:
            break;
            
        case 7:
            break;
            
        case 8:
            break;
            
        case 9:
            break;
            
    }
}
