#include "math.h"
#include "F28335Serial.h"

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398
#define GRAV        9.81

// encoder offsets
float offset_Enc2_rad =-0.433365253;
float offset_Enc3_rad =0.229336264;
// Your global variables.

long mycount = 0;

#pragma DATA_SECTION(whattoprint, ".my_vars")
float whattoprint = 0.0;

#pragma DATA_SECTION(whattoprint2, ".my_vars")
float whattoprint2 = 0.1;

#pragma DATA_SECTION(theta1array, ".my_arrs")
float theta1array[100];

#pragma DATA_SECTION(theta2array, ".my_arrs")
float theta2array[100];

long arrayindex = 0;
int UARTprint = 0;

// joint torques to be commanded
float t1=0;
float t2=0;
float t3=0;

// end effector position [m]
float x=0;
float y=0;
float z=0;

// previous end effector position [m]
float xold=0;
float yold=0;
float zold=0;

// end effector velocity at current time and previous 2 time steps [m/s]
float vxold1=0;
float vxold2=0;
float vx=0;
float vyold1=0;
float vyold2=0;
float vy=0;
float vzold1=0;
float vzold2=0;
float vz=0;

// current joint velocities [rad/s]
float w1=0;
float w2=0;
float w3=0;

// old joint positions [rad]
float t1old = 0;
float t2old = 0;
float t3old = 0;

// old joint velocities [rad/s]
float w1old1=0;
float w1old2=0;
float w2old1=0;
float w2old2=0;
float w3old1=0;
float w3old2=0;

// desired end effector position [m]
float xd=0.3;
float yd=0;
float zd=0.3;

// desired end effector velocity [m/s]
float vxd=0;
float vyd=0;
float vzd=0;

// values to be plotted in Simulink
float Simulink_PlotVar1 = 0;
float Simulink_PlotVar2 = 0;
float Simulink_PlotVar3 = 0;
float Simulink_PlotVar4 = 0;

// joint friction terms for joint 1
float fvp1=0.2513;
float fvn1=0.2477;
float fcp1=0.3637;
float fcn1=-0.2948;
float wMin1=0.1;
float slopeC1=3.6;
float ufric1=0;

// joint friction terms for joint 2
float fvp2=0.25;
float fvn2=0.2870;
float fcp2=0.1675;
float fcn2=-0.1675;
float wMin2=0.05;
float slopeC2=3.6;
float ufric2=0;

// joint friction terms for joint 3
float fvp3=0.25;
float fvn3=0.287;
float fcp3=0.1675;
float fcn3=-0.1675;
float wMin3=0.05;
float slopeC3=3.6;
float ufric3=0;

// joint angle trig function terms
float cosq1 = 0;
float sinq1 = 0;
float cosq2 = 0;
float sinq2 = 0;
float cosq3 = 0;
float sinq3 = 0;

// end effector jacobian transpose elements
float JT_11 = 0;
float JT_12 = 0;
float JT_13 = 0;
float JT_21 = 0;
float JT_22 = 0;
float JT_23 = 0;
float JT_31 = 0;
float JT_32 = 0;
float JT_33 = 0;

// rotation matrix angle trig terms
float cosz = 0;
float sinz = 0;
float cosx = 0;
float sinx = 0;
float cosy = 0;
float siny = 0;

// rotation matrix angles
float thetaz = 0;
float thetax = 0;
float thetay = 0;

// rotation matrix and rotation matrix transpose elements
float R11 = 0;
float R12 = 0;
float R13 = 0;
float R21 = 0;
float R22 = 0;
float R23 = 0;
float R31 = 0;
float R32 = 0;
float R33 = 0;
float RT11 = 0;
float RT12 = 0;
float RT13 = 0;
float RT21 = 0;
float RT22 = 0;
float RT23 = 0;
float RT31 = 0;
float RT32 = 0;
float RT33 = 0;

// task space PD control gains
float kpx=300;
float kpy=300;
float kpz=300;
float kdx=20;
float kdy=20;
float kdz=20;

// friction factor for feedforward force
float ff=0.7;

// commanded force in z dir
float fzcmd=0;

// torque constant
float kt=6.0;

//straight line trajectory constants
float tTot=0; //total time to complete trajectory
float ts=0; //time at start of line trajectory

// initial to final position distances
float dxStraight=0;
float dyStraight=0;
float dzStraight=0;

// line initial position
float xaLine=0;
float yaLine=0;
float zaLine=0;

// intermediate time variable
float t=0;

// state variable used to determine which trajectory the end effector should follow
int state = 0;

// state variable used to determine if feed forward or impedance control should be used
int feedfwd = 0;

// height of egg
float eggH = 0.273;


void mains_code(void);

//
// Main
//
void main(void)
{
    mains_code();
}

void rotTrig()
{
    /*
     * Calculates trig terms for the rotation matrix using global variables
     */
    cosz = cos(thetaz);
    sinz = sin(thetaz);
    cosx = cos(thetax);
    sinx = sin(thetax);
    cosy = cos(thetay);
    siny = sin(thetay);
}

void zxyRot()
{
    /*
     * Calculates terms of the rotation matrix and its transpose
     * Note that rotTrig is expected to be called before this function is run to set the trig terms
     */
    RT11 = R11 = cosz*cosy-sinz*sinx*siny;
    RT21 = R12 = -sinz*cosx;
    RT31 = R13 = cosz*siny+sinz*sinx*cosy;
    RT12 = R21 = sinz*cosy+cosz*sinx*siny;
    RT22 = R22 = cosz*cosx;
    RT32 = R23 = sinz*siny-cosz*sinx*cosy;
    RT13 = R31 = -cosx*siny;
    RT23 = R32 = sinx;
    RT33 = R33 = cosx*cosy;
}

void forwardKin(float theta1motor, float theta2motor, float theta3motor)
{
    /*
     * Forward kinematics to find the position of the end effector in the world frame
     */
    x = 0.254*cos(theta1motor)*(cos(theta3motor) + sin(theta2motor));
    y=0.254*sin(theta1motor)*(cos(theta3motor) + sin(theta2motor));
    z=0.254*cos(theta2motor) - 0.254*sin(theta3motor) + 0.254;
}

void eeVel()
{
    /*
     * Calculates the current velocity of the end effector and applies a filter to reduce noise
     */
    vx = (x-xold)/0.001;
    vx = (vx+vxold1+vxold2)/3.0;
    xold=x;
    vxold1=vx;
    vxold2=vxold1;

    vy = (y-yold)/0.001;
    vy = (vy+vyold1+vyold2)/3.0;
    yold=y;
    vyold1=vy;
    vyold2=vyold1;

    vz = (z-zold)/0.001;
    vz = (vz+vzold1+vzold2)/3.0;
    zold=z;
    vzold1=vz;
    vzold2=vzold1;
}

void jointVel(float theta1motor, float theta2motor, float theta3motor)
{
    /*
     * Calculates joint velocities and applies a filter to reduce noise.
     */
    w1 = (theta1motor-t1old)/0.001;
    w1 = (w1 + w1old1 + w1old2)/3.0;
    t1old=theta1motor;
    w1old1=w1;

    w2 = (theta2motor-t2old)/0.001;
    w2 = (w2 + w2old1 + w2old2)/3.0;
    t2old=theta2motor;
    w2old2=w2old1;
    w2old1=w2;

    w3 = (theta3motor-t3old)/0.001;
    w3 = (w3 + w3old1 + w3old2)/3.0;
    t3old=theta3motor;
    w3old2=w3old1;
    w3old1=w3;
}

void frictionComp()
{
    /*
     * Calculates friction compensation for each of the joints.
     */
    if(w1>wMin1)
        ufric1 = fvp1*w1+fcp1;
    else if(w1<-wMin1)
        ufric1 = fvn1*w1+fcn1;
    else
        ufric1 = slopeC1*w1;

    if(w2>wMin2)
        ufric2 = fvp2*w2+fcp2;
    else if(w2<-wMin2)
        ufric2 = fvn2*w2+fcn2;
    else
        ufric2 = slopeC2*w2;

    if(w3>wMin3)
        ufric3 = fvp3*w3+fcp3;
    else if(w3<-wMin3)
        ufric3 = fvn3*w3+fcn3;
    else
        ufric3 = slopeC3*w3;
}

void feedforwardForce()
{
    /*
     * Calculates joint torques to apply a desired force in the z direction.
     * The jacobian transpose is used to convert between required forces in the task space to torques in the joint space.
     * Algebraic expressions for each joint torque calculated using matlab.
     */
    t1 = JT_13*(kdz*(vzd - vz) - kpz*(z - zd)) - JT_11*(kdx*(vx - vxd) + kpx*(x - xd)) - JT_12*(kdy*(vy - vyd) + kpy*(y - yd)) + ff*ufric1 + (JT_13*fzcmd)/kt;
    t2 = JT_23*(kdz*(vzd - vz) - kpz*(z - zd)) - JT_21*(kdx*(vx - vxd) + kpx*(x - xd)) - JT_22*(kdy*(vy - vyd) + kpy*(y - yd)) + ff*ufric2 + (JT_23*fzcmd)/kt;
    t3 = JT_33*(kdz*(vzd - vz) - kpz*(z - zd)) - JT_31*(kdx*(vx - vxd) + kpx*(x - xd)) - JT_32*(kdy*(vy - vyd) + kpy*(y - yd)) + ff*ufric3 + (JT_33*fzcmd)/kt;
}

void impedanceControl()
{
    /*
     * Calculates joint torques required to implement impedance control.
     * Rotation matrix terms are used to transform the end effector world frame position to the N frame, where PD
     * control can then be applied. Applying control in the N frame allows arbitrary directions of the end effector
     * motion to be made free (which would be much more difficult in the world frame).
     * A rotation matrix is again used to transform desired forces from the N frame to the world frame.
     * The jacobian transpose is used to find joint torques from the desired forces.
     * Algebraic expressions for each joint torque calculated using matlab.
     */
    t1 = - (JT_11*R11 + JT_12*R21 + JT_13*R31)*(R11*kdx*(vx - vxd) + R21*kdx*(vy - vyd) + R31*kdx*(vz - vzd) + R11*kpx*(x - xd) + R21*kpx*(y - yd) + R31*kpx*(z - zd)) - (JT_11*R12 + JT_12*R22 + JT_13*R32)*(R12*kdy*(vx - vxd) + R22*kdy*(vy - vyd) + R32*kdy*(vz - vzd) + R12*kpy*(x - xd) + R22*kpy*(y - yd) + R32*kpy*(z - zd)) - (JT_11*R13 + JT_12*R23 + JT_13*R33)*(R13*kdz*(vx - vxd) + R23*kdz*(vy - vyd) + R33*kdz*(vz - vzd) + R13*kpz*(x - xd) + R23*kpz*(y - yd) + R33*kpz*(z - zd));
    t2 = - (JT_21*R11 + JT_22*R21 + JT_23*R31)*(R11*kdx*(vx - vxd) + R21*kdx*(vy - vyd) + R31*kdx*(vz - vzd) + R11*kpx*(x - xd) + R21*kpx*(y - yd) + R31*kpx*(z - zd)) - (JT_21*R12 + JT_22*R22 + JT_23*R32)*(R12*kdy*(vx - vxd) + R22*kdy*(vy - vyd) + R32*kdy*(vz - vzd) + R12*kpy*(x - xd) + R22*kpy*(y - yd) + R32*kpy*(z - zd)) - (JT_21*R13 + JT_22*R23 + JT_23*R33)*(R13*kdz*(vx - vxd) + R23*kdz*(vy - vyd) + R33*kdz*(vz - vzd) + R13*kpz*(x - xd) + R23*kpz*(y - yd) + R33*kpz*(z - zd));
    t3 = - (JT_31*R11 + JT_32*R21 + JT_33*R31)*(R11*kdx*(vx - vxd) + R21*kdx*(vy - vyd) + R31*kdx*(vz - vzd) + R11*kpx*(x - xd) + R21*kpx*(y - yd) + R31*kpx*(z - zd)) - (JT_31*R12 + JT_32*R22 + JT_33*R32)*(R12*kdy*(vx - vxd) + R22*kdy*(vy - vyd) + R32*kdy*(vz - vzd) + R12*kpy*(x - xd) + R22*kpy*(y - yd) + R32*kpy*(z - zd)) - (JT_31*R13 + JT_32*R23 + JT_33*R33)*(R13*kdz*(vx - vxd) + R23*kdz*(vy - vyd) + R33*kdz*(vz - vzd) + R13*kpz*(x - xd) + R23*kpz*(y - yd) + R33*kpz*(z - zd));
}

float norm(float x,float y,float z)
{
    /*
     * Calculates the length of a vector of length 3
     * Input args:
     *      x   : vector element 1
     *      y   : vector element 2
     *      z   : vector element 3
     */
    return sqrt(pow(x,2)+pow(y,2)+pow(z,2));
}

void lineTrajGen(float xa, float ya, float za, float xb, float yb, float zb, float v)
{
    /*
     * Calculates terms required for a straight line trajectory as P_des = P_init + V_des*(t-t_init)
     * Input args:
     *      xa  : initial x position of trajectory
     *      ya  : initial y position of trajectory
     *      za  : initial z position of trajectory
     *      xb  : final x position of trajectory
     *      yb  : final y position of trajectory
     *      zb  : final z position of trajectory
     *      v   : desired end effector velocity for path
     */

    // time for complete trajectory
    tTot = norm(xb-xa,yb-ya,zb-za)/v;
    // start time
    ts=t;
    // vector elements from initial to final position
    dxStraight = xb-xa;
    dyStraight = yb-ya;
    dzStraight = zb-za;
}

void goToLinGen(float xb, float yb, float zb, float v)
{
    /*
     * Sets linear trajectory terms from current position to new position at a desired velocity
     *      xb  : final x position of trajectory
     *      yb  : final y position of trajectory
     *      zb  : final z position of trajectory
     *      v   : desired end effector velocity for path
     */
    xaLine=x;
    yaLine=y;
    zaLine=z;
    lineTrajGen(xaLine,yaLine,zaLine,xb,yb,zb,v);
}

void lineFollow(float t, float delay)
{
    /*
     * Calculates desired end effector position at current time to follow a line
     * Uses global variables calculated in lineTrajGen()
     * Leaves desired position at the previous value if the trajectory has been completed
     * Input args:
     *      t       : current time [s]
     *      delay   : delay unit state is incremented after trajectory is finished
     */
    if(t-ts<=tTot) {
        xd = dxStraight * (t - ts) / tTot + xaLine;
        yd = dyStraight * (t - ts) / tTot + yaLine;
        zd = dzStraight * (t - ts) / tTot + zaLine;
    }
    else if(t-ts>=tTot+delay && state%2!=0)
        state++;
}

void resetGains()
{
    /*
     * Resets task space PD gains to original values
     */
    kpx=300;
    kpy=300;
    kpz=300;
    kdx=20;
    kdy=20;
    kdz=20;
}

void resetAngles()
{
    /*
     * Resets N frame orientation to match the world frame
     */
    thetax=0;
    thetay=0;
    thetaz=0;
}

// This function is called every 1 ms
void lab(float theta1motor,float theta2motor,float theta3motor,float *tau1,float *tau2,float *tau3, int error) {
    // save past states
    if ((mycount%50)==0) {

        theta1array[arrayindex] = theta1motor;

        if (arrayindex >= 99) {
            arrayindex = 0;
        } else {
            arrayindex++;
        }
    }

    if ((mycount%500)==0) {
        UARTprint = 1;
        GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1; // Blink LED on Control Card
        GpioDataRegs.GPBTOGGLE.bit.GPIO60 = 1; // Blink LED on Emergency Stop Box
    }

    // calculate N-frame trig terms, rotation matrix, and rotation matrix transpose
    rotTrig();
    zxyRot();

    // calculate motor angle trig terms and end effector jacobian transpose terms
    cosq1 = cos(theta1motor);
    sinq1 = sin(theta1motor);
    cosq2 = cos(theta2motor);
    sinq2 = sin(theta2motor);
    cosq3 = cos(theta3motor);
    sinq3 = sin(theta3motor);
    JT_11 = -0.254*sinq1*(cosq3 + sinq2);
    JT_12 = 0.254*cosq1*(cosq3 + sinq2);
    JT_13 = 0;
    JT_21 = 0.254*cosq1*(cosq2 - sinq3);
    JT_22 = 0.254*sinq1*(cosq2 - sinq3);
    JT_23 = -0.254*(cosq3 + sinq2);
    JT_31 = -0.254*cosq1*sinq3;
    JT_32 = -0.254*sinq1*sinq3;
    JT_33 = -0.254*cosq3;

    // forward kinematics to find end effector position
    forwardKin(theta1motor, theta2motor, theta3motor);

    // end effector velocity calculations
    eeVel();

    // joint velocity calculations
    jointVel(theta1motor, theta2motor, theta3motor);

    // friction compensation calculations
    frictionComp();

    // calculates and intermediate time variable for convenience
    t = mycount/1000.0;

    /*
     * Switches behavior of the robot between states.
     * In even states, the robot generates a new trajectory.
     * In odd states, the robot follows the given trajectory.
     * Note that even states typically last for a single increment.
     * For flexibility of states, the state increment is placed outside of goToLinGen.
     * Although each state is fairly independent in the current implementation, more interesting behaviors
     * may benefit from trajectories defined and modified in multiple states.
     */
    if(state==0) { // Arbitrary starting point
        goToLinGen(0.3, 0, 0.45, 0.4);
        state++;
    }
    else if(state==2 || state==6) { // Position above hole
        goToLinGen(0.045, 0.35, 0.25, 0.15);
        state++;
    }
    else if(state==4) { // TO FULL DEPTH
        goToLinGen(0.045, 0.35, 0.125, 0.15);
        state++;
    }
    else if(state==8){ // Waypoint to avoid obstacle
        goToLinGen(0.26, 0.02,0.39,0.3);
        state++;
    }
    else if(state==10){ //ENTRANCE to zig zag
        goToLinGen(0.39, 0.08,0.37, 0.1);
        state++;
    }
    else if(state==12){ // waypoint1 (zig zag)
        goToLinGen(0.39, 0.08,0.215, 0.1);
        state++;
    }
    else if(state==14){ // waypoint2 (zig zag)
        goToLinGen(0.41, 0.02,0.208,0.1);
        kpx=0;
        kdx=0;
        state++;
    }
    else if(state==16){ // waypoint3 (zig zag)
        goToLinGen(0.35,0.03,0.210,0.1);
        resetGains();
        state++;
    }
    else if(state==18){ // waypoint4 (zig zag)
        goToLinGen(0.32,0.02,0.210,0.1);
        state++;
        }
    else if(state==20){ // waypoint5 (zig zag)
        goToLinGen(0.32,0.02,0.210,0.1);
        state++;
        }
    else if(state==22){ // waypoint6 (zig zag)
        goToLinGen(0.33,0.0,0.210,0.1);
        state++;
        }
    else if(state==24){ // out of zig zag
        goToLinGen(0.36, -0.11,0.210,0.1);
        resetGains();
        resetAngles();
        kpx=0;
        kdx=0;
        state++;
    }
    else if(state==26){ //EXIT
        goToLinGen(0.36,-0.11,0.40,0.1);
        resetGains();
        state++;
    }
    else if(state==28 || state==32){ // Position above egg
        goToLinGen(0.24, 0.13,0.35-0.06,0.15);
        resetGains();
        feedfwd=0;
        state++;
    }
    else if(state==30){ // EGG HEIGHT
        goToLinGen(0.24, 0.13,eggH,0.15);
        fzcmd=-0.05*9.81;
        feedfwd=1;
        state++;
    }
    else if(state==34){ // End pose
        goToLinGen(0.254,0,0.508,0.15);
        state++;
    }
    else if(state==5 || state==31){
        // creates a 2 s delay while in the hole and on the egg
        lineFollow(t,2.0);
        }
    else if(state>=10 && state<26){
        // delay of 0.25 s between waypoints in the zig zag
        lineFollow(t,0.25);
    }
    else
        // delay of 1.0 s added to allow for dwells at each end of the linear motion
        lineFollow(t, 1.0);


    if(feedfwd==0)
        // applies impedance control given the desired end effector position
        impedanceControl();
    else
        // applies feed forward control to apply a force in the z-direction while applying a force in the z-direction
        feedforwardForce();

    //
    *tau1=t1;
    *tau2=t2;
    *tau3=t3;
    //feedfwd=0;

    // applies torque saturation by setting torque magnitude to 5 if magnitude is greater than 5
    if(fabs(*tau1)>5)
        *tau1=5.0*(*tau1)/fabs(*tau1);
    if(fabs(*tau2)>5)
        *tau2=5.0*(*tau2)/fabs(*tau2);
    if(fabs(*tau3)>5)
        *tau3=5.0*(*tau3)/fabs(*tau3);

    // sets values to be plotted in Simulink
    Simulink_PlotVar1 = theta1motor;
    Simulink_PlotVar2 = theta2motor;
    Simulink_PlotVar3 = theta3motor;
    Simulink_PlotVar4 = xd;

    mycount++;
}

void printing(void){
    /*
     * prints values to TeraTerm
     */
    if (whattoprint == 0) {
        serial_printf(&SerialA, "World Frame Position: %.2f, %.2f,%.2f,%.2i   \n\r",x,y,z,state);
        serial_printf(&SerialA,"\n\n");
    } else {
        serial_printf(&SerialA, "Print test   \n\r");
    }

}
