# GPU related module
from pycuda.compiler import SourceModule

# C kernel written for gpu computation
kernel_func = SourceModule("""
#include <iostream>
#include <vector>
#include <math.h>
#include <cmath>

using namespace std;

float DIST_THRESH = 1.5;

vector<float> cart2spher(vector<float> cart_pos){
	/*
		Convert cartesian coordinate to spherical coordinate
		-----------------------------------------------------
		Input:  3 size float vector, representint (x, y, z)
		-----------------------------------------------------
		Output: 2 size float vector, representint (theta, phi) corresponding to (pitch, yaw)
	*/
	float x = cart_pos[0], y = cart_pos[1], z = cart_pos[2];
	float radius = sqrt( pow(x,2) + pow(y,2) + pow(z,2) );
	float theta = acos( z/radius );
	float phi = 0;
	// M_PI/2 : some problem occur because the coordinate of camera world and gazebo world
	if ( y >= 0 ){
		phi = acos( x/sqrt( pow(x,2)+pow(y,2) )) + M_PI/2;	
	}else{
		phi = acos( x/sqrt( pow(x,2)+pow(y,2) )) + M_PI*3/2;
	}
	vector<float> spher_pos{theta, phi};
	return spher_pos;
}

vector<float> spher2cart(vector<float> spher_pos){
	/*
		Convert spherical coordinate to cartesian coordinate
		-----------------------------------------------------
		Input:  2 size float vector, represent a (theta, phi) corresponding to (pitch, yaw)
		-----------------------------------------------------
		Output: 3 size float vector, represent a (x, y, z)
	*/
    float theta = spher_pos[0], phi = spher_pos[1];
    float z = cos(theta);
    float y = sin(theta) * sin(phi - M_PI/2);
    float x = sin(theta) * cos(phi - M_PI/2);
    vector<float> cart_pos{x, y, z};
    return cart_pos;
}

vector<float> spher2cart(float theta, float phi){
	vector<float> a = {theta, phi};
	return spher2cart(a);
}

float prob(vector<float> pos){
	/*
		Calculate the probability of drone occuring at certain position pos. With fixed droneArg
		droneArg : [radius, theta(yaw), alpha(pitch)]
		Corresponding to the function "phi" in the Optimization.py line 63 
		-----------------------------------------------------
		Input:	pos is a 3 size vector representing (x, y, z)
		-----------------------------------------------------
		Output: probability of drone occuring at pos
	*/
	vector< vector<float> > droneList ={ {20,	0.23,	 0.34,	 1.0},
										 {20.3,	 1.5,	 0.34,	 1.0},
										 {20.9,	 0,		 0,		 1.0}};
	float probability = 0;
	for (auto drone = droneList.begin(); drone!=droneList.end(); ++drone){
		// Convert spher_coordinate to cartesian coordinate
		float radius = (*drone)[0];
		float theta = (*drone)[1];
		float phi = (*drone)[2];
		vector<float> cart_pos = spher2cart(theta, phi); 
		for (int i=0;i<3;++i){cart_pos[i] *= radius;}

		// Calculate distance
		float dist = 0;
		for (int i=0;i<3;++i){dist += pow((pos[i]-cart_pos[i]), 2);}
		dist = sqrt(dist);

		// Calculate probability according to distance
		if (dist < DIST_THRESH){ probability += (*drone)[3] * exp(-dist); }
		else {continue;}
	}
	return probability;
}

"""
)
