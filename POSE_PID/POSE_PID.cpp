// POSE_PID.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include "NavHelper.h"
#include "PID.h"
#include "Quaternion.h"

int main()
{
	typedef float MyType;
	static const unsigned int AXISCOUNT = 3;
	static const MyType DELTATIME = 0.001; // must not be zero
	typedef Quaternion<MyType> MyQuaternion;
	typedef NavHelper<MyType, AXISCOUNT>  MyNavHelper;
	typedef PID<MyType, AXISCOUNT> MyPID;

	//some sample PID gains needed by the code
	static const MyType PGAIN[AXISCOUNT] = { 0.7, 0.7, 0.7 };
	static const MyType IGAIN[AXISCOUNT] = {0.02, 0.02, 0.02};
	static const MyType DGAIN[AXISCOUNT] = { 0.05, 0.05, 0.05 };

	MyQuaternion currentOrientationQuat;
	MyQuaternion prevOrientationQuat;
	MyQuaternion currentTargetOrientationQuat;
	MyQuaternion prevTargetOrientationQuat;
	MyQuaternion QTransform;
	MyQuaternion* ConjugateQTransform = nullptr;
	MyNavHelper  navHelper;
	MyNavHelper::Position currentPositionMobileFrame;
	MyNavHelper::Position prevPositionMobileFrame;
	MyNavHelper::Position currentPositionTargetFrame;
	MyNavHelper::Position prevPositionTargetFrame;
	MyType PTransform[AXISCOUNT][AXISCOUNT];
	MyPID PIDController(&PGAIN[0], &IGAIN[0], &DGAIN[0], DELTATIME);
	MyType velocities[AXISCOUNT];
	MyType command[AXISCOUNT];

   //Here we would obtain current PTransform, and QTransform
   // bool run = GetRunState();
   //while(run) // production loop
	{
		// In this part of the code we would get the current values of QTransform, currentOrientationQuat and CurrentPosition 
		//
		// Next, we convert the present and previous mobile frame data to target frame.

		//position translation first
		navHelper.Transformer(&PTransform[0][0], AXISCOUNT, &currentPositionMobileFrame, &currentPositionTargetFrame);
		navHelper.Transformer(&PTransform[0][0], AXISCOUNT, &prevPositionMobileFrame, &prevPositionTargetFrame);

		// Next translate the quaternions
		ConjugateQTransform = navHelper.Conjucate(&QTransform);
		//using the overloaded multiply (*) we compute the translated quaternions
		MyQuaternion temp = (QTransform * currentOrientationQuat);
		currentTargetOrientationQuat = (temp * (*ConjugateQTransform));

		temp = (QTransform * prevOrientationQuat);
		prevTargetOrientationQuat = (temp * (*ConjugateQTransform));

		//compute the velocities
		for (int iI = 0; iI < AXISCOUNT; ++iI)
		{
			 MyType temp = (currentPositionTargetFrame.Get()[iI] - prevPositionTargetFrame.Get()[iI]);
			 std::cout << "Target Frame position error, axis # " << iI << " = " << temp << std::endl;
			 velocities[iI] = temp / DELTATIME;
		}

		//we would get the command value here
		// GetUpdatedCommand(&command[0]);
		//Run the PID Controller on this position based velocity signal
		MyPID.Run(&velocities[0], &command[0]);
	}
}
