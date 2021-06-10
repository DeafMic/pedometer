#include <stdio.h>
#include <phidget21.h>
#include <armadillo>

#ifndef PHIDGET
#define PHIDGET

using namespace arma;

//Porcata
mat imudata;

//callback that will run if the Spatial is attached to the computer
int CCONV AttachHandler(CPhidgetHandle spatial, void *userptr);

//callback that will run if the Spatial is detached from the computer
int CCONV DetachHandler(CPhidgetHandle spatial, void *userptr);
//callback that will run if the Spatial generates an error
int CCONV ErrorHandler(CPhidgetHandle spatial, void *userptr, int ErrorCode, const char *unknown);

//callback that will run at datarate
//data - array of spatial event data structures that holds the spatial data packets that were sent in this event
//count - the number of spatial data event packets included in this event
int CCONV SpatialDataHandler(CPhidgetSpatialHandle spatial, void *userptr, CPhidgetSpatial_SpatialEventDataHandle *data, int count);

//Display the properties of the attached phidget to the screen.  
//We will be displaying the name, serial number, version of the attached device, the number of accelerometer, gyro, and compass Axes, and the current data rate
// of the attached Spatial.
int display_properties(CPhidgetHandle phid);

int spatial_simple();

#endif
