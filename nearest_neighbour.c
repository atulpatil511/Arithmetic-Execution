#include<stdio.h>
#include"Arithematic_execution.h"


void main()
{	
	LatLong test;
	test.lat=5.0;
	test.lng=7.0;
	float d_distance_= 9.0;
	int numrecords=8;
	 NearestNeighbor( &test, &d_distance_, numrecords,7.2,3.4);

}
