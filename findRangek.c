#include<stdio.h>
#include"Arithematic_execution.h"

void main()
{
Node test;
	knode test1;
	
	int val1, val2, val3,val4;
	long dummy1,dummy2,dummy3,dummy4;
	

	dummy1=10;
	dummy2=20;
	dummy3=30;
	dummy4=40;
	
	val1=2;
	val2=3;
	val3=7;
	val4=8;


	test1.location=5;
	test1.indices[0]=2;
	test1.keys[0]=3;
	test1.num_keys=2;
	
findRangeK(100,&test1,100,&dummy1,&dummy2,&dummy3,&dummy4,&val1,&val2,&val3,&val4);
}
