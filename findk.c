#include<stdio.h>
#include"Arithematic_execution.h"

void main()
{	knode test1;
	record test2,test3;
	int val1;

	long dummy1,dummy2;

	test1.location=5;
	test1.indices[0]=2;
	test1.keys[0]=3;
	test1.num_keys=2;
	test2.value=4;
	test3.value=5;
	val1=2;
	dummy1=10;
	dummy2=20;
findK(100,&test1,4,&test2,&dummy1,&dummy2,&val1, &test3);
}
