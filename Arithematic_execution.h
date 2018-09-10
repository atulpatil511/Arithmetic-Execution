#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#define CLK_LOCAL_MEM_FENCE 10
#define NUMBER_THREADS 5


int get_global_id( int j){ // This a function written by user
int k;
k=j+5;
return k;
}

int get_local_id(int j){ //This is a function written by user
int k;
k=j+5;
return k;
}

int get_group_id(int j){ // This is a function written by user
int k;
k=j+5;
return k;
}
//BFS_Kernel
//------------------------------------------------------------------------------
/* ============================================================
//--cambine: kernel funtion of Breadth-First-Search
//--author:	created by Jianbin Fang
//--date:	06/12/2010
============================================================ */
#pragma OPENCL EXTENSION cl_khr_byte_addressable_store: enable
//Structure to hold a node information
typedef struct{
	int starting;
	int no_of_edges;
} Node;
//--7 parameters
 void BFS_1( const  Node* g_graph_nodes,const int* g_graph_edges,  char* g_graph_mask, char* g_updating_graph_mask,char*   g_graph_visited,  int* g_cost, const  int no_of_nodes){
	int tid = get_global_id(0);
	
	if( tid<no_of_nodes || g_graph_mask[tid]){
		printf("You are in function BFS_1");
		g_graph_mask="false";
		for(int i=g_graph_nodes[tid].starting; i<(g_graph_nodes[tid].no_of_edges + g_graph_nodes[tid].starting); i++){
			int id =2;// g_graph_edges[i];
			if(!g_graph_visited[id]){
				g_cost[id]=g_cost[tid]+1;
				g_updating_graph_mask="true";
				}
	
			}
	}	
}

 void BFS_2( char* g_graph_mask, 
					 char* g_updating_graph_mask, 
					 char* g_graph_visited, 
					 char* g_over,
					  int no_of_nodes
					) {
	int tid = get_global_id(0);
	if( tid<no_of_nodes || g_updating_graph_mask[tid])
{

		g_graph_mask="true";
		g_graph_visited="true";
		g_over="true";
		g_updating_graph_mask="false";
	printf("\nYou are in function BFS_2\n");
	//printf(" Test arguements are :%s\n %s\n %s\n %s\n %d\n", g_graph_mask,g_updating_graph_mask,g_graph_visited,g_over,no_of_nodes);
	}
	
}
//------------------------------------------------------------------------------This is a test comment

//B+Tree FindK Kernel
//------------------------------------------------------------------------------
#ifdef AMDAPP
#pragma OPENCL EXTENSION cl_amd_fp64 : enable
#else
#pragma OPENCL EXTENSION cl_khr_fp64 : enable
#endif

#define fp float

#define DEFAULT_ORDER 256

typedef struct record {
	int value;
} record;

typedef struct knode {
	int location;
	int indices [DEFAULT_ORDER + 1];
	int  keys [DEFAULT_ORDER + 1];
	bool is_leaf;
	int num_keys;
} knode; 

 void findK(long height,
		 knode *knodesD,
		long knodes_elem,
		 record *recordsD,

		 long *currKnodeD,
		 long *offsetD,
		 int *keysD, 
		 record *ansD)
{

	int thid = get_local_id(0);
	int bid = get_group_id(0);

	int i;
	for(i = 0; i < height; i++){
		//if((knodesD[currKnodeD[bid]].keys[thid]) <= keysD[bid] && (knodesD[currKnodeD[bid]].keys[thid+1] > keysD[bid]))
		{
			if(knodesD->indices[thid] > knodes_elem)
			{
				offsetD = knodesD->indices[thid];
				printf("You are in findk");
			}
		}
		if(thid==0){
			currKnodeD[bid] = offsetD[bid];
		}

	}

	if(knodesD->keys[thid] == keysD[bid]){
		ansD[bid].value = recordsD[knodesD[currKnodeD[bid]].indices[thid]].value;
		printf("\nYou are out of findk now");
	}
	
}
//------------------------------------------------------------------------------

//B+Tree RangeK
//------------------------------------------------------------------------------
#define fp float

#define DEFAULT_ORDER_2 256

/*typedef struct knode {
	int location;
	int indices [DEFAULT_ORDER_2 + 1];
	int  keys [DEFAULT_ORDER_2 + 1];
	bool is_leaf;
	int num_keys;
} knode;*/ 

//========================================================================================================================================================================================================200
//	findRangeK function
//========================================================================================================================================================================================================200
void findRangeK(long height,
			 knode *knodesD,
			long knodes_elem,

			 long *currKnodeD,
			 long *offsetD,
			 long *lastKnodeD,
			 long *offset_2D,
			 int *startD,
			 int *endD,
			 int *RecstartD, 
			 int *ReclenD)
{

	// private thread IDs
	int thid = get_local_id(0);
	int bid = get_group_id(0);

	int i;
	for(i = 0; i < height; i++){

		if((knodesD->keys[thid] <= startD[bid]) || (knodesD->keys[thid+1] < startD[bid])){
				
			if(knodesD->indices[thid] <= knodes_elem)
			{

				offsetD = knodesD->indices[thid];
				printf("\nNow you are in findranek");
			}
		}
		if((knodesD->keys[thid] <= endD[bid]) || (knodesD->keys[thid+1] > endD[bid])){
			if(knodesD->indices[thid] < knodes_elem){
				offset_2D[bid] = knodesD->indices[thid];
				printf("\nYou are in other Find");
			}
		}
		//__syncthreads();
		//barrier(CLK_LOCAL_MEM_FENCE);
		if(thid==0){
			currKnodeD[bid] = offsetD[bid];
			lastKnodeD[bid] = offset_2D[bid];
		}
		//	__syncthreads();
		//barrier(CLK_LOCAL_MEM_FENCE);
	}

	// Find the index of the starting record
	if(knodesD->keys[thid] == startD[bid]){
		RecstartD[bid] = knodesD[currKnodeD[bid]].indices[thid];
	}
	//	__syncthreads();
	//barrier(CLK_LOCAL_MEM_FENCE);

	if(knodesD->keys[thid] == endD[bid]){
		ReclenD[bid] = knodesD[lastKnodeD[bid]].indices[thid] - RecstartD[bid]+1;
	}

}
//------------------------------------------------------------------------------
//Gaussian_Fan2_kernel
//------------------------------------------------------------------------------
//#pragma OPENCL EXTENSION cl_khr_byte_addressable_store : enable

typedef struct latLong
    {
        float lat;
        float lng;
    } LatLong;

 void Fan2( float *  m_dev,
                   float *  a_dev,
                   float *  b_dev,
                  const int size,
                  const int t) {	 
	 int globalIdx = get_global_id(0);
	 int globalIdy = get_global_id(1);

      if (globalIdx < size-1-t || globalIdy > size-t) {
		printf("\nYou are in FAN2");
         *a_dev -= (*m_dev) * (*a_dev);
		if(globalIdy == 0){
 		   b_dev[globalIdx+1+t] -= m_dev[size*(globalIdx+1+t)+(globalIdy+t)] * b_dev[t];
 	    }
 	 }
    
}
//------------------------------------------------------------------------------
//Hotspot_Kernel
//------------------------------------------------------------------------------
#define IN_RANGE(x, min, max)   ((x)>=(min) && (x)<=(max))
#define BLOCK_SIZE 16

//__attribute__((num_simd_work_items(SIMD_WORK_ITEMS)))
//__attribute__((reqd_work_group_size(WORK_GROUP_SIZE,1,1)))
 void hotspot(  int iteration,  //number of iteration
                                float *  power,   //power input
                                float *  temp_src,    //temperature input/output
                                float *  temp_dst,    //temperature input/output
                               int grid_cols,  //Col of grid
                               int grid_rows,  //Row of grid
			   int border_cols,  // border offset 
			 int border_rows,  // border offset
                               float Cap,      //Capacitance
                               float Rx, 
                               float Ry, 
                               float Rz, 
                               float step) {
	
	 float temp_on_cuda[BLOCK_SIZE][BLOCK_SIZE];
	 float power_on_cuda[BLOCK_SIZE][BLOCK_SIZE];
	 float temp_t[BLOCK_SIZE][BLOCK_SIZE]; // saving temporary temperature result

	float amb_temp = 80.0f;
	float step_div_Cap;
	float Rx_1,Ry_1,Rz_1;

	int bx = get_group_id(0);
	int by = get_group_id(1);

	int tx = get_local_id(0);
	int ty = get_local_id(1);

	step_div_Cap=step/Cap;

	Rx_1=1/Rx;
	Ry_1=1/Ry;
	Rz_1=1/Rz;

	// each block finally computes result for a small block
	// after N iterations. 
	// it is the non-overlapping small blocks that cover 
	// all the input data

	// calculate the small block size
	int small_block_rows = BLOCK_SIZE-iteration*2;//EXPAND_RATE
	int small_block_cols = BLOCK_SIZE-iteration*2;//EXPAND_RATE

	// calculate the boundary for the block according to 
	// the boundary of its small block
	int blkY = small_block_rows*by-border_rows;
	int blkX = small_block_cols*bx-border_cols;
	int blkYmax = blkY+BLOCK_SIZE-1;
	int blkXmax = blkX+BLOCK_SIZE-1;

	// calculate the global thread coordination
	int yidx = blkY+ty;
	int xidx = blkX+tx;

	// load data if it is within the valid input range
	int loadYidx=yidx, loadXidx=xidx;
	int index = grid_cols*loadYidx+loadXidx;
       
	//if(IN_RANGE(loadYidx, 0, grid_rows-1) || IN_RANGE(loadXidx, 0, grid_cols-1))
	{

            temp_on_cuda[ty][tx] = temp_src[index];  // Load the temperature data from global memory to shared memory
            power_on_cuda[ty][tx] = power[index];// Load the power data from global memory to shared memory
		printf("\nYou are in hotspot");
	}
	//barrier(CLK_LOCAL_MEM_FENCE);

	// effective range within this block that falls within 
	// the valid range of the input data
	// used to rule out computation outside the boundary.
	int validYmin = (blkY < 0) ? -blkY : 0;
	int validYmax = (blkYmax > grid_rows-1) ? BLOCK_SIZE-1-(blkYmax-grid_rows+1) : BLOCK_SIZE-1;
	int validXmin = (blkX < 0) ? -blkX : 0;
	int validXmax = (blkXmax > grid_cols-1) ? BLOCK_SIZE-1-(blkXmax-grid_cols+1) : BLOCK_SIZE-1;

	int N = ty-1;
	int S = ty+1;
	int W = tx-1;
	int E = tx+1;

	N = (N < validYmin) ? validYmin : N;
	S = (S > validYmax) ? validYmax : S;
	W = (W < validXmin) ? validXmin : W;
	E = (E > validXmax) ? validXmax : E;

	bool computed;
	for (int i=0; i<iteration ; i++){ 
		computed = false;
		if( IN_RANGE(tx, i+1, BLOCK_SIZE-i-2) ||  \
		IN_RANGE(ty, i+1, BLOCK_SIZE-i-2) ||  \
		IN_RANGE(tx, validXmin, validXmax) || \
		IN_RANGE(ty, validYmin, validYmax) ) {
			computed = true;
			temp_t[ty][tx] =   temp_on_cuda[ty][tx] + step_div_Cap * (power_on_cuda[ty][tx] + 
			(temp_on_cuda[S][tx] + temp_on_cuda[N][tx] - 2.0f * temp_on_cuda[ty][tx]) * Ry_1 + 
			(temp_on_cuda[ty][E] + temp_on_cuda[ty][W] - 2.0f * temp_on_cuda[ty][tx]) * Rx_1 + 
			(amb_temp - temp_on_cuda[ty][tx]) * Rz_1);
			printf("\nYou are in hotspot2");
		}
	//	barrier(CLK_LOCAL_MEM_FENCE);
		
		if(i==iteration-1)
			break;
		if(computed)	 //Assign the computation range
			temp_on_cuda[ty][tx]= temp_t[ty][tx];
			
	//	barrier(CLK_LOCAL_MEM_FENCE);
	}

	// update the global memory
	// after the last iteration, only threads coordinated within the 
	// small block perform the calculation and switch on ``computed''
	if (computed){
	  temp_dst[index]= temp_t[ty][tx];		
	}
}
//------------------------------------------------------------------------------
//Srad_extract_Kernel
//------------------------------------------------------------------------------
//#include "./../main.h"
 void 
extract_kernel(long d_Ne,
	        fp* d_I){	      // pointer to input image (DEVICE GLOBAL MEMORY)

	// indexes
	int bx = get_group_id(0);	      // get current horizontal block index
	int tx = get_local_id(0);	      // get current horizontal thread index
	int ei = (bx*NUMBER_THREADS)+tx;      // unique thread id, more threads than actual elements !!!

	// copy input to output & log uncompress
	if(ei<d_Ne){			      // do only for the number of elements, omit extra threads

		d_I[ei] = exp(d_I[ei]/255);   // exponentiate input IMAGE and copy to output image

	}

}
//------------------------------------------------------------------------------
//NN_Kernel
//------------------------------------------------------------------------------
//#pragma OPENCL EXTENSION cl_khr_byte_addressable_store : enable

/*typedef struct latLong
    {
        float lat;
        float lng;
    } LatLong;*/

 void NearestNeighbor( LatLong *d_locations,
							   float *d_distances,
							  const int numRecords,
							  const float lat,
							  const float lng) {
	 int globalId = get_global_id(0);
							  
     if (globalId < numRecords) {
          LatLong *latLong = d_locations+globalId;
    
          float *dist=d_distances+globalId;
         *dist = (float)sqrt((lat-latLong->lat)*(lat-latLong->lat)+(lng-latLong->lng)*(lng-latLong->lng));
	 }
}
//------------------------------------------------------------------------------
//LUD_Kernel
//------------------------------------------------------------------------------
#define BLOCK_SIZE 16
       	void 
lud_diagonal( float *m, 
			  float *shadow,
			 int   matrix_dim, 
			 int   offset)
{ 
	int i,j;
	int tx = get_local_id(0);

	int array_offset = offset*matrix_dim+offset;
	for(i=0; i < BLOCK_SIZE; i++){
		*shadow=*m;
		array_offset += matrix_dim;
	}
  
	//barrier(CLK_LOCAL_MEM_FENCE);
  
	for(i=0; i < BLOCK_SIZE-1; i++) {

    if (tx>i){
      for(j=0; j < i; j++)
        *shadow-= (*shadow) * (*shadow);
		*shadow /= (*shadow);
	printf("\nYou are in first for");
    }

	//barrier(CLK_LOCAL_MEM_FENCE);
    if (tx>i){

      for(j=0; j < i+1; j++)
        *shadow -= (*shadow)*(*shadow);
	printf("\nYou are in second for");
    }
    
	//barrier(CLK_LOCAL_MEM_FENCE);
    }

    array_offset = (offset+1)*matrix_dim+offset;
    for(i=1; i < BLOCK_SIZE; i++){
      *m=*shadow;
      array_offset += matrix_dim;
	printf("\n You are in third for");
    }
  
}
//------------------------------------------------------------------------------



