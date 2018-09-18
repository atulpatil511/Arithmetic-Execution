all:
	cc -Wall -g BFS_1.c -o BFS_1 -lm
	cc -Wall -g BFS_2.c -o BFS_2 -lm
	cc -Wall -g hotspot.c -o hotspot -lm
	cc -Wall -g Fan2.c -o Fan2 -lm
	cc -Wall -g findk.c -o findk -lm
	cc -Wall -g findRangek.c -o findRangek -lm
	cc -Wall -g lud_diagonal.c -o lud_diagonal -lm
	cc -Wall -g nearest_neighbour.c -o nearest_neighbour -lm
	cc -Wall -g extract_kernel.c -o extract_kernel -lm


