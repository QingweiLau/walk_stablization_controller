#ifndef _GRAPH_H
#define _GRAPH_H
#include <stdio.h>
#include <stdlib.h>
#include <vector>




using namespace std;

class plot {
public:
 plot() { }

 void plot_data(FILE* gp, vector<double> x,vector<double> y) {
  for(int k=0;k<x.size();k++) {
    fprintf(gp,"plot '-' w %s \n","points");
    //fprintf(gp,"plot '-' w %s \n","line");
    fprintf(gp,"%f %f \n",x[k],y[k]);
  }
  fprintf(gp,"e\n");
  fflush(gp);
 }
 
};

#endif
