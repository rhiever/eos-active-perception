/*
 * tGame.cpp
 *
 * This file is part of the aBeeDa Swarm Evolution project.
 *
 * Copyright 2012 Randal S. Olson, Arend Hintze.
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "tGame.h"
#include <math.h>
#include <float.h>
#include <stdlib.h>
#include <stdio.h>

#define cPI 3.14159265

// simulation-specific constants
#define     pathfinderVisionRange           50.0 * 50.0
#define     pathfinderSize                  5.0 * 5.0
#define     smallFoodSize                   5.0 * 5.0
#define     largeFoodSize                   20.0 * 20.0
#define     totalStepsInSimulation          2000
#define     gridX                           400.0
#define     gridY                           400.0
#define     gridXAcross                     2.0 * gridX
#define     gridYAcross                     2.0 * gridY
#define     boundaryDist                    400.0
#define     numFood                         100

// precalculated lookup tables for the game
double cosLookup[360];
double sinLookup[360];

tGame::tGame()
{
    // fill lookup tables
    for (int i = 0; i < 360; ++i)
    {
        cosLookup[i] = cos((double)i * (cPI / 180.0));
        sinLookup[i] = sin((double)i * (cPI / 180.0));
    }
}

tGame::~tGame() { }

// runs the simulation for the given agent(s)
string tGame::executeGame(tAgent* pathfinderAgent, FILE *data_file, bool report)
{
    string reportString = "";
    double pathfinderX = 0.0, pathfinderY = 0.0, pathfinderAngle = 0.0, pathfinderFitness = 0.0;
    double foodX[numFood], foodY[numFood], foodSize[numFood];
    bool foodEaten[numFood];
    
    pathfinderAgent->fitness = 1.0;
    pathfinderAgent->setupPhenotype();
    
    pathfinderX = 0.0;//0.5 * ((2.0 * randDouble * gridX) - gridX);
    pathfinderY = 0.0;//0.5 * ((2.0 * randDouble * gridY) - gridY);
    pathfinderAngle = (int)(randDouble * 360.0);
    
    for (int i = 0; i < numFood; ++i)
    {
        bool goodPos = true;
        
        if (i > numFood / 4)
        {
            foodSize[i] = smallFoodSize;
        }
        else
        {
            foodSize[i] = largeFoodSize;
        }
        
        do
        {
            goodPos = true;
            foodX[i] = 0.95 * ((2.0 * randDouble * gridX) - gridX);
            foodY[i] = 0.95 * ((2.0 * randDouble * gridY) - gridY);
            
            for (int j = 0; j < i; ++j)
            {
                if (calcDistanceSquared(foodX[i], foodY[i], foodX[j], foodY[j]) - foodSize[i] / 2.0 - foodSize[j] / 2.0 <= pathfinderVisionRange)
                {
                    goodPos = false;
                    break;
                }
            }
        } while (!goodPos);
        
        foodEaten[i] = false;
    }
    
    /*       BEGINNING OF SIMULATION LOOP       */
    
    for(int step = 0; step < totalStepsInSimulation; ++step)
    {
        
        /*       CREATE THE REPORT STRING FOR THE VIDEO       */
        if(report)
        {
            // report X, Y, angle of pathfinding agent
            char text1[1000];
            sprintf(text1, "%f,%f,%f,%f,%d,%d,%d=", pathfinderX, pathfinderY, pathfinderAngle, sqrt(pathfinderSize), 0, 0, 0);
            reportString.append(text1);
            
            // report X, Y of food
            for (int i = 0; i < numFood; ++i)
            {
                if (!foodEaten[i])
                {
                    char text2[1000];
                    sprintf(text2, "%f,%f,%f,%f,%d,%d,%d=", foodX[i], foodY[i], 0.0, sqrt(foodSize[i]), 0, 200, 0);
                    reportString.append(text2);
                }
            }
            
            reportString.append("N");
            
        }
        /*       END OF REPORT STRING CREATION       */
        
        
        /*       SAVE DATA FOR THE LOD FILE       */
        if(data_file != NULL)
        {
            
        }
        /*       END OF DATA GATHERING       */
        
        
        /*       UPDATE PATHFINDER       */
        
        
        // clear the pathfinder sensors
        for (int i = 0; i < 181; ++i)
        {
            pathfinderAgent->states[i] = 0;
        }
        
        // update the pathfinder sensors
        for (int i = 0; i < numFood; ++i)
        {
            if (!foodEaten[i])
            {
                double distToFood = calcDistanceSquared(pathfinderX, pathfinderY, foodX[i], foodY[i]) - pathfinderSize / 2.0 - foodSize[i] / 2.0;
                
                if (distToFood <= pathfinderVisionRange)
                {
                    /*pathfinderX = pathfinderY = 0;
                    pathfinderAngle = 90;
                    foodX[i] = 0;
                    foodY[i] = sqrt(largeFoodSize);
                    foodSize[i] = largeFoodSize;*/
                    
                    double foodRadius = sqrt(foodSize[i]) / 2;
                    
                    int angleTL = (int)calcAngle(pathfinderX, pathfinderY, pathfinderAngle, foodX[i] - foodRadius, foodY[i] + foodRadius);
                    int angleTR = (int)calcAngle(pathfinderX, pathfinderY, pathfinderAngle, foodX[i] + foodRadius, foodY[i] + foodRadius);
                    int angleBL = (int)calcAngle(pathfinderX, pathfinderY, pathfinderAngle, foodX[i] - foodRadius, foodY[i] - foodRadius);
                    int angleBR = (int)calcAngle(pathfinderX, pathfinderY, pathfinderAngle, foodX[i] + foodRadius, foodY[i] - foodRadius);
                    
                    int activateFromAngle = min(angleTL, min(angleTR, min(angleBL, angleBR)));
                    int activateToAngle = max(angleTL, max(angleTR, max(angleBL, angleBR)));
                    
                    // keep angles within -90 and 90
                    activateFromAngle = fmax(-90, activateFromAngle);
                    activateToAngle = fmin(90, activateToAngle);
                    
                    /*cout << activateFromAngle << " " << activateToAngle << endl;
                    exit(0);*/
                    
                    for (int j = activateFromAngle; j <= activateToAngle; ++j)
                    {
                        pathfinderAgent->states[j + 90] = 1;
                    }
                    
                    /*if (distToFood < largeFoodSize)
                    {
                        for (int j = activateFromAngle; j <= activateToAngle; ++j)
                        {
                            pathfinderAgent->states[181 + j + 90] = 1;
                        }
                    }*/
                }
            }
        }
        
        // activate the pathfinder's brain
        pathfinderAgent->updateStates();
        
        int action = ((pathfinderAgent->states[(maxNodes - 1)] & 1) << 1) + (pathfinderAgent->states[(maxNodes - 2)] & 1);
        
        switch(action)
        {
            // don't move
            case 0:
                break;
                
            // turn right
            case 1:
                pathfinderAngle += 6.0;
                
                while(pathfinderAngle >= 360.0)
                {
                    pathfinderAngle -= 360.0;
                }
                
                pathfinderX += cosLookup[(int)pathfinderAngle] * 2.0;
                pathfinderY += sinLookup[(int)pathfinderAngle] * 2.0;
                
                break;
                
            // turn left
            case 2:
                pathfinderAngle -= 6.0;
                
                while(pathfinderAngle < 0.0)
                {
                    pathfinderAngle += 360.0;
                }
                
                pathfinderX += cosLookup[(int)pathfinderAngle] * 2.0;
                pathfinderY += sinLookup[(int)pathfinderAngle] * 2.0;
                
                break;
                
            // move straight ahead
            case 3:
                pathfinderX += cosLookup[(int)pathfinderAngle] * 2.0;
                pathfinderY += sinLookup[(int)pathfinderAngle] * 2.0;
                
                break;
                
            default:
                break;
        }
        
        // keep position within simulation boundary
        applyBoundary(pathfinderX);
        applyBoundary(pathfinderY);
        
        
        // determine if pathfinder collided with food
        for (int i = 0; i < numFood; ++i)
        {
            if (!foodEaten[i])
            {
                double distToFood = calcDistanceSquared(pathfinderX, pathfinderY, foodX[i], foodY[i]) - pathfinderSize / 2.0 - foodSize[i] / 2.0;
                
                if (distToFood <= 0.0)
                {
                    int relativeAngle = (int)calcAngle(pathfinderX, pathfinderY, pathfinderAngle, foodX[i], foodY[i]);
                    
                    if (fabs(relativeAngle) <= 90)
                    {
                        foodEaten[i] = true;
                        
                        if (foodSize[i] == smallFoodSize)
                        {
                            pathfinderFitness += 1;
                        }
                        else
                        {
                            pathfinderFitness -= 1;
                        }
                        
                        break;
                    }
                }
            }
        }
        
        /*       END OF PATHFINDER UPDATE       */
        
    }
    /*       END OF SIMULATION LOOP       */
    
    // compute overall fitness
    pathfinderAgent->fitness = max(pathfinderFitness, 0.000001);
    
    // output to data file, if provided
    if (data_file != NULL)
    {
        fprintf(data_file, "%d,%f\n",
                pathfinderAgent->born,              // update born (prey)
                pathfinderAgent->fitness            // pathfinder fitness
                );
    }
    
    return reportString;
}


// wraps a position around a preset boundary (toroidal world)
void tGame::applyBoundary(double& positionVal)
{
    double val = positionVal;
    
    if (fabs(val) > boundaryDist)
    {
        if (val < 0)
        {
            val = boundaryDist;// - 10.0;
        }
        else
        {
            val = -boundaryDist;// + 10.0;
        }
    }
    
    positionVal = val;
}

/*// maintains a position within a preset boundary
 void tGame::applyBoundary(double& positionVal)
 {
 double val = positionVal;
 
 if (fabs(val) > boundaryDist)
 {
 if (val < 0)
 {
 val = -1.0 * boundaryDist;
 }
 else
 {
 val = boundaryDist;
 }
 }
 
 positionVal = val;
 }*/

// calculates the distance^2 between two points (toroidal world)
double tGame::calcDistanceSquared(double fromX, double fromY, double toX, double toY)
{
    double diffX = fabs(fromX - toX);
    double diffY = fabs(fromY - toY);
    
    if (diffX > gridX)
    {
        diffX = gridXAcross - diffX;
    }
    
    if (diffY > gridY)
    {
        diffY = gridYAcross - diffY;
    }
    
    return ( diffX * diffX ) + ( diffY * diffY );
}

/*// calculates the distance^2 between two points
 double tGame::calcDistanceSquared(double fromX, double fromY, double toX, double toY)
 {
 double diffX = fromX - toX;
 double diffY = fromY - toY;
 
 return ( diffX * diffX ) + ( diffY * diffY );
 }*/

// calculates the angle between two agents
double tGame::calcAngle(double fromX, double fromY, double fromAngle, double toX, double toY)
{
    double Ux = 0.0, Uy = 0.0, Vx = 0.0, Vy = 0.0;
    
    Ux = (toX - fromX);
    Uy = (toY - fromY);
    
    Vx = cosLookup[(int)fromAngle];
    Vy = sinLookup[(int)fromAngle];
    
    int firstTerm = (int)((Ux * Vy) - (Uy * Vx));
    int secondTerm = (int)((Ux * Vx) + (Uy * Vy));
    
    return atan2(firstTerm, secondTerm) * 180.0 / cPI;
    //return atan2Lookup[firstTerm + 400][secondTerm + 400];
}

// sums a vector of values
double tGame::sum(vector<double> values)
{
    double sum = 0.0;
    
    for (vector<double>::iterator i = values.begin(); i != values.end(); ++i)
    {
        sum += *i;
    }
    
    return sum;
}

// averages a vector of values
double tGame::average(vector<double> values)
{
    return sum(values) / (double)values.size();
}

// computes the variance of a vector of values
double tGame::variance(vector<double> values)
{
    double sumSqDist = 0.0;
    double mean = average(values);
    
    for (vector<double>::iterator i = values.begin(); i != values.end(); ++i)
    {
        sumSqDist += pow( *i - mean, 2.0 );
    }
    
    return sumSqDist /= (double)values.size();
}

double tGame::mutualInformation(vector<int> A,vector<int>B)
{
	set<int> nrA,nrB;
	set<int>::iterator aI,bI;
	map<int,map<int,double> > pXY;
	map<int,double> pX,pY;
	int i;
	double c=1.0/(double)A.size();
	double I=0.0;
	for(i=0;i<A.size();i++){
		nrA.insert(A[i]);
		nrB.insert(B[i]);
		pX[A[i]]=0.0;
		pY[B[i]]=0.0;
	}
	for(aI=nrA.begin();aI!=nrA.end();aI++)
		for(bI=nrB.begin();bI!=nrB.end();bI++){
			pXY[*aI][*bI]=0.0;
		}
	for(i=0;i<A.size();i++){
		pXY[A[i]][B[i]]+=c;
		pX[A[i]]+=c;
		pY[B[i]]+=c;
	}
	for(aI=nrA.begin();aI!=nrA.end();aI++)
		for(bI=nrB.begin();bI!=nrB.end();bI++)
			if((pX[*aI]!=0.0)&&(pY[*bI]!=0.0)&&(pXY[*aI][*bI]!=0.0))
				I+=pXY[*aI][*bI]*log2(pXY[*aI][*bI]/(pX[*aI]*pY[*bI]));
	return I;
	
}

double tGame::entropy(vector<int> list){
	map<int, double> p;
	map<int,double>::iterator pI;
	int i;
	double H=0.0;
	double c=1.0/(double)list.size();
	for(i=0;i<list.size();i++)
		p[list[i]]+=c;
	for (pI=p.begin();pI!=p.end();pI++) {
        H+=p[pI->first]*log2(p[pI->first]);	
	}
	return -1.0*H;
}

double tGame::ei(vector<int> A,vector<int> B,int theMask){
	set<int> nrA,nrB;
	set<int>::iterator aI,bI;
	map<int,map<int,double> > pXY;
	map<int,double> pX,pY;
	int i;
	double c=1.0/(double)A.size();
	double I=0.0;
	for(i=0;i<A.size();i++){
		nrA.insert(A[i]&theMask);
		nrB.insert(B[i]&theMask);
		pX[A[i]&theMask]=0.0;
		pY[B[i]&theMask]=0.0;
	}
	for(aI=nrA.begin();aI!=nrA.end();aI++)
		for(bI=nrB.begin();bI!=nrB.end();bI++){
			pXY[*aI][*bI]=0.0;
		}
	for(i=0;i<A.size();i++){
		pXY[A[i]&theMask][B[i]&theMask]+=c;
		pX[A[i]&theMask]+=c;
		pY[B[i]&theMask]+=c;
	}
	for(aI=nrA.begin();aI!=nrA.end();aI++)
		for(bI=nrB.begin();bI!=nrB.end();bI++)
			if((pX[*aI]!=0.0)&&(pY[*bI]!=0.0)&&(pXY[*aI][*bI]!=0.0))
				I+=pXY[*aI][*bI]*log2(pXY[*aI][*bI]/(pY[*bI]));
	return -I;
}

double tGame::computeAtomicPhi(vector<int>A,int states){
	int i;
	double P,EIsystem;
	vector<int> T0,T1;
	T0=A;
	T1=A;
	T0.erase(T0.begin()+T0.size()-1);
	T1.erase(T1.begin());
	EIsystem=ei(T0,T1,(1<<states)-1);
	P=0.0;
	for(i=0;i<states;i++){
		double EIP=ei(T0,T1,1<<i);
        //		cout<<EIP<<endl;
		P+=EIP;
	}
    //	cout<<-EIsystem+P<<" "<<EIsystem<<" "<<P<<" "<<T0.size()<<" "<<T1.size()<<endl;
	return -EIsystem+P;
}

double tGame::computeR(vector<vector<int> > table,int howFarBack){
	double Iwh,Iws,Ish,Hh,Hs,Hw,Hhws,delta,R;
	int i;
	for(i=0;i<howFarBack;i++){
		table[0].erase(table[0].begin());
		table[1].erase(table[1].begin());
		table[2].erase(table[2].begin()+(table[2].size()-1));
	}
	table[4].clear();
	for(i=0;i<table[0].size();i++){
		table[4].push_back((table[0][i]<<14)+(table[1][i]<<10)+table[2][i]);
	}
	Iwh=mutualInformation(table[0],table[2]);
    Iws=mutualInformation(table[0],table[1]);
    Ish=mutualInformation(table[1],table[2]);
    Hh=entropy(table[2]);
    Hs=entropy(table[1]);
    Hw=entropy(table[0]);
    Hhws=entropy(table[4]);
    delta=Hhws+Iwh+Iws+Ish-Hh-Hs-Hw;
    R=Iwh-delta;
  	return R;
}

double tGame::computeOldR(vector<vector<int> > table){
	double Ia,Ib;
	Ia=mutualInformation(table[0], table[2]);
	Ib=mutualInformation(table[1], table[2]);
	return Ib-Ia;
}

double tGame::predictiveI(vector<int>A){
	vector<int> S,I;
	S.clear(); I.clear();
	for(int i=0;i<A.size();i++){
		S.push_back((A[i]>>12)&15);
		I.push_back(A[i]&3);
	}
	return mutualInformation(S, I);
}

double tGame::nonPredictiveI(vector<int>A){
	vector<int> S,I;
	S.clear(); I.clear();
	for(int i=0;i<A.size();i++){
		S.push_back((A[i]>>12)&15);
		I.push_back(A[i]&3);
	}
	return entropy(I)-mutualInformation(S, I);
}

double tGame::predictNextInput(vector<int>A){
	vector<int> S,I;
	S.clear(); I.clear();
	for(int i=0;i<A.size();i++){
		S.push_back((A[i]>>12)&15);
		I.push_back(A[i]&3);
	}
	S.erase(S.begin());
	I.erase(I.begin()+I.size()-1);
	return mutualInformation(S, I);
}

void tGame::loadExperiment(char *filename){
    theExperiment.loadExperiment(filename);
}

//** tOctuplet implementation
void tOctuplet::loadOctuplet(FILE *f){
    int i,IN;
    data.clear();
    data.resize(8);
    for(i=0;i<8;i++){
        fscanf(f,"  %i",&IN);
        data[i]=IN;
    }
}

//** tEperiment class implementations
void tExperiment::loadExperiment(char *filename){
    FILE *f=fopen(filename,"r+t");
    int i,j,k;
    fscanf(f,"%i:",&j);
    dropSequences.resize(j);
    for(i=0;i<dropSequences.size();i++)
        dropSequences[i].loadOctuplet(f);
    fscanf(f,"%i:",&j);
    sizeSequences.resize(j);
    for(i=0;i<sizeSequences.size();i++)
        sizeSequences[i].loadOctuplet(f);
    fscanf(f,"%i:",&j);
    selfSequences.resize(j);
    for(i=0;i<selfSequences.size();i++)
        selfSequences[i].loadOctuplet(f);
    shouldHit.resize(drops());
    for(i=0;i<shouldHit.size();i++){
        shouldHit[i].resize(sizes());
        for(j=0;j<shouldHit[i].size();j++){
            shouldHit[i][j].resize(selves());
            for(k=0;k<shouldHit[i][j].size();k++){
                int l;
                fscanf(f,"%i\n",&l);
                if(l==1)
                    shouldHit[i][j][k]=true;
                else
                    shouldHit[i][j][k]=false;
            }
        }
    }
    fclose(f);
}

void tExperiment::showExperimentProtokoll(void){
    int i,j,k;
    printf("drop directions: %i\n",drops());
    for(i=0;i<drops();i++){
        printf("%i:",i);
        for(j=0;j<8;j++)
            printf("    %i",dropSequences[i].data[j]);
        printf("\n");
    }
    printf("drop sizes: %i\n",sizes());
    for(i=0;i<sizes();i++){
        printf("%i:",i);
        for(j=0;j<8;j++)
            printf("    %i",sizeSequences[i].data[j]);
        printf("\n");
    }
    printf("self sizes: %i\n",selves());
    for(i=0;i<selves();i++){
        printf("%i:",i);
        for(j=0;j<8;j++)
            printf("    %i",selfSequences[i].data[j]);
        printf("\n");
    }
    printf("should hit\n%i means true\nD  B   S   catch\n",(int)true);
    for(i=0;i<shouldHit.size();i++)
        for(j=0;j<shouldHit[i].size();j++)
            for(k=0;k<shouldHit[i][j].size();k++)
                printf("%i  %i  %i  %i\n",i,j,k,(int)shouldHit[i][j][k]);
}

int tExperiment::drops(void){
    return (int) dropSequences.size();
}

int tExperiment::sizes(void){
    return (int) sizeSequences.size();
}

int tExperiment::selves(void){
    return (int) selfSequences.size();
    
}
