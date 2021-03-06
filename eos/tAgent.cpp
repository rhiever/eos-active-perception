/*
 * tAgent.cpp
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

#include <stdio.h>
#include <stdlib.h>
#include <map>
#include <math.h>
#include "tAgent.h"

tAgent::tAgent(){
	nrPointingAtMe=1;
	ancestor = NULL;
    predator = NULL;
	for(int i=0;i<maxNodes;i++)
    {
		states[i]=0;
		newStates[i]=0;
	}
	bestSteps=-1;
	ID=masterID;
	masterID++;
	saved=false;
	hmmus.clear();
	nrOfOffspring=0;
	retired=false;
	food=0;
    totalSteps=0;
#ifdef useANN
	ANN=new tANN;
#endif
}

tAgent::~tAgent()
{
	for (int i = 0; i < hmmus.size(); ++i)
    {
        delete hmmus[i];
    }
    
    if (predator != NULL)
    {
        delete predator;
    }
    
	if (ancestor!=NULL)
    {
		ancestor->nrPointingAtMe--;
		if (ancestor->nrPointingAtMe == 0)
        {
			delete ancestor;
        }
	}
#ifdef useANN
	delete ANN;
#endif
}

void tAgent::setupRandomAgent(int nucleotides)
{
	int i;
	genome.resize(nucleotides);
	for(i=0;i<nucleotides;i++)
		genome[i]=127;//rand()&255;
	ampUpStartCodons();
//	setupPhenotype();
#ifdef useANN
	ANN->setup();
#endif
}
void tAgent::loadAgent(char* filename)
{
	FILE *f=fopen(filename,"r+t");
	int i;
	genome.clear();
	while(!(feof(f)))
    {
		fscanf(f,"%i	",&i);
		genome.push_back((unsigned char)(i&255));
	}
	fclose(f);
	//setupPhenotype();
}

void tAgent::loadAgentWithTrailer(char* filename)
{
#ifdef useANN
	ANN=new tANN;
	ANN->load(filename);
#else
	FILE *f=fopen(filename,"r+t");
	int i;
	genome.clear();
	fscanf(f,"%i	",&i);
	while(!(feof(f))){
		fscanf(f,"%i	",&i);
		genome.push_back((unsigned char)(i&255));
	}
	//setupPhenotype();
#endif
}


void tAgent::ampUpStartCodons(void)
{
	int i,j;
	for(i=0;i<genome.size();i++)
    {
		genome[i]=rand()&255;
    }
	for(i=0;i<numStartingGates;i++)
	{
		j=rand()%((int)genome.size()-100);
		genome[j]=42;
		genome[j+1]=(255-42);
		for(int k=2;k<20;k++)
			genome[j+k]=rand()&255;
	}
}

void tAgent::inherit(tAgent *from, double mutationRate, int theTime)
{
	int nucleotides=(int)from->genome.size();
	int i,s,o,w;
	//double localMutationRate=4.0/from->genome.size();
	vector<unsigned char> buffer;
	born=theTime;
	ancestor=from;
	from->nrPointingAtMe++;
	from->nrOfOffspring++;
	genome.clear();
	genome.resize(from->genome.size());
	for(i=0;i<nucleotides;i++)
    {
		if(((double)rand()/(double)RAND_MAX)<mutationRate)
        {
			genome[i]=rand()&255;
        }
		else
        {
			genome[i]=from->genome[i];
        }
    }
    
    if (mutationRate != 0.0)
    {
        if((((double)rand()/(double)RAND_MAX)<0.05)&&(genome.size()<maxGenomeSize))
        {
            //duplication
            w=15+rand()&511;
            s=rand()%((int)genome.size()-w);
            o=rand()%(int)genome.size();
            buffer.clear();
            buffer.insert(buffer.begin(),genome.begin()+s,genome.begin()+s+w);
            genome.insert(genome.begin()+o,buffer.begin(),buffer.end());
        }
        if((((double)rand()/(double)RAND_MAX)<0.02)&&(genome.size()>minGenomeSize))
        {
            //deletion
            w=15+rand()&511;
            s=rand()%((int)genome.size()-w);
            genome.erase(genome.begin()+s,genome.begin()+s+w);
        }
    }

	//setupPhenotype();
	fitness=0.0;
#ifdef useANN
	ANN->inherit(ancestor->ANN,mutationRate);
#endif
}

void tAgent::setupPhenotype(void)
{
	int i;
	tHMMU *hmmu;
	if(hmmus.size()!=0)
    {
		for(i=0;i<hmmus.size();i++)
        {
			delete hmmus[i];
        }
    }
	hmmus.clear();
	for(i=0;i<genome.size();i++)
    {
		if((genome[i]==42)&&(genome[(i+1)%genome.size()]==(255-42)))
        {
			hmmu=new tHMMU;
			//hmmu->setupQuick(genome,i);
			hmmu->setup(genome,i);
			hmmus.push_back(hmmu);
		}
        /*
		if((genome[i]==43)&&(genome[(i+1)%genome.size()]==(255-43))){
			hmmu=new tHMMU;
			//hmmu->setup(genome,i);
			hmmu->setupQuick(genome,i);
			hmmus.push_back(hmmu);
		}
         */
	}
}
void tAgent::setupMegaPhenotype(int howMany)
{
	int i,j;
	tHMMU *hmmu;
    
	if(hmmus.size() > 0)
    {
		for(vector<tHMMU*>::iterator it = hmmus.begin(), end = hmmus.end(); it != end; ++it)
        {
			delete *it;
        }
    }
	hmmus.clear();
	for(i=0;i<genome.size();i++)
    {
		if((genome[i]==42)&&(genome[(i+1)%genome.size()]==(255-42)))
        {
            for(j=0;j<howMany;j++)
            {
                hmmu=new tHMMU;
                hmmu->setup(genome, i);
                //hmmu->setupQuick(genome,i);
                for(int k=0;k<4;k++){
                    hmmu->ins[k]+=(j*maxNodes);
                    hmmu->outs[k]+=(j*maxNodes);
                }
                hmmus.push_back(hmmu);
            }
        }
        /*
         if((genome[i]==43)&&(genome[(i+1)%genome.size()]==(255-43))){
         hmmu=new tHMMU;
         //hmmu->setup(genome,i);
         hmmu->setupQuick(genome,i);
         hmmus.push_back(hmmu);
         }
         */
	}
    
}


void tAgent::retire(void)
{
	retired=true;
}

unsigned char * tAgent::getStatesPointer(void)
{
	return states;
}

void tAgent::resetBrain(void)
{
	for(int i=0;i<maxNodes;i++)
    {
		states[i]=0;
    }
#ifdef useANN
	ANN->resetBrain();
#endif
}

void tAgent::updateStates(void)
{
	for(vector<tHMMU*>::iterator it = hmmus.begin(), end = hmmus.end(); it != end; ++it)
    {
		(*it)->update(&states[0],&newStates[0]);
    }
    
	for(int i=0;i<maxNodes;i++)
    {
		states[i]=newStates[i];
		newStates[i]=0;
	}
	++totalSteps;
}

void tAgent::showBrain(void)
{
	for(int i=0;i<maxNodes;i++)
    {
		cout<<(int)states[i];
    }
	cout<<endl;
}

void tAgent::initialize(int x, int y, int d)
{
	//int i,j;
	//unsigned char dummy;
	xPos=x;
	yPos=y;
	direction=d;
	steps=0;
	/*
	if((rand()&1)==0){
		scramble[1]=2;
		scramble[2]=1;
	}
	*/
}

tAgent* tAgent::findLMRCA(void)
{
	tAgent *r,*d;
	if(ancestor==NULL)
		return NULL;
	else{
		r=ancestor;
		d=NULL;
		while(r->ancestor!=NULL){
			if(r->ancestor->nrPointingAtMe!=1)
				d=r;
			r=r->ancestor;
		}
		return d;
	}
}

void tAgent::saveFromLMRCAtoNULL(FILE *statsFile,FILE *genomeFile){
	if(ancestor!=NULL)
		ancestor->saveFromLMRCAtoNULL(statsFile,genomeFile);
	if(!saved){ 
		fprintf(statsFile,"%i	%i	%i	%f	%i	%f	%i	%i\n",ID,born,(int)genome.size(),fitness,bestSteps,(float)totalSteps/(float)nrOfOffspring,correct,incorrect);
		fprintf(genomeFile,"%i	",ID);
		for(int i=0;i<genome.size();i++)
			fprintf(genomeFile,"	%i",genome[i]);
		fprintf(genomeFile,"\n");
		saved=true;
	}
	if((saved)&&(retired)) genome.clear();
}

/*
void tAgent::saveLOD(FILE *statsFile,FILE *genomeFile){
	if(ancestor!=NULL)
		ancestor->saveLOD(statsFile,genomeFile);
#ifdef useANN
	fprintf(genomeFile,"%i	",ID);
	fprintf(statsFile,"%i	%i	%i	%f	%i	%f	%i	%i\n",ID,born,(int)genome.size(),fitness,bestSteps,(float)totalSteps/(float)nrOfOffspring,correct,incorrect);
	ANN->saveLOD(genomeFile);
#else	
	fprintf(statsFile,"%i	%i	%i	%f	%i	%f	%i	%i\n",ID,born,(int)genome.size(),fitness,bestSteps,(float)totalSteps/(float)nrOfOffspring,correct,incorrect);
	fprintf(genomeFile,"%i	",ID);
	for(int i=0;i<genome.size();i++)
		fprintf(genomeFile,"	%i",genome[i]);
	fprintf(genomeFile,"\n");
#endif
	
}*/

void tAgent::showPhenotype(void)
{
	for(int i=0;i<hmmus.size();i++)
		hmmus[i]->show();
	cout<<"------"<<endl;
}

void tAgent::saveToDot(const char *filename)
{
	FILE *f=fopen(filename,"w+t");
	fprintf(f,"digraph brain {\n");
	fprintf(f,"	ranksep=2.0;\n");
    
    // determine which nodes to print (no connections to output = do not print)
    bool connects_to_input[maxNodes];
    bool connects_to_output[maxNodes];
    
    for(int i = 0; i < maxNodes; ++i)
    {
        if (i <= 180)
        {
            connects_to_input[i] = true;
        }
        else
        {
            connects_to_input[i] = false;
        }
        
        connects_to_output[i] = false;
    }
    
    // outputs of course "connect" to themselves!
    connects_to_output[maxNodes - 1] = connects_to_output[maxNodes - 2] = true;
    
    for (int reps = 0; reps < 5; ++reps)
    {
        for (int i = 0; i < hmmus.size(); ++i)
        {
            // connects to inputs?
            for (int j = 0; j < hmmus[i]->ins.size(); ++j)
            {
                if (hmmus[i]->ins[j] <= 180 || connects_to_input[hmmus[i]->ins[j]])
                {
                    for (int k = 0; k < hmmus[i]->outs.size(); ++k)
                    {
                        connects_to_input[hmmus[i]->outs[k]] = true;
                    }
                }
            }
            
            // connects to outputs?
            for (int j = 0; j < hmmus[i]->outs.size(); ++j)
            {
                if (hmmus[i]->outs[j] >= 254 || connects_to_output[hmmus[i]->outs[j]])
                {
                    for (int k = 0; k < hmmus[i]->ins.size(); ++k)
                    {
                        connects_to_output[hmmus[i]->ins[k]] = true;
                    }
                }
            }
        }
    }
    
    /*for(int i=0;i<hmmus.size();i++)
    {
        for(int j=0;j<hmmus[i]->ins.size();j++)
        {
            connects_to_input[hmmus[i]->ins[j]] = connects_to_output[hmmus[i]->ins[j]] = true;
        }
        
        for(int k=0;k<hmmus[i]->outs.size();k++)
        {
            connects_to_input[hmmus[i]->outs[k]] = connects_to_output[hmmus[i]->outs[k]] = true;
        }
    }*/
    
    // input layer
	for(int node=0;node<181;node++)
    {
        if(connects_to_input[node] && connects_to_output[node])
        {
            fprintf(f,"	%i [shape=invtriangle,style=filled,color=cornflowerblue];\n",node);
        }
    }
    
    // hidden states
    for(int node=181;node<maxNodes-2;node++)
    {
        if(connects_to_input[node] && connects_to_output[node])
        {
            fprintf(f,"	%i [shape=circle,color=black];\n",node);
        }
    }
    
    // outputs
	for(int node=maxNodes-2;node<maxNodes;node++)
    {
		fprintf(f,"	%i [shape=triangle,style=filled,color=green];\n",node);
    }
    
    // connections
	for(int i=0;i<hmmus.size();i++)
    {
		for(int j=0;j<hmmus[i]->ins.size();j++)
        {
            if (connects_to_output[hmmus[i]->ins[j]] && connects_to_input[hmmus[i]->ins[j]])
            {
                for(int k=0;k<hmmus[i]->outs.size();k++)
                {
                    // don't show connections leading to inputs
                    if (hmmus[i]->outs[k] > 180)
                    {
                        fprintf(f,"	%i	->	%i;\n",hmmus[i]->ins[j],hmmus[i]->outs[k]);
                    }
                }
            }
		}
	}
    
    // which nodes go on the same level
    
    // inputs
    fprintf(f,"	{ rank=same; ");
    
    for(int node = 0; node < 181; node++)
    {
        if(connects_to_input[node] && connects_to_output[node])
        {
            fprintf(f, "%d; ", node);
        }
    }
    
    fprintf(f, "}\n");
    
    // hidden states
    fprintf(f,"	{ rank=same; ");
    
    for(int node = 181; node < maxNodes-2; node++)
    {
        if(connects_to_input[node] && connects_to_output[node])
        {
            fprintf(f, "%d; ", node);
        }
    }
    
    fprintf(f, "}\n");
    
    // outputs
    fprintf(f,"	{ rank=same; ");
    
    for(int node = maxNodes-2; node < maxNodes; node++)
    {
        fprintf(f, "%d; ", node);
    }
    
	fprintf(f,"}\n}");
	fclose(f);
}

void tAgent::saveToDotFullLayout(char *filename){
	FILE *f=fopen(filename,"w+t");
	int i,j,k;
	fprintf(f,"digraph brain {\n");
	fprintf(f,"	ranksep=2.0;\n");
	for(i=0;i<hmmus.size();i++){
		fprintf(f,"MM_%i [shape=box]\n",i);
		for(j=0;j<hmmus[i]->ins.size();j++)
			fprintf(f,"	t0_%i -> MM_%i\n",hmmus[i]->ins[j],i);
		for(k=0;k<hmmus[i]->outs.size();k++)
			fprintf(f,"	MM_%i -> t1_%i\n",i,hmmus[i]->outs[k]);
		
	}
	fprintf(f,"}\n");
}

void tAgent::setupDots(int x, int y,double spacing){
	double xo,yo;
	int i,j,k;
	xo=(double)(x-1)*spacing;
	xo=-(xo/2.0);
	yo=(double)(y-1)*spacing;
	yo=-(yo/2.0);
	dots.resize(x*y);
	k=0;
	for(i=0;i<x;i++)
		for(j=0;j<y;j++){
//			dots[k].xPos=(double)(rand()%(int)(spacing*x))+xo;
//			dots[k].yPos=(double)(rand()%(int)(spacing*y))+yo;
			dots[k].xPos=xo+((double)i*spacing);
			dots[k].yPos=yo+((double)j*spacing);
//			cout<<dots[k].xPos<<" "<<dots[k].yPos<<endl;
			k++;
		}
}

void tAgent::saveLogicTable(const char *filename)
{
    FILE *f=fopen(filename, "w");
    
    // determine which nodes to print (no connections to output = do not print)
    bool connects_to_input[maxNodes];
    bool connects_to_output[maxNodes];
    
    for(int i = 0; i < maxNodes; ++i)
    {
        if (i <= 180)
        {
            connects_to_input[i] = true;
        }
        else
        {
            connects_to_input[i] = false;
        }
        
        connects_to_output[i] = false;
    }
    
    // outputs of course "connect" to themselves!
    connects_to_output[maxNodes - 1] = connects_to_output[maxNodes - 2] = true;
    
    for (int reps = 0; reps < 5; ++reps)
    {
        for (int i = 0; i < hmmus.size(); ++i)
        {
            // connects to inputs?
            for (int j = 0; j < hmmus[i]->ins.size(); ++j)
            {
                if (hmmus[i]->ins[j] <= 180 || connects_to_input[hmmus[i]->ins[j]])
                {
                    for (int k = 0; k < hmmus[i]->outs.size(); ++k)
                    {
                        connects_to_input[hmmus[i]->outs[k]] = true;
                    }
                }
            }
            
            // connects to outputs?
            for (int j = 0; j < hmmus[i]->outs.size(); ++j)
            {
                if (hmmus[i]->outs[j] >= 254 || connects_to_output[hmmus[i]->outs[j]])
                {
                    for (int k = 0; k < hmmus[i]->ins.size(); ++k)
                    {
                        connects_to_output[hmmus[i]->ins[k]] = true;
                    }
                }
            }
        }
    }
    
    /*for(int i=0;i<hmmus.size();i++)
     {
         for(int j=0;j<hmmus[i]->ins.size();j++)
         {
            connects_to_output[hmmus[i]->ins[j]] = true;
         }
         
         for(int k=0;k<hmmus[i]->outs.size();k++)
         {
            connects_to_output[hmmus[i]->outs[k]] = true;
         }
     }*/
    
    vector<int> statesUsed;
    
    for (int i = 0; i < maxNodes - 2; ++i)
    {
        if (connects_to_input[i] && connects_to_output[i])
        {
            statesUsed.push_back(i);
        }
    }
    
    fprintf(f, ".i %d\n.o 2\n.ilb", (int)statesUsed.size());
    
    for (int i = 0; i < statesUsed.size(); ++i)
    {
        fprintf(f," s%i", statesUsed[i]);
    }
    
    fprintf(f, "\n.ob o1 o2\n");
    
    for(long i = 0; i < pow(2.0, statesUsed.size()); ++i)
    {
        map<vector<int>, int> outputCounts;
        const int NUM_REPEATS = 1001;
        
        for (int repeat = 1; repeat < NUM_REPEATS; ++repeat)
        {
            int stateCount = 0;
            
            for (int j = 0; j < maxNodes - 2; ++j)
            {
                if (j == statesUsed[stateCount])
                {
                    if (repeat == 1)
                    {
                        fprintf(f, "%lu ", (i >> stateCount) & 1);
                    }
                    
                    states[j] = (i >> stateCount) & 1;
                    
                    ++stateCount;
                }
            }
            
            updateStates();
            
            vector<int> output;
            // order: 254 255
            output.push_back(states[maxNodes - 2]);
            output.push_back(states[maxNodes - 1]);
            
            if (outputCounts.count(output) > 0)
            {
                outputCounts[output]++;
            }
            else
            {
                outputCounts[output] = 1;
            }
            
            // all repeats completed; determine most common output
            if (repeat == (NUM_REPEATS - 1))
            {
                map<vector<int>, int>::iterator it;
                map<vector<int>, int>::iterator mostCommonOutput = outputCounts.begin();
                
                for (it = outputCounts.begin(); it != outputCounts.end(); ++it)
                {
                    if (it->second > mostCommonOutput->second)
                    {
                        mostCommonOutput = it;
                    }
                }
                
                fprintf(f, "%i %i\n", mostCommonOutput->first[0], mostCommonOutput->first[1]);
            }
        }
	}
    
    fprintf(f, ".e\n");
    
    fclose(f);
}

void tAgent::saveGenome(const char *filename)
{
    FILE *f=fopen(filename, "w");
    
	for (int i = 0, end = (int)genome.size(); i < end; ++i)
    {
		fprintf(f, "%i	", genome[i]);
    }
    
	fprintf(f, "\n");
    
    fclose(f);
}
