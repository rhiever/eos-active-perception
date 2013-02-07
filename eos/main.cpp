/*
 * main.cpp
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
#include <algorithm>
#include <sstream>
#include <string.h>
#include <vector>
#include <map>
#include <math.h>
#include <time.h>
#include <iostream>
#include <dirent.h>

#include "globalConst.h"
#include "tHMM.h"
#include "tAgent.h"
#include "tGame.h"

#include <sys/socket.h>       /*  socket definitions        */
#include <sys/types.h>        /*  socket types              */
#include <arpa/inet.h>        /*  inet (3) funtions         */
#include <unistd.h>           /*  misc. UNIX functions      */

#include "helper.h"           /*  our own helper functions  */

#define ECHO_PORT          (2002)
#define MAX_LINE           (100000)

#define BUFLEN              512
#define NPACK               10
#define PORT                9930

int       list_s;                /*  listening socket          */
int       conn_s;                /*  connection socket         */
short int port;                  /*  port number               */
struct    sockaddr_in servaddr;  /*  socket address structure  */
char      buffer[MAX_LINE];      /*  character buffer          */
char     *endptr;                /*  for strtol()              */

void    setupBroadcast(void);
void    doBroadcast(string data);
string  findBestRun(tAgent *pathfinderAgent);

using namespace std;

//double  replacementRate             = 0.1;
double  perSiteMutationRate         = 0.005;
int     populationSize              = 100;
int     totalGenerations            = 1252;
tGame   *game                       = NULL;

bool    make_interval_video         = false;
int     make_video_frequency        = 25;
bool    make_LOD_video              = false;
bool    track_best_brains           = false;
int     track_best_brains_frequency = 25;
bool    display_only                = false;
bool    display_directory           = false;
bool    make_logic_table            = false;
bool    make_dot                    = false;

int main(int argc, char *argv[])
{
	vector<tAgent*> pathfinderAgents, PANextGen;
	tAgent *pathfinderAgent = NULL, *bestPathfinderAgent = NULL;
	double pathfinderMaxFitness = 0.0;
    string LODFileName = "", pathfinderGenomeFileName = "", inputGenomeFileName = "";
    string pathfinderDotFileName = "", logicTableFileName = "";
    int displayDirectoryArgvIndex = 0;
    
    // initial object setup
	game = new tGame;
	pathfinderAgent = new tAgent;
    
    // time-based seed by default. can change with command-line parameter.
    srand((unsigned int)time(NULL));
    
    for (int i = 1; i < argc; ++i)
    {
        // -d [in file name]: display
        if (strcmp(argv[i], "-d") == 0 && (i + 1) < argc)
        {
            ++i;
            pathfinderAgent->loadAgent(argv[i]);
            
            display_only = true;
        }
        
        // -dd [directory]: display all genome files in a given directory
        if (strcmp(argv[i], "-dd") == 0 && (i + 1) < argc)
        {
            ++i;
            displayDirectoryArgvIndex = i;
            
            display_directory = true;
        }
        
        // -e [out file name] [out file name]: evolve
        else if (strcmp(argv[i], "-e") == 0 && (i + 2) < argc)
        {
            ++i;
            stringstream lodfn;
            lodfn << argv[i];
            LODFileName = lodfn.str();
            
            ++i;
            stringstream sgfn;
            sgfn << argv[i];
            pathfinderGenomeFileName = sgfn.str();
        }
        
        // -s [int]: set seed
        else if (strcmp(argv[i], "-s") == 0 && (i + 1) < argc)
        {
            ++i;
            srand(atoi(argv[i]));
        }
        
        // -g [int]: set generations
        else if (strcmp(argv[i], "-g") == 0 && (i + 1) < argc)
        {
            ++i;
            
            // add 2 generations because we look at ancestor->ancestor as best agent at end of sim
            totalGenerations = atoi(argv[i]) + 2;
            
            if (totalGenerations < 3)
            {
                cerr << "minimum number of generations permitted is 3." << endl;
                exit(0);
            }
        }
        
        // -p [int]: set GA population size
        else if (strcmp(argv[i], "-p") == 0 && (i + 1) < argc)
        {
            ++i;
            
            populationSize = atoi(argv[i]);
            
            if (populationSize < 1)
            {
                cerr << "minimum GA population size permitted is 1." << endl;
                exit(0);
            }
        }
        
        // -t [int]: track best brains
        else if (strcmp(argv[i], "-t") == 0 && (i + 1) < argc)
        {
            track_best_brains = true;
            ++i;
            track_best_brains_frequency = atoi(argv[i]);
            
            if (track_best_brains_frequency < 1)
            {
                cerr << "minimum brain tracking frequency is 1." << endl;
                exit(0);
            }
        }
        
        // -v [int]: make video of best brains at an interval
        else if (strcmp(argv[i], "-v") == 0 && (i + 1) < argc)
        {
            make_interval_video = true;
            ++i;
            make_video_frequency = atoi(argv[i]);
            
            if (make_video_frequency < 1)
            {
                cerr << "minimum video creation frequency is 1." << endl;
                exit(0);
            }
        }
        
        // -lv: make video of LOD of best agent brain at the end
        else if (strcmp(argv[i], "-lv") == 0)
        {
            make_LOD_video = true;
        }
        
        // -lt [in file name] [out file name]: create logic table for given genome
        else if (strcmp(argv[i], "-lt") == 0 && (i + 2) < argc)
        {
            ++i;
            pathfinderAgent->loadAgent(argv[i]);
            pathfinderAgent->setupPhenotype();
            ++i;
            stringstream ltfn;
            ltfn << argv[i];
            logicTableFileName = ltfn.str();
            make_logic_table = true;
        }
        
        // -df [in file name] [out file name]: create dot image file for given genome
        else if (strcmp(argv[i], "-df") == 0 && (i + 2) < argc)
        {
            ++i;
            pathfinderAgent->loadAgent(argv[i]);
            pathfinderAgent->setupPhenotype();
            ++i;
            stringstream dfn;
            dfn << argv[i];
            pathfinderDotFileName = dfn.str();
            make_dot = true;
        }
}
    
    // initial population setup
    pathfinderAgents.resize(populationSize);
    
    if (display_only || display_directory || make_interval_video || make_LOD_video)
    {
        // start monitor first, then abeeda
        setupBroadcast();
    }
    
    if (display_only)
    {
        string bestString = findBestRun(pathfinderAgent);
        bestString.append("X");
        doBroadcast(bestString);
        exit(0);
    }
    
    if (display_directory)
    {
        DIR *dir;
        struct dirent *ent;
        dir = opendir(argv[displayDirectoryArgvIndex]);
        
        // map: run # -> [swarm file name, predator file name]
        map< int, vector<string> > fileNameMap;
        
        if (dir != NULL)
        {
            cout << "reading in files" << endl;
            
            // read all of the files in the directory
            while ((ent = readdir(dir)) != NULL)
            {
                string dirFile = string(ent->d_name);
                
                if (dirFile.find(".genome") != string::npos)
                {
                    // find the first character in the file name that is a digit
                    int i = 0;
                    
                    for ( ; i < dirFile.length(); ++i)
                    {
                        if (isdigit(dirFile[i]))
                        {
                            break;
                        }
                    }
                    
                    // get the run number from the file name
                    int runNumber = atoi(dirFile.substr(i).c_str());
                    
                    if (fileNameMap[runNumber].size() == 0)
                    {
                        fileNameMap[runNumber].resize(2);
                    }
                    
                    // map the file name into the appropriate location
                    if (dirFile.find("pathfinder") != string::npos)
                    {
                        fileNameMap[runNumber][0] = argv[displayDirectoryArgvIndex] + dirFile;
                    }
                }
            }
            
            closedir(dir);
        }
        else
        {
            cerr << "invalid directory: " << argv[displayDirectoryArgvIndex] << endl;
            exit(0);
        }
        
        // display every set of swarm/predator files
        for (map< int, vector<string> >::iterator it = fileNameMap.begin(), end = fileNameMap.end(); it != end; )
        {
            if (it->second[0] != "" && it->second[1] != "")
            {
                cout << "building video for run " << it->first << endl;
                
                pathfinderAgent->loadAgent((char *)it->second[0].c_str());
                
                string bestString = findBestRun(pathfinderAgent);
                
                cout << "displaying video for run " << it->first << endl;
                
                if ( (++it) == end )
                {
                    bestString.append("X");
                }
                
                doBroadcast(bestString);
            }
            else
            {
                cerr << "unmatched file: " << it->second[0] << " " << it->second[1] << endl;
            }
        }
        
        exit(0);
    }
    
    if (make_logic_table)
    {
        pathfinderAgent->saveLogicTable(logicTableFileName.c_str());
        exit(0);
    }
    
    if (make_dot)
    {
        pathfinderAgent->saveToDot(pathfinderDotFileName.c_str(), false);
        exit(0);
    }
    
    // seed the agents
    delete pathfinderAgent;
    pathfinderAgent = new tAgent;
    pathfinderAgent->setupRandomAgent(initialGenomeSize);
    //pathfinderAgent->loadAgent("startPathfinderAgent.genome");
    
    // make mutated copies of the start genome to fill up the initial population
	for(int i = 0; i < populationSize; ++i)
    {
		pathfinderAgents[i] = new tAgent;
		pathfinderAgents[i]->inherit(pathfinderAgent, 0.01, 1);
    }
    
	PANextGen.resize(populationSize);
    PANextGen.resize(populationSize);
    
	pathfinderAgent->nrPointingAtMe--;
    
	cout << "setup complete" << endl;
    cout << "starting evolution" << endl;
    
    // main loop
	for (int update = 1; update <= totalGenerations; ++update)
    {
        // reset fitnesses
		for(int i = 0; i < populationSize; ++i)
        {
			pathfinderAgents[i]->fitness = 0.0;
			//pathfinderAgents[i]->fitnesses.clear();
		}
        
        // determine fitness of population
		pathfinderMaxFitness = 0.0;
        double pathfinderAvgFitness = 0.0;
        
		for(int i = 0; i < populationSize; ++i)
        {
            game->executeGame(pathfinderAgents[i], NULL, false);
            
            pathfinderAvgFitness += pathfinderAgents[i]->fitness;
            
            //pathfinderAgents[i]->fitnesses.push_back(pathfinderAgents[i]->fitness);
            //predatorAgents[i]->fitnesses.push_back(predatorAgents[i]->fitness);
            
            if(pathfinderAgents[i]->fitness > pathfinderMaxFitness)
            {
                pathfinderMaxFitness = pathfinderAgents[i]->fitness;
                bestPathfinderAgent = pathfinderAgents[i];
            }
		}
        
        pathfinderAvgFitness /= (double)populationSize;
		
        if (update == 1 || update % 50 == 0)
        {
            cout << "generation " << update << ": pathfinder [" << pathfinderAvgFitness << " : " << pathfinderMaxFitness << "] " << bestPathfinderAgent->hmmus.size() << " " << bestPathfinderAgent->genome.size() << endl;
        }
        
        // display video of simulation
        if (make_interval_video)
        {
            bool finalGeneration = (update == totalGenerations);
            
            if (update % make_video_frequency == 0 || finalGeneration)
            {
                string bestString = game->executeGame(bestPathfinderAgent, NULL, true);
                
                if (finalGeneration)
                {
                    bestString.append("X");
                }
                
                doBroadcast(bestString);
            }
        }
        
        
		for(int i = 0; i < populationSize; ++i)
		{
            // construct pathfinder agent population for the next generation
			tAgent *offspring = new tAgent;
            int j = 0;
            
			do
            {
                j = rand() % populationSize;
            } while((j == i) || (randDouble > (pathfinderAgents[j]->fitness / pathfinderMaxFitness)));
            
			offspring->inherit(pathfinderAgents[j], perSiteMutationRate, update);
			PANextGen[i] = offspring;
		}
        
		for(int i = 0; i < populationSize; ++i)
        {
            // retire and replace the swarm agents from the previous generation
			pathfinderAgents[i]->retire();
			pathfinderAgents[i]->nrPointingAtMe--;
			if(pathfinderAgents[i]->nrPointingAtMe == 0)
            {
				delete pathfinderAgents[i];
            }
			pathfinderAgents[i] = PANextGen[i];
		}
        
		pathfinderAgents = PANextGen;
        
        if (track_best_brains && update % track_best_brains_frequency == 0)
        {
            stringstream pss;
            
            pss << "pathfinder" << update << ".genome";
            
            pathfinderAgents[0]->ancestor->ancestor->saveGenome(pss.str().c_str());
        }
	}
	
    // save the genome file of the lmrca
	pathfinderAgents[0]->ancestor->ancestor->saveGenome(pathfinderGenomeFileName.c_str());
    
    // save video and quantitative stats on the best swarm agent's LOD
    vector<tAgent*> saveLOD;
    
    cout << "building ancestor list" << endl;
    
    // use 2 ancestors down from current population because that ancestor is highly likely to have high fitness
    tAgent* curAncestor = pathfinderAgents[0]->ancestor->ancestor;
    
    while (curAncestor != NULL)
    {
        // don't add the base ancestor
        if (curAncestor->ancestor != NULL)
        {
            saveLOD.insert(saveLOD.begin(), curAncestor);
        }
        
        curAncestor = curAncestor->ancestor;
    }
    
    FILE *LOD = fopen(LODFileName.c_str(), "w");

    fprintf(LOD, "generation,pathfinder_fitness\n");
    
    cout << "analyzing ancestor list" << endl;
    
    for (vector<tAgent*>::iterator it = saveLOD.begin(); it != saveLOD.end(); ++it)
    {
        // collect quantitative stats
        game->executeGame(*it, LOD, false);
        
        // make video
        if (make_LOD_video)
        {
            string bestString = findBestRun(pathfinderAgent);
            
            if ( (it + 1) == saveLOD.end() )
            {
                bestString.append("X");
            }
            
            doBroadcast(bestString);
        }
    }
    
    fclose(LOD);
    
    return 0;
}

string findBestRun(tAgent *pathfinderAgent)
{
    string reportString = "", bestString = "";
    double bestFitness = 0.0;
    
    for (int rep = 0; rep < 100; ++rep)
    {
        reportString = game->executeGame(pathfinderAgent, NULL, true);
        
        if (pathfinderAgent->fitness > bestFitness)
        {
            bestString = reportString;
            bestFitness = pathfinderAgent->fitness;
        }
    }
    
    return bestString;
}

void setupBroadcast(void)
{
    port = ECHO_PORT;
	if ( (list_s = socket(AF_INET, SOCK_STREAM, 0)) < 0 )
    {
		fprintf(stderr, "ECHOSERV: Error creating listening socket.\n");
    }
	/*  Set all bytes in socket address structure to
	 zero, and fill in the relevant data members   */
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family      = AF_INET;
    servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    servaddr.sin_port        = htons(port);
	/*  Bind our socket addresss to the 
	 listening socket, and call listen()  */
    if ( bind(list_s, (struct sockaddr *) &servaddr, sizeof(servaddr)) < 0 )
    {
		fprintf(stderr, "ECHOSERV: Error calling bind()\n");
    }
    if ( listen(list_s, LISTENQ) < 0 )
    {
		fprintf(stderr, "ECHOSERV: Error calling listen()\n");
    }
}

void doBroadcast(string data)
{
    if ( (conn_s = accept(list_s, NULL, NULL) ) < 0 )
    {
        fprintf(stderr, "ECHOSERV: Error calling accept()\n");
    }
    Writeline(conn_s, data.c_str(), data.length());
    
    if ( close(conn_s) < 0 )
    {
        fprintf(stderr, "ECHOSERV: Error calling close()\n");
    }
}
