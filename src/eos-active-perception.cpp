/* main.cpp
 * 
 * This file is part of the Evolution of Swarming project.
 * 
 * Copyright 2013 Randal S. Olson.
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
#include <ea/evolutionary_algorithm.h>
#include <ea/generational_models/death_birth_process.h>
#include <ea/representations/circular_genome.h>
#include <ea/fitness_function.h>
#include <ea/cmdline_interface.h>
#include <ea/datafiles/fitness.h>
#include <ea/markov_network.h>

#include <sys/socket.h>       /*  socket definitions        */
#include <sys/types.h>        /*  socket types              */
#include <arpa/inet.h>        /*  inet (3) funtions         */
#include <unistd.h>           /*  misc. UNIX functions      */

#include "helper.h"           /*  our own helper functions  */

using namespace ea;

LIBEA_MD_DECL(OUT_FILE_NAME, "ea.fitness_function.out_file_name", std::string);
LIBEA_MD_DECL(VIDEO_PERIOD, "ea.fitness_function.video_period", int);

/*! Fitness function for pathfinder.
 */
struct pathfinder_fitness : fitness_function<unary_fitness<double>, constantS, absoluteS, stochasticS>
{
        
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
    
    // simulation-specific constants
    #define     cPI                             3.14159265
    #define     pathfinderVisionRange           50.0 * 50.0
    #define     pathfinderSize                  5.0
    #define     totalStepsInSimulation          2000
    #define     gridX                           400.0
    #define     gridY                           400.0
    #define     gridXAcross                     2.0 * gridX
    #define     gridYAcross                     2.0 * gridY
    #define     collisionDist                   5.0 * 5.0
    #define     boundaryDist                    400.0
    #define     numFood                         100
    
    // precalculated lookup tables for the game
    double cosLookup[360];
    double sinLookup[360];
    
    // internal flag indicating whether a video should be generated for the current simulation
    bool report;
    
    /*! Initialize this fitness function -- load data, etc. */
    template <typename RNG, typename EA>
    void initialize(RNG& rng, EA& ea)
    {
        report = false;
        
        if (get<VIDEO_PERIOD>(ea) > 0)
        {
            setupBroadcast();
        }
        
        // fill lookup tables
        for (int i = 0; i < 360; ++i)
        {
            cosLookup[i] = cos((double)i * (cPI / 180.0));
            sinLookup[i] = sin((double)i * (cPI / 180.0));
        }
    }
    
    template <typename Individual, typename RNG, typename EA>
	void record(Individual& ind, RNG& rng, EA& ea)
    {
        report = true;
        operator()(ind,rng,ea);
        report = false;
    }
    
    ~pathfinder_fitness()
    {
        // send signal to processing to close video
        ;
    }
    
	template <typename Individual, typename RNG, typename EA>
	double operator()(Individual& ind, RNG& rng, EA& ea)
    {
        using namespace mkv;
        
        markov_network net(make_markov_network_desc(get<MKV_DESC>(ea)), rng);
        
        // build a markov network from the individual's genome:
        mkv::build_markov_network(net, ind.repr().begin(), ind.repr().end(), ea);
        
        // initial simulation space setup
        std::string reportString = "";
        double pathfinderX = 0.0, pathfinderY = 0.0, pathfinderAngle = 0.0, pathfinderFitness = 0.0;
        double foodX[numFood], foodY[numFood], foodSize[numFood];
        bool foodEaten[numFood];
        
        pathfinderX = 0.0;//0.5 * ((2.0 * rng.uniform_real(0, 1) * gridX) - gridX);
        pathfinderY = 0.0;//0.5 * ((2.0 * rng.uniform_real(0, 1) * gridY) - gridY);
        pathfinderAngle = (int)(rng.uniform_real(0, 1) * 360.0);
        
        for (int i = 0; i < numFood; ++i)
        {
            bool goodPos = true;
            
            if (i > numFood / 4)
            {
                foodSize[i] = 5.0;
            }
            else
            {
                foodSize[i] = 20.0;
            }
            
            do
            {
                goodPos = true;
                foodX[i] = 0.95 * ((2.0 * rng.uniform_real(0, 1) * gridX) - gridX);
                foodY[i] = 0.95 * ((2.0 * rng.uniform_real(0, 1) * gridY) - gridY);
                
                for (int j = 0; j < i; ++j)
                {
                    if (calcDistanceSquared(foodX[i], foodY[i], foodX[j], foodY[j]) - pow(foodSize[i], 2.0) / 2.0 - pow(foodSize[j], 2.0) / 2.0 <= pathfinderVisionRange)
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
            
            if (report)
            {
                // report X, Y, angle of pathfinding agent
                char text1[1000];
                sprintf(text1, "%f,%f,%f,%f,%d,%d,%d=", pathfinderX, pathfinderY, pathfinderAngle, pathfinderSize, 255, 255, 255);
                reportString.append(text1);
                
                // report X, Y of food
                for (int i = 0; i < numFood; ++i)
                {
                    if (!foodEaten[i])
                    {
                        char text2[1000];
                        sprintf(text2, "%f,%f,%f,%f,%d,%d,%d=", foodX[i], foodY[i], 0.0, foodSize[i], 0, 255, 0);
                        reportString.append(text2);
                    }
                }
                
                reportString.append("N");
            }
            
            /*       END OF REPORT STRING CREATION       */
            
            
            /*       SAVE DATA FOR THE LOD FILE       */
            
            /*if (data_file != NULL)
            {
             
            }*/
            
            /*       END OF DATA GATHERING       */
            
            
            /*       UPDATE PATHFINDER       */
            
            // update the pathfinder sensors
            std::vector<int> inputs(net.ninput_states(), 0);
            
            for (int i = 0; i < numFood; ++i)
            {
                if (!foodEaten[i])
                {
                    double distToFood = calcDistanceSquared(pathfinderX, pathfinderY, foodX[i], foodY[i]) - pow(pathfinderSize, 2.0) / 2.0 - pow(foodSize[i], 2.0) / 2.0;
                    
                    if (distToFood <= pathfinderVisionRange)
                    {
                        double foodRadius = foodSize[i] / 2.0;
                        
                        int angleTL = (int)calcAngle(pathfinderX, pathfinderY, pathfinderAngle, foodX[i] - foodRadius, foodY[i] + foodRadius);
                        int angleTR = (int)calcAngle(pathfinderX, pathfinderY, pathfinderAngle, foodX[i] + foodRadius, foodY[i] + foodRadius);
                        int angleBL = (int)calcAngle(pathfinderX, pathfinderY, pathfinderAngle, foodX[i] - foodRadius, foodY[i] - foodRadius);
                        int angleBR = (int)calcAngle(pathfinderX, pathfinderY, pathfinderAngle, foodX[i] + foodRadius, foodY[i] - foodRadius);
                        
                        int activateFromAngle = fmin(angleTL, fmin(angleTR, fmin(angleBL, angleBR)));
                        int activateToAngle = fmax(angleTL, fmax(angleTR, fmax(angleBL, angleBR)));
                        
                        // keep angles within -90 and 90
                        activateFromAngle = fmax(-90, activateFromAngle);
                        activateToAngle = fmin(90, activateToAngle);
                        
                        for (int j = activateFromAngle; j <= activateToAngle; ++j)
                        {
                            inputs[j + 90] = 1;
                        }
                    }
                }
            }
            
            // activate the pathfinder's Markov network
            update(net, get<MKV_UPDATE_N>(ea), inputs.begin());
            
            int action = algorithm::range2int(net.begin_output(), net.end_output());
            
            switch(action)
            {
                // move straight ahead
                case 0:
                    pathfinderX += cosLookup[(int)pathfinderAngle] * 2.0;
                    pathfinderY += sinLookup[(int)pathfinderAngle] * 2.0;
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
                    
                //
                case 3:
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
                    double distToFood = calcDistanceSquared(pathfinderX, pathfinderY, foodX[i], foodY[i]) - pow(pathfinderSize, 2.0) / 2.0 - pow(foodSize[i], 2.0) / 2.0;
                    
                    if (distToFood <= 0.0)
                    {
                        foodEaten[i] = true;
                        
                        if (foodSize[i] == 5.0)
                        {
                            pathfinderFitness += 1;
                        }
                        else
                        {
                            pathfinderFitness -= 1;
                        }
                    }
                }
            }
            
            /*       END OF PATHFINDER UPDATE       */
            
        }
        /*       END OF SIMULATION LOOP       */
        
        if (report)
        {
            doBroadcast(reportString);
        }
        
        // and return some measure of fitness:
        return fmax(0.00000001, pathfinderFitness);
    }
    
    
    // wraps a position around a preset boundary (toroidal world)
    void applyBoundary(double& positionVal)
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
     void applyBoundary(double& positionVal)
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
    double calcDistanceSquared(double fromX, double fromY, double toX, double toY)
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
     double calcDistanceSquared(double fromX, double fromY, double toX, double toY)
     {
         double diffX = fromX - toX;
         double diffY = fromY - toY;
         
         return ( diffX * diffX ) + ( diffY * diffY );
     }*/
    
    // calculates the angle between two agents
    double calcAngle(double fromX, double fromY, double fromAngle, double toX, double toY)
    {
        double Ux = 0.0, Uy = 0.0, Vx = 0.0, Vy = 0.0;
        
        Ux = (toX - fromX);
        Uy = (toY - fromY);
        
        Vx = cosLookup[(int)fromAngle];
        Vy = sinLookup[(int)fromAngle];
        
        int firstTerm = (int)((Ux * Vy) - (Uy * Vx));
        int secondTerm = (int)((Ux * Vx) + (Uy * Vy));
        
        return atan2(firstTerm, secondTerm) * 180.0 / cPI;
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
    
    void doBroadcast(std::string data)
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
};


/*! User-defined configuration struct -- methods on this struct are called at
 various useful points during initialization of an EA.
 */
template <typename EA>
struct configuration : public abstract_configuration<EA>
{
    //! Called to generate the initial EA population.
    void initial_population(EA& ea)
    {
        generate_ancestors(mkv_random_individual(), get<POPULATION_SIZE>(ea), ea);
    }
};


//! Evolutionary algorithm definition.
typedef evolutionary_algorithm<
circular_genome<int>,
mkv_smart_mutation,
pathfinder_fitness,
configuration,
recombination::asexual,
generational_models::death_birth_process< >
> ea_type;

//! Stats recording event.
template <typename EA>
struct pathfinder_stats : record_statistics_event<EA>
{
    pathfinder_stats(EA& ea) : record_statistics_event<EA>(ea), _df(get<OUT_FILE_NAME>(ea))
    {
        _df.add_field("update")
        .add_field("mean_generation")
        .add_field("mean_fitness")
        .add_field("max_fitness");
    }
    
    virtual ~pathfinder_stats()
    {
    }
    
    virtual void operator()(EA& ea)
    {
        using namespace boost::accumulators;
        accumulator_set<double, stats<tag::mean> > gen;
        accumulator_set<double, stats<tag::mean, tag::max> > fit;
        
        for(typename EA::population_type::iterator i=ea.population().begin(); i!=ea.population().end(); ++i)
        {
            gen((*i)->generation());
            fit(static_cast<double>(ea::fitness(**i)));
        }
        
        _df.write(ea.current_update())
        .write(mean(gen))
        .write(mean(fit))
        .write(max(fit))
        .endl();
    }
    
    datafile _df;
};

//! Record video event.
template <typename EA>
struct record_video_event : periodic_event<VIDEO_PERIOD, EA>
{
    
    record_video_event(EA& ea) : periodic_event<VIDEO_PERIOD,EA>(ea)
    {
    }

    void operator()(EA& ea)
    {
        typename EA::individual_type& ind = analysis::find_dominant(ea);
        typename EA::rng_type rng(get<FF_RNG_SEED>(ind));
        ea.fitness_function().record(ind,rng,ea);
    }
};

/*! Define the EA's command-line interface.
 */
template <typename EA>
class cli : public cmdline_interface<EA>
{
public:
    virtual void gather_options()
    {
        // markov network options
        add_option<MKV_DESC>(this);
        add_option<MKV_UPDATE_N>(this);
        add_option<MKV_GATE_TYPES>(this);
        add_option<MKV_INITIAL_GATES>(this);
        add_option<MKV_REPR_INITIAL_SIZE>(this);
        add_option<MKV_REPR_MAX_SIZE>(this);
        add_option<MKV_REPR_MIN_SIZE>(this);
        add_option<GATE_INPUT_LIMIT>(this);
        add_option<GATE_INPUT_FLOOR>(this);
        add_option<GATE_OUTPUT_LIMIT>(this);
        add_option<GATE_OUTPUT_FLOOR>(this);
        add_option<GATE_HISTORY_LIMIT>(this);
        add_option<GATE_HISTORY_FLOOR>(this);
        add_option<GATE_WV_STEPS>(this);
        
        // fitness function options
        add_option<OUT_FILE_NAME>(this);
        add_option<VIDEO_PERIOD>(this);
        
        // ea options
        add_option<REPRESENTATION_SIZE>(this);
        add_option<POPULATION_SIZE>(this);
        add_option<REPLACEMENT_RATE_P>(this);
        add_option<MUTATION_PER_SITE_P>(this);
        add_option<MUTATION_DELETION_P>(this);
        add_option<MUTATION_DUPLICATION_P>(this);
        add_option<MUTATION_UNIFORM_INT_MAX>(this);
        add_option<RUN_UPDATES>(this);
        add_option<RUN_EPOCHS>(this);
        add_option<CHECKPOINT_OFF>(this);
        add_option<CHECKPOINT_PREFIX>(this);
        add_option<RNG_SEED>(this);
        add_option<RECORDING_PERIOD>(this);
    }
    
    virtual void gather_events(EA& ea)
    {
        add_event<pathfinder_stats>(this, ea);
        add_event<record_video_event>(this, ea);
    };
};
LIBEA_CMDLINE_INSTANCE(ea_type, cli);
