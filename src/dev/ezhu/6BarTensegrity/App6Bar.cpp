/*
 * Copyright © 2012, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 * All rights reserved.
 *
 * The NASA Tensegrity Robotics Toolkit (NTRT) v1 platform is licensed
 * under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0.
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific language
 * governing permissions and limitations under the License.
*/

/**
 * @file App6Bar.cpp
 * @brief Contains the definition function main() for App6Bar
 * which builds a 6 bar tensegrity structure defined in YAML or through tgCreator
 * @author Edward Zhu
 * $Id$
 */

// This application
// For yaml model builder
//#include "../../../yamlbuilder/TensegrityModel.h"
// For tgCreator
#include "models/sixBarModel.h"
// This library
#include "core/terrain/tgBoxGround.h"
#include "core/terrain/tgImportGround.h"
#include "core/tgModel.h"
#include "core/tgSimulation.h"
#include "core/tgSimViewGraphics.h"
#include "core/tgWorld.h"
// Bullet Physics
#include "LinearMath/btVector3.h"
// The C++ Standard Library
#include <iostream>
#include <fstream>
#include <string>
#include <math.h>
#include <stdlib.h>
// Controller
#include "controllers/T6RollingController.h"

#define PI 3.14159

/**
 * The entry point.
 * @param[in] argc the number of command-line arguments
 * @param[in] argv argv[0] is the executable name
 * @param[in] argv argv[1] is the path of the YAML encoded structure
 * @return 0
 */
int main(int argc, char** argv)
{
    // create the ground and world. Specify ground rotation in radians
    const double yaw = 0.0;
    const double pitch =0.0*PI/180;
    const double roll = 0.0;

    double sf = 10;

    // ---------------------------------------------------------------------------------
    // Import Ground
    // ---------------------------------------------------------------------------------
    // Set ground parameters
    // btVector3 orientation = btVector3(yaw, pitch, roll);
    // const double friction = 1;
    // const double restitution = 0.0;
    // btVector3 origin = btVector3(0.0, 0.0, 0.0);
    // const double margin = 0.05;
    // const double offset = 0.5;
    // const double scalingFactor = 1;
    // // const double scalingFactor = sf*1000;
    // const int interp = 0;
    // const bool twoLayer = false;
    //
    // // Configure ground characteristics
    // const tgImportGround::Config groundConfig(orientation, friction, restitution,
    //     origin, margin, offset, scalingFactor, interp, twoLayer);
    //
    // // Get filename from argv
    // // std::string filename_in = argv[1];
    // std::string filename_in = "./lunar_terrain_8_16_17.txt";
    //
    // // Check filename
    // if (filename_in.find(".txt") == std::string::npos) {
    //     std::cout << "Incorrect filetype, input file should be a .txt file" << std::endl;
    //     exit(EXIT_FAILURE);
    // }
    //
    // //Create filestream
    // std::fstream file_in;
    //
    // // Open filestream
    // file_in.open(filename_in.c_str(), std::fstream::in);
    //
    // // Check if input file opened successfully
    // if (!file_in.is_open()) {
    //     std::cout << "Failed to open input file" << std::endl;
    //     exit(EXIT_FAILURE);
    // }
    // else {
    //     std::cout << "Input file opened successfully" << std::endl;
    // }
    // tgImportGround* ground = new tgImportGround(groundConfig, file_in);

    // ---------------------------------------------------------------------------------
    // Box ground
    // ---------------------------------------------------------------------------------
    const tgBoxGround::Config groundConfig(btVector3(yaw, pitch, roll));
    tgBoxGround* ground = new tgBoxGround(groundConfig);

    // ---------------------------------------------------------------------------------
    // Parse input arguments
    // ---------------------------------------------------------------------------------
    float psi, theta, phi;
    std::string log_name;

    if (argc == 2) {
        psi = 0;
        theta = 0;
        phi = 0;
        log_name = argv[1];
    }
    else if (argc == 4) {
        // Initial yaw
<<<<<<< HEAD
        psi = atoi(argv[1]);
        // Initial pitch
        theta = atoi(argv[2]);
=======
        psi = atof(argv[1]);
        // Initial pitch 
        theta = atof(argv[2]);
>>>>>>> 954b7c74d8ab39f6d2596b3df7086c598f5b0ce3
        // Initial roll
        phi = atof(argv[3]);
    }
    else if (argc == 5) {
        // Initial yaw
<<<<<<< HEAD
        psi = atoi(argv[1]);
        // Initial pitch
        theta = atoi(argv[2]);
=======
        psi = atof(argv[1]);
        // Initial pitch 
        theta = atof(argv[2]);
>>>>>>> 954b7c74d8ab39f6d2596b3df7086c598f5b0ce3
        // Initial roll
        phi = atof(argv[3]);
        // File to write to
        log_name = argv[4];
    }
    else {
        psi = 0;
        theta = 0;
        phi = 0;
    }
    std::cout << "Initializing model with yaw: " << psi << ", pitch: " << theta << ", and roll: " << phi << std::endl;
    if (!log_name.empty()) {
        std::cout << "Writing to file: " << log_name << std::endl;
    }
    else {
        std::cout << "No log file specified, data will not be logged" << std::endl;
    }

    // double gravity = 9.81*sf;
    double gravity = 1.62*sf;
    const tgWorld::Config config(gravity); // gravity, dm/sec^2
    tgWorld world(config, ground);

    // create the view
    const double timestep_physics = 0.0001; // seconds
    const double timestep_graphics = 1.f/60.f; // seconds
    tgSimViewGraphics view(world, timestep_physics, timestep_graphics);

    // create the simulation
    tgSimulation simulation(view);

    // create the models with their controllers and add the models to the simulation
    // Use yaml model builder
    //TensegrityModel* const myModel = new TensegrityModel(argv[1]);

    // Use tgCreator
    // sixBarModel* const myModel = new sixBarModel();
    sixBarModel* const myModel = new sixBarModel(psi,theta,phi);

    // Define path for controller
    // int *pathPtr;
    // int path[] = {2, 15, 13, 0, 5, 7, 10}; // Repeat unit is [15, 13, 0, 5, 7, 10]
    // int pathSize = sizeof(path)/sizeof(int);
    // pathPtr = path;

    // Define thrust magnitude and period
    btVector3 initVel;
    double initVel_x = 0*sf;
    double initVel_y = 2*sf;
    double initVel_z = 2*sf;
    initVel.setX(initVel_x);
    initVel.setY(initVel_y);
    initVel.setZ(initVel_z);
    double thrustDist = 2*sf;

    // Configure the controlller
    // const T6RollingController::Config controllerConfig(gravity, "face", 2, log_name);
    // const T6RollingController::Config controllerConfig(gravity, "path", pathPtr, pathSize, log_name);
    const T6RollingController::Config controllerConfig(gravity, "thrust", initVel, thrustDist, log_name);
    // const T6RollingController::Config controllerConfig(gravity, "dr", btVector3(100,0,-100), log_name);

    // Create the controller
    //tensionSensor* const tension_sensor = new tensionSensor();
    T6RollingController* const rollingController = new T6RollingController(controllerConfig);

    // Attach controller to the model
    myModel->attach(rollingController);

    // Add the model to the world
    simulation.addModel(myModel);

    // Run the simulation
    simulation.run();

    // teardown is handled by delete
    return 0;
}
